#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 分类任务控制节点(状态机)。

流程:
  SCAN   : 订阅 /grid_detections 快照, 取一个未处理且有物体的网格
  GRASP  : 向扩展抓取控制器 /classify_grasp 发送目标(网格号 + 目标料盒)
  PLACE  : 抓取控制器内部完成"抓取->搬运->放置到对应料盒->返回"
  DONE   : 全部网格处理完(空网格/未识别跳过), 输出总结

异常处理(实验要求三.6 / 四.4):
  - 空网格: 扫描无检测 -> 跳过, 记 skipped_empty
  - 未识别: 检测框无类别或置信度 < min_score -> 跳过, 记 skipped_unknown
  - 不可达: 抓取 action 返回不可达 -> 跳过, 记 unreachable
  - 抓取失败: action result.success=False -> 重试1次, 再失败记 failed 并跳过
  - 所有异常记入 JSON 日志 ~/classify_logs/task_log.json

类别->料盒映射: green_block -> bin_0(绿盒), yellow_block -> bin_1(黄盒)。
"""
import json
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_grasp_interfaces.action import ClassifyGrasp
from std_msgs.msg import String


class ClassifyTaskNode(Node):
    # 状态机
    S_IDLE, S_SCAN, S_GRASP, S_DONE = 'IDLE', 'SCAN', 'GRASP', 'DONE'

    def __init__(self):
        super().__init__('classify_task_node')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('scan_timeout', 8.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/classify_logs'))
        self.declare_parameter(
            'class_bin_map',
            '{"green_block": "bin_0", "yellow_block": "bin_1", '
            ' "red_block": "bin_2", "blue_block": "bin_3"}')
        self.min_score = float(self.get_parameter('min_score').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.log_dir = self.get_parameter('log_dir').value
        _cbm = self.get_parameter('class_bin_map').value
        self.class_bin = (json.loads(_cbm) if isinstance(_cbm, str)
                          else dict(_cbm))
        os.makedirs(self.log_dir, exist_ok=True)

        self._cb = ReentrantCallbackGroup()
        self._action = ActionClient(self, ClassifyGrasp, 'classify_grasp',
                                    callback_group=self._cb)
        self.create_subscription(String, '/grid_detections', self._grid_cb, 10,
                                 callback_group=self._cb)
        self.state_pub = self.create_publisher(String, '/task_state', 10)
        self.latest = []           # 最近一次 /grid_detections 快照
        # 扫描稳定窗口(s): 进入 SCAN 后至少等这么久再取快照,
        # 避免相机刚出图/方块未落稳时只检测到部分网格就开工
        self.scan_stable = 3.0
        self.pending_grids = []    # 待处理网格列表
        self.results = []          # 每网格结果日志
        self.state = self.S_IDLE
        self.scan_t0 = None
        self.current_goal = None
        self.get_logger().info('classify_task_node 就绪 (类别->料盒: %s)' % self.class_bin)
        # 启动 1s 后进入 SCAN
        self.create_timer(1.0, self._kick)

    # ---------- 状态发布 ----------
    def _pub_state(self, s, extra=''):
        m = String()
        m.data = json.dumps({'state': s, 'extra': extra, 't': time.time()})
        self.state_pub.publish(m)
        self.get_logger().info('[task] %s %s' % (s, extra))

    def _kick(self):
        if self.state == self.S_IDLE:
            self.state = self.S_SCAN
            self.scan_t0 = time.time()
            self._pub_state(self.S_SCAN, '开始扫描桌面')

    # ---------- 订阅 ----------
    def _grid_cb(self, msg):
        try:
            self.latest = json.loads(msg.data)
        except Exception:
            pass

    # ---------- 主循环 ----------
    def _log(self, entry):
        self.results.append(entry)
        path = os.path.join(self.log_dir, 'task_log.json')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(self.results, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().warn('日志写失败: %s' % e)

    def spin_once(self):
        """由外部定时器周期调用。"""
        if self.state == self.S_SCAN:
            # 快照稳定(有数据且已过稳定窗口, 或整体超时)
            stable = (time.time() - self.scan_t0) >= self.scan_stable
            if (self.latest and stable) or \
                    (time.time() - self.scan_t0) > self.scan_timeout:
                self._plan()
        elif self.state == self.S_GRASP:
            pass  # 等 action 结果(异步回调)
        self._timer = self.create_timer(0.2, self._tick)

    def _tick(self):
        if self.state == self.S_SCAN:
            stable = (time.time() - self.scan_t0) >= self.scan_stable
            if self.latest and stable:
                self._plan()
            elif (time.time() - self.scan_t0) > self.scan_timeout:
                # 桌面一直空: 可能方块未生成, 重置继续等(最多 5 轮)
                self._empty_rounds = getattr(self, '_empty_rounds', 0) + 1
                if self._empty_rounds >= 5:
                    self._finish()
                else:
                    self.scan_t0 = time.time()
                    self.get_logger().info('[task] 桌面空, 继续等待 (%d/5)' % self._empty_rounds)
        elif self.state == self.S_GRASP and self.current_goal is not None:
            g = self.current_goal
            if g.done():
                self._on_grasp_done(g)

    def _plan(self):
        """根据快照生成待处理网格队列, 逐个进 GRASP。"""
        items = list(self.latest)
        self.latest = []
        # 过滤未识别(无类别/低置信度/无网格)
        queue = []
        for it in items:
            grid, cls, score = it.get('grid'), it.get('cls'), it.get('score', 0.0)
            if grid is None:
                self._log({'grid': None, 'cls': cls, 'result': 'skipped_unknown',
                           'reason': '检测框不在任何取物网格', 'score': score})
                continue
            if not cls or score < self.min_score:
                self._log({'grid': grid, 'cls': cls, 'result': 'skipped_unknown',
                           'reason': '类别缺失或置信度低', 'score': score})
                continue
            it['retry'] = 0
            queue.append(it)
        # 确定性顺序: 按 cell 编号升序处理(cell_1 最先)。目的: cell_1 与 cell_2 相邻,
        # 之前 cell_1 总排最后, 被前序搬运/抓取蹭飞(8~17cm)导致夹空失败; 让其最先处理,
        # block_0 在被触碰前即被抓走移走, 后续格不再有邻块可蹭。
        queue.sort(key=lambda it: int(it['grid'].split('_')[1]))
        # 记录空网格(全部 6 网格中未出现的)
        seen = {it['grid'] for it in queue}
        for g in ('cell_1', 'cell_2', 'cell_3', 'cell_4', 'cell_5', 'cell_6'):
            if g not in seen:
                self._log({'grid': g, 'cls': None, 'result': 'skipped_empty',
                           'reason': '空网格'})
        self.pending_grids = queue
        if not self.pending_grids:
            self._finish()
        else:
            self._pub_state(self.S_SCAN, '计划: %s' %
                            [(i['grid'], i['cls']) for i in self.pending_grids])
            self.state = self.S_GRASP   # 进入抓取态, 停止重复扫描
            self._next()

    def _next(self):
        if not self.pending_grids:
            self._finish()
            return
        it = self.pending_grids[0]
        cls = it['cls']
        bin_id = self.class_bin.get(cls, 'bin_0')
        goal = ClassifyGrasp.Goal()
        goal.grid_id = it['grid']
        goal.class_id = cls
        goal.bin_id = bin_id
        self._pub_state(self.S_GRASP, '%s %s -> %s (retry=%d)'
                        % (it['grid'], cls, bin_id, it.get('retry', 0)))
        fut = self._action.wait_for_server(timeout_sec=5.0)
        if not fut:
            self.get_logger().error('classify_grasp action server 不可用')
            self._log({'grid': it['grid'], 'cls': cls, 'result': 'failed',
                       'reason': 'action server 不可用'})
            self.pending_grids.pop(0)
            return
        self._send_fut = self._action.send_goal_async(goal)
        self._send_fut.add_done_callback(self._goal_sent)
        self.current_item = it

    def _goal_sent(self, fut):
        gh = fut.result()
        if not gh.accepted:
            it = self.current_item
            self.get_logger().warn('goal 被拒: %s' % it['grid'])
            self._log({'grid': it['grid'], 'cls': it['cls'], 'result': 'failed',
                       'reason': 'goal rejected'})
            self.pending_grids.pop(0)
            self.current_goal = None
            self._next()
            return
        self.state = self.S_GRASP
        res_fut = gh.get_result_async()
        self.current_goal = res_fut

    def _on_grasp_done(self, res_fut):
        self.current_goal = None
        it = self.pending_grids[0]
        try:
            res = res_fut.result().result
        except Exception as e:
            res = None
            self.get_logger().error('action 结果异常: %s' % e)
        ok = bool(res and res.success)
        if ok:
            self._log({'grid': it['grid'], 'cls': it['cls'],
                       'bin': res.bin_id if hasattr(res, 'bin_id') else '',
                       'result': 'placed', 'message': res.message})
            self.pending_grids.pop(0)
            self._next()
        else:
            # 抓取失败重试 1 次
            if it.get('retry', 0) < 1:
                it['retry'] = it.get('retry', 0) + 1
                self._log({'grid': it['grid'], 'cls': it['cls'], 'result': 'retry',
                           'reason': res.message if res else 'no result'})
                self._next()
            else:
                self._log({'grid': it['grid'], 'cls': it['cls'], 'result': 'failed',
                           'reason': (res.message if res else 'no result')})
                self.pending_grids.pop(0)
                self._next()

    def _finish(self):
        self.state = self.S_DONE
        placed = sum(1 for r in self.results if r['result'] == 'placed')
        failed = sum(1 for r in self.results if r['result'] == 'failed')
        skipped = sum(1 for r in self.results if r['result'].startswith('skipped'))
        self._pub_state(self.S_DONE,
                        'placed=%d failed=%d skipped=%d (empty+unknown)' % (placed, failed, skipped))
        self._log({'grid': None, 'cls': None, 'result': 'summary',
                   'placed': placed, 'failed': failed, 'skipped': skipped})


def main():
    rclpy.init()
    node = ClassifyTaskNode()
    node.spin_once()
    exe = rclpy.executors.MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
