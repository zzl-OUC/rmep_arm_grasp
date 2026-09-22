#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 分类任务控制节点(状态机)。

流程:
  SCAN   : 订阅 /grid_detections 快照, 取一个未处理且有物体的网格
  GRASP  : 向扩展抓取控制器 /classify_grasp 发送目标(网格号 + 目标料盒)
  PLACE  : 抓取控制器内部完成"抓取->搬运->放置到对应料盒->返回"
  DONE   : 全部网格处理完(空网格/未识别跳过), 输出总结

异常处理(实验要求三.6 / 四.4):
  - 空网格: 本轮快照里完全没有出现过的网格 -> 记 skipped_empty(不中断)
  - 未识别: 检测框不在任何取物网格, 或类别为 unknown / 置信度 < min_score
            -> 记 skipped_unknown(不中断)
  - 不可达: 抓取 action 返回 error_code=1 -> 记 unreachable(不重试, 目标超出可达包络)
  - 抓取失败: action 返回 error_code=2/3 -> 重试 1 次, 再失败记 grasp_failed/motion_failed
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


# 全部取物网格(与 grid_mapper.CELLS / classify_grasp_server.GRIDS 一致)
ALL_GRIDS = ('cell_1', 'cell_2', 'cell_3', 'cell_4', 'cell_5', 'cell_6')

# error_code -> 任务日志里的结果标签
EC_LABEL = {
    1: 'unreachable',     # 运动学/IK 不可达
    2: 'grasp_failed',    # 夹空
    3: 'motion_failed',   # 关节未收敛
    4: 'unknown_target',  # 非法网格/料盒
    5: 'failed',          # 内部异常
    6: 'place_failed',    # 放置未达判定阈值
}
# 确定性失败(重试必然同样失败), 不浪费一个抓取周期
NO_RETRY_CODES = (1, 4)


class ClassifyTaskNode(Node):
    # 状态机
    S_IDLE, S_SCAN, S_GRASP, S_DONE = 'IDLE', 'SCAN', 'GRASP', 'DONE'

    def __init__(self):
        super().__init__('classify_task_node')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('scan_timeout', 8.0)
        self.declare_parameter('scan_max_wait', 300.0)
        # 扫描稳定窗口(s): 进入 SCAN 后至少累积这么久再定计划。
        # 原为硬编码 3.0 —— 实测不足: 方块 spawn(第 26s)后需落稳, 靠桌沿的方块
        # 首帧检出可晚至 spawn+8s, 3s 窗口会把它整轮漏掉。改 6.0 并开放为参数。
        self.declare_parameter('scan_stable', 6.0)
        # 新网格出现时重开稳定窗口的封顶时长(s): 6 个 block 由 6 个并行 spawn 进程
        # 异步注入, 实测最晚一个可比利害关系最晚的检测晚 1s 以上。
        self.declare_parameter('scan_reopen_cap', 20.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/classify_logs'))
        self.declare_parameter(
            'class_bin_map',
            '{"tennis_ball": "bin_1", "bottle": "bin_0"}')
        self.min_score = float(self.get_parameter('min_score').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.scan_max_wait = float(self.get_parameter('scan_max_wait').value)
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
        self.latest = []           # 最近一帧的检测(调试/兼容用)
        # 扫描窗口内累积的观测: grid -> 最新条目; 以及未落格的条目(按位置去重)。
        # ⚠ 不能只取"最后一帧": 方块刚落地/被遮挡时某帧会漏检, 单帧快照会把该网格
        # 整轮误判成空网格(实测 miss_grasp 场景 cell_1 方块落地瞬间漏检 ->
        # placed 只剩 1)。改为窗口内逐格累积, 取每格最新一次观测。
        self._acc = {}
        self._acc_none = {}
        # 扫描稳定窗口(s): 见 __init__ 声明处说明
        self.scan_stable = float(self.get_parameter('scan_stable').value)
        self.scan_reopen_cap = float(self.get_parameter('scan_reopen_cap').value)
        self._seen_grids = set()   # 本轮 SCAN 已出现过的网格
        self.scan_enter_t = time.time()
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
            self.scan_enter_t = time.time()
            self._acc, self._acc_none = {}, {}
            self._seen_grids = set()
            self._pub_state(self.S_SCAN, '开始扫描桌面')

    # ---------- 订阅 ----------
    def _grid_cb(self, msg):
        try:
            items = json.loads(msg.data)
        except Exception:
            return
        self.latest = items
        for it in items:
            g = it.get('grid')
            if g:
                # 逐格保留最新一帧观测(同一格重复出现时后面的覆盖前面的)
                self._acc[g] = it
                # 本轮首次见到的网格说明桌面还没被看全, 重开稳定窗口(有封顶)。
                # 只认「新」网格: 同一格反复出现/消失不重开, 免得视觉抖动定不了计划。
                if g not in self._seen_grids:
                    self._seen_grids.add(g)
                    if (time.time() - self.scan_enter_t) < self.scan_reopen_cap:
                        self.scan_t0 = time.time()
            else:
                # 未落格的检测(不在任何取物网格): 按 10px 粒度去重, 保留最新
                key = (round(float(it.get('u', 0)) / 10.0),
                       round(float(it.get('v', 0)) / 10.0))
                self._acc_none[key] = it

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
            if ((self._acc or self._acc_none) and stable) or \
                    (time.time() - self.scan_t0) > self.scan_timeout:
                self._plan()
        elif self.state == self.S_GRASP:
            pass  # 等 action 结果(异步回调)
        self._timer = self.create_timer(0.2, self._tick)

    def _tick(self):
        if self.state == self.S_SCAN:
            stable = (time.time() - self.scan_t0) >= self.scan_stable
            if (self._acc or self._acc_none) and stable:
                self._plan()
            elif (time.time() - self.scan_t0) > self.scan_timeout:
                # 桌面一直空: 感知链路启动慢, 持续重扫; 仅当总扫描时长超过
                # scan_max_wait(默认 300s)才放弃, 避免像之前 40s 过早结束导致 placed=0。
                if (time.time() - self.scan_enter_t) > self.scan_max_wait:
                    self.get_logger().warn('[task] 桌面持续空超过 %.0fs, 放弃' % self.scan_max_wait)
                    self._finish()
                else:
                    self.scan_t0 = time.time()
                    self.get_logger().info('[task] 桌面空, 继续等待(已扫 %.0fs/%.0fs)' % (time.time() - self.scan_enter_t, self.scan_max_wait))
        elif self.state == self.S_GRASP and self.current_goal is not None:
            g = self.current_goal
            if g.done():
                self._on_grasp_done(g)

    def _plan(self):
        """根据扫描窗口内累积的观测生成待处理网格队列, 逐个进 GRASP。"""
        items = list(self._acc.values()) + list(self._acc_none.values())
        self._acc, self._acc_none = {}, {}
        # 过滤未识别(无类别/低置信度/无网格)
        queue = []
        for it in items:
            grid, cls, score = it.get('grid'), it.get('cls'), it.get('score', 0.0)
            if grid is None:
                self._log({'grid': None, 'cls': cls, 'result': 'skipped_unknown',
                           'reason': '检测框不在任何取物网格', 'score': score})
                continue
            if not cls or cls == 'unknown' or score < self.min_score:
                self._log({'grid': grid, 'cls': cls, 'result': 'skipped_unknown',
                           'reason': ('物体颜色不属任何已知类别(未识别)'
                                      if cls in (None, 'unknown')
                                      else '置信度 %.3f < %.2f' % (score, self.min_score)),
                           'score': score})
                continue
            it['retry'] = 0
            queue.append(it)
        # 确定性顺序: 按 cell 编号升序处理(cell_1 最先)。目的: cell_1 与 cell_2 相邻,
        # 之前 cell_1 总排最后, 被前序搬运/抓取蹭飞(8~17cm)导致夹空失败; 让其最先处理,
        # block_0 在被触碰前即被抓走移走, 后续格不再有邻块可蹭。
        queue.sort(key=lambda it: int(it['grid'].split('_')[1]))
        # 记录空网格: 只有「本轮快照里完全没有出现过的网格」才算空网格。
        # ⚠ seen 必须取所有出现过的网格(含因未识别而进不了 queue 的), 不能只取 queue ——
        # 否则未识别的格子会先记一条 skipped_unknown、再被当成空网格补记一条
        # skipped_empty, 同一格重复计数(2026-09-14 修)。
        seen = {it.get('grid') for it in items if it.get('grid')}
        for g in ALL_GRIDS:
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
        # ⚠ 表外类别绝不静默兜底到 bin_0:
        # 检测模型 / 仲裁器(VLM) 可能吐出 class_bin_map 里没有的名字, 以前 get(...,'bin_0')
        # 会把它**静默放进 bin_0**(错料盒且不留痕)。改为记 skipped_unknown 并跳过。
        if cls not in self.class_bin:
            self.get_logger().warn('类别 %r 不在 class_bin_map 内 -> 记 skipped_unknown' % cls)
            self._log({'grid': it['grid'], 'cls': cls, 'result': 'skipped_unknown',
                       'reason': '类别 %r 不在 class_bin_map 内' % cls})
            self.pending_grids.pop(0)
            self._next()
            return
        bin_id = self.class_bin[cls]
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
        code = int(getattr(res, 'error_code', 0)) if res is not None else 0
        if ok:
            self._log({'grid': it['grid'], 'cls': it['cls'],
                       'bin': res.bin_id if hasattr(res, 'bin_id') else '',
                       'result': 'placed', 'message': res.message})
            self.pending_grids.pop(0)
            self._next()
            return
        label = EC_LABEL.get(code, 'failed')
        msg = res.message if res else 'no result'
        # 重试一次: 仅对可能因位置/接触状态变化的失败(夹空/未收敛/内部)。
        # 不可达(1)/非法目标(4)是确定性失败, 重试必然同样失败, 直接跳过。
        may_retry = code not in NO_RETRY_CODES
        if may_retry and it.get('retry', 0) < 1:
            it['retry'] = it.get('retry', 0) + 1
            self._log({'grid': it['grid'], 'cls': it['cls'], 'result': 'retry',
                       'error_code': code, 'reason': msg})
            self._next()
        else:
            self._log({'grid': it['grid'], 'cls': it['cls'], 'result': label,
                       'error_code': code, 'reason': msg})
            self.pending_grids.pop(0)
            self._next()

    def _finish(self):
        self.state = self.S_DONE

        def _c(name):
            return sum(1 for r in self.results if r.get('result') == name)

        counts = {k: _c(k) for k in
                  ('placed', 'unreachable', 'grasp_failed', 'motion_failed',
                   'place_failed', 'failed', 'skipped_empty', 'skipped_unknown')}
        placed = counts['placed']
        failed = sum(counts[k] for k in ('unreachable', 'grasp_failed',
                                         'motion_failed', 'place_failed', 'failed'))
        skipped = counts['skipped_empty'] + counts['skipped_unknown']
        self._pub_state(self.S_DONE,
                        'placed=%d failed=%d skipped=%d (empty+unknown)' % (placed, failed, skipped))
        self._log({'grid': None, 'cls': None, 'result': 'summary',
                   'placed': placed, 'failed': failed, 'skipped': skipped,
                   'counts': counts})


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
