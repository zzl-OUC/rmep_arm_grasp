#!/usr/bin/env python3
"""定点抓取验收客户端：发送 cycles=5 的 GraspCycle 目标，打印每轮状态与最终判定。

用法（在 sim launch 之后另开终端）：
  ros2 run arm_grasp_sim grasp_interface
判定：success_count >= 4/5 即达到实验验收标准。
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arm_grasp_interfaces.action import GraspCycle


class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_interface')
        self._ac = ActionClient(self, GraspCycle, 'grasp_cycle')

    def run(self, cycles=5):
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('grasp_cycle Action 服务端未启动')
            return
        goal = GraspCycle.Goal(); goal.cycles = cycles
        send = self._ac.send_goal_async(goal,
                                         feedback_callback=self._fb)
        handle = send.result(timeout_sec=30.0)
        if handle is None:
            self.get_logger().error('目标未被接受')
            return
        self.get_logger().info('目标已接受，开始 %d 轮抓取' % cycles)
        result = handle.get_result(timeout_sec=600.0)
        self.get_logger().info('==== 验收结果 ====')
        self.get_logger().info('成功轮数: %d/%d' % (result.result.success_count,
                                                    result.result.total_count))
        self.get_logger().info('判定: %s' % ('PASS' if result.result.success else 'FAIL'))
        self.get_logger().info('说明: %s' % result.result.message)

    def _fb(self, msg):
        fb = msg.feedback
        self.get_logger().info('[feedback] %s  progress=%.2f  gripper_closed=%s'
                               % (fb.current_state, fb.progress, fb.gripper_closed))


def main():
    rclpy.init()
    n = GraspClient()
    try:
        n.run(5)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
