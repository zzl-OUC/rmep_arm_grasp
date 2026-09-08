#!/usr/bin/env python3
"""发送 GraspCycle 目标并等待结果（验收用）。

用法: ros2 run arm_grasp_sim send_grasp_goal.py [cycles]
默认 cycles=5。结果打印 success / success_count / total_count / message。
"""
import rclpy
import sys
from rclpy.action import ActionClient
from arm_grasp_interfaces.action import GraspCycle


def main():
    rclpy.init()
    node = rclpy.create_node('grasp_goal_sender')
    client = ActionClient(node, GraspCycle, '/grasp_cycle')
    if not client.wait_for_server(timeout_sec=20.0):
        print('ERROR: /grasp_cycle Action 服务端不可用（grasp_controller 未就绪？）')
        node.destroy_node()
        rclpy.shutdown()
        return
    goal = GraspCycle.Goal()
    goal.cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    print('发送 GraspCycle 目标 cycles=%d ...' % goal.cycles)
    send_fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=30.0)
    handle = send_fut.result()
    if handle is None:
        print('ERROR: 未获得 goal handle')
        node.destroy_node()
        rclpy.shutdown()
        return
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=900.0)
    res = res_fut.result()
    print('===== GraspCycle 结果 =====')
    print('success        = %s' % res.result.success)
    print('success_count  = %d' % res.result.success_count)
    print('total_count    = %d' % res.result.total_count)
    print('message        = %s' % res.result.message)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
