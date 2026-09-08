#!/usr/bin/env python3
"""FK probe: 发布各 waypoint 关节角，读取 tool0 在 table_link 下的坐标，用于标定定点抓取 waypoint。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = ['joint1','joint2','joint3','joint4','joint5','joint6','gripper_left','gripper_right']

# 初值，待 FK 标定后回填真值
WAYPOINTS = {
  'HOME':        [0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.025],
  'A_APPROACH':  [0.0, -1.0,  1.0,  0.0, -0.6,  0.0,  0.025, 0.025],
  'A_GRASP':     [0.0, -1.4,  1.4,  0.0, -1.0,  0.0,  0.025, 0.025],
  'LIFT':        [0.0, -1.0,  1.0,  0.0, -0.6,  0.0,  0.025, 0.025],
  'B_APPROACH':  [0.6, -1.0,  1.0,  0.0, -0.6,  0.0,  0.025, 0.025],
  'B_PLACE':     [0.6, -1.4,  1.4,  0.0, -1.0,  0.0,  0.025, 0.025],
  'LIFT2':       [0.6, -1.0,  1.0,  0.0, -0.6,  0.0,  0.025, 0.025],
}


class FKProbe(Node):
    def __init__(self):
        super().__init__('fk_probe')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('FKProbe ready; probing %d waypoints' % len(WAYPOINTS))

    def probe(self, name, q):
        js = JointState()
        js.name = JOINT_NAMES
        js.position = [float(v) for v in q]
        js.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(js)
        # 等 transform 在 buffer 中出现（查 latest，不要用 now() 以免 extrapolation）
        tf = None
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('table_link', 'tool0', rclpy.time.Time())
                break
            except Exception:
                pass
        if tf is None:
            self.get_logger().warn('%s -> tf unavailable' % name)
            return
        t = tf.transform.translation
        self.get_logger().info('%s -> tool0(table_link) = (%.3f, %.3f, %.3f)' % (name, t.x, t.y, t.z))

    def run(self):
        for name, q in WAYPOINTS.items():
            self.probe(name, q)
        self.get_logger().info('FK probe finished')


def main():
    rclpy.init()
    n = FKProbe()
    n.run()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
