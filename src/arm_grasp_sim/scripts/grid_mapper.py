#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 网格映射节点: 订阅 /detections (Detection2DArray),
把检测框中心(像素)映射到桌面取物网格编号 cell_1..cell_6, 发布 /grid_detections。

世界->像素 标定表来自 table_grid_4c2b.world 的 desk_layout 模型几何:
  cell_1 ( 0.1785, 0.0650) cell_2 ( 0.1271, 0.1412)
  cell_3 ( 0.0460, 0.1844) cell_4 (-0.0460, 0.1844)
  cell_5 (-0.1271, 0.1412) cell_6 (-0.1785, 0.0650)
相机 top_camera 位于 (0,0,1.0) 俯视, hfov=0.95rad, 800x640。
投影关系(俯视, pitch=+90deg): u = cx + fx*y_w, v = cy + fy*x_w。
如实际朝向不同, 改 swap_axes 参数即可, 不用重算。

发布: GridDetection {grid_id: 'cell_N', class_id, score} (std_msgs/String JSON 也行,
这里用简单自定义话题: /grid_detections 是 vision_msgs/Detection2DArray,
其中 bbox.center.position.x 复用为网格编号 int)。
为保持接口简单: 发布 std_msgs/String JSON 列表, 每项 {grid, cls, score, u, v}。
"""
import json
import math
import rclpy
import numpy as np
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String


# 世界坐标网格中心(x,y), 与 world 文件一致 (全向布局: 正前/左前/右前/正后/左后/右后)
CELLS = {
    "cell_1": ( 0.2250,  0.0000),
    "cell_2": ( 0.1591,  0.1591),
    "cell_3": ( 0.1591, -0.1591),
    "cell_4": (-0.2250,  0.0000),
    "cell_5": (-0.1591,  0.1591),
    "cell_6": (-0.1591, -0.1591),
}
CELL_TOL = 0.075   # 网格半径(m), 0.16x0.10 格对角半径 ~0.094, 取 0.075 防重叠


class GridMapper(Node):
    def __init__(self):
        super().__init__('grid_mapper')
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 640)
        self.declare_parameter('hfov', 0.95)
        self.declare_parameter('cam_height', 1.0)
        self.declare_parameter('swap_axes', True)   # u~+Y, v~+X (俯视 pitch=90deg)
        self.declare_parameter('cell_tol', CELL_TOL)
        # ── 真机标定出口（默认空 -> 与仿真行为逐行一致）──────────────────────
        # cells      : JSON {"cell_1": [x, y], ...} 真机实测网格中心(臂基座系, 米)
        # homography : JSON 3x3 行主序(9 个数), 世界(x,y) -> 像素(u,v); 非空则
        #              优先于理想针孔模型(俯视相机用单应更准, 也免测 hfov/高度)。
        #              标定见 scripts/calib_real_grid.py
        self.declare_parameter('cells', '')
        self.declare_parameter('homography', '')
        self.swap = bool(self.get_parameter('swap_axes').value)
        self.tol = float(self.get_parameter('cell_tol').value)
        self.cells = self._parse_cells(self.get_parameter('cells').value) or CELLS
        self.H, self.Hinv = self._parse_homography(
            self.get_parameter('homography').value)
        W = int(self.get_parameter('image_width').value)
        H = int(self.get_parameter('image_height').value)
        hfov = float(self.get_parameter('hfov').value)
        cam_h = float(self.get_parameter('cam_height').value)
        import math as _m
        vfov = 2 * _m.atan(_m.tan(hfov / 2) * H / W)
        self.fx = (W / 2) / _m.tan(hfov / 2)
        self.fy = (H / 2) / _m.tan(vfov / 2)
        self.cx, self.cy = W / 2.0, H / 2.0
        self.cam_h = cam_h
        # 预计算各网格像素中心
        self.cell_px = {}
        for name, (x, y) in self.cells.items():
            u, v = self.world_to_pixel(x, y)
            self.cell_px[name] = (u, v)
        self.sub = self.create_subscription(Detection2DArray, '/detections', self._cb, 10)
        self.pub = self.create_publisher(String, '/grid_detections', 10)
        self.get_logger().info(
            'grid_mapper 就绪 映射=%s 网格数=%d fx=%.1f fy=%.1f 网格像素=%s'
            % ('单应' if self.H is not None else '针孔',
               len(self.cells), self.fx, self.fy,
               {k: (round(u), round(v)) for k, (u, v) in self.cell_px.items()}))

    @staticmethod
    def _parse_cells(raw):
        """JSON 字符串 -> {name: (x, y)}; 空/非法返回 None(调用方回退内置几何)。"""
        if not raw:
            return None
        try:
            d = json.loads(raw) if isinstance(raw, str) else dict(raw)
        except Exception:
            return None
        out = {}
        for k, v in d.items():
            try:
                out[str(k)] = (float(v[0]), float(v[1]))
            except Exception:
                continue
        return out or None

    @staticmethod
    def _parse_homography(raw):
        """JSON 9 个数 -> (H, Hinv); 空/非法/奇异返回 (None, None)。"""
        if not raw:
            return None, None
        try:
            v = json.loads(raw) if isinstance(raw, str) else list(raw)
            H = np.asarray(v, dtype=np.float64).reshape(3, 3)
            Hinv = np.linalg.inv(H)
            return H, Hinv
        except Exception:
            return None, None

    def world_to_pixel(self, x, y):
        """桌面世界坐标 -> 像素。给了 homography 就用它(俯视相机更准), 否则针孔模型。"""
        if self.H is not None:
            p = self.H @ np.array([x, y, 1.0])
            if abs(p[2]) < 1e-12:
                return float('nan'), float('nan')
            return float(p[0] / p[2]), float(p[1] / p[2])
        if self.swap:
            # 俯视 pitch=+90deg 且图像相对世界旋转 180 度(实测标定)
            u = self.cx - self.fx * y / self.cam_h
            v = self.cy - self.fy * x / self.cam_h
        else:
            u = self.cx + self.fx * x / self.cam_h
            v = self.cy + self.fy * y / self.cam_h
        return u, v

    def pixel_to_world(self, u, v):
        """像素 -> 桌面世界坐标 (x, y)。与 world_to_pixel 严格互逆。"""
        if self.Hinv is not None:
            p = self.Hinv @ np.array([u, v, 1.0])
            if abs(p[2]) < 1e-12:
                return float('nan'), float('nan')
            return float(p[0] / p[2]), float(p[1] / p[2])
        if self.swap:
            y = (self.cx - u) * self.cam_h / self.fx
            x = (self.cy - v) * self.cam_h / self.fy
        else:
            x = (u - self.cx) * self.cam_h / self.fx
            y = (v - self.cy) * self.cam_h / self.fy
        return x, y

    def pixel_to_grid(self, u, v):
        """像素 -> 世界 -> 最近网格(距离阈值内), 返回 grid_id 或 None。"""
        x, y = self.pixel_to_world(u, v)
        best, bd = None, 1e9
        for name, (gx, gy) in self.cells.items():
            d = math.hypot(x - gx, y - gy)
            if d < bd:
                best, bd = name, d
        if bd <= self.tol:
            return best
        return None

    def _cb(self, msg):
        out = []
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            score = float(d.results[0].hypothesis.score)
            u, v = d.bbox.center.position.x, d.bbox.center.position.y
            grid = self.pixel_to_grid(u, v)
            # 反投影: 像素 -> 桌面世界坐标。抓取用"实测位置"而非固定格心 ——
            # 前序抓取会把邻近方块碰歪, 按格心盲抓会夹空(曾致 cell_1 dist=0.452 失败)。
            wx, wy = self.pixel_to_world(u, v)
            out.append({'grid': grid, 'cls': cls, 'score': round(score, 3),
                        'u': round(u, 1), 'v': round(v, 1),
                        'x': round(wx, 4), 'y': round(wy, 4)})
        if out:
            m = String()
            m.data = json.dumps(out)
            self.pub.publish(m)


def main():
    rclpy.init()
    node = GridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
