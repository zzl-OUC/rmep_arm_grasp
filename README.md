# RoboMaster EP 机械臂定点抓取（仿真 + 真机）

机器人集成小组项目Ⅰ · 小组实验：机械臂定点抓取。
固定取物点 A 抓取目标物，搬运到固定放置区 B 释放。不考视觉定位。

- 仿真：Windows 11 + WSL2 (Ubuntu 22.04) + ROS 2 Humble + Gazebo Classic 11 + ros2_control
- 真机：RoboMaster EP（机械臂小车，仅作静态载体不移动），ap 热点直连（车 = 192.168.2.1）
- 结果：仿真 5/5 成功，真机 5/5 成功（视频见 `videos/`）

## 仓库结构

```
src/
  arm_grasp_interfaces/    GraspCycle action 定义（仿真/真机共用同一接口）
  arm_grasp_sim/           仿真 + 真机全部任务代码
    launch/grasp_sim.launch.py      一键启动仿真（Gazebo+控制器+抓取节点）
    urdf/arm_grasp.urdf.xacro       EP 机械臂+底盘模型（臂杆碰撞体已按基线决策移除，见下方注意事项）
    urdf/block.urdf                 目标方块（红色）
    worlds/empty_grasp.world        桌面场景（update_rate=2500，RTF≈2.2）
    config/controllers.yaml         ros2_control 控制器配置
    config/grasp_real.yaml          真机点位参数（A/B/安全高度，仿真→真机只改这里）
    scripts/grasp_controller.py     仿真抓取节点（GraspCycle action server）
    scripts/grasp_controller_real.py 真机抓取节点（同一 action 接口，底层走 EP 驱动 move_arm/gripper）
    scripts/grasp_interface.py      验收客户端（cycles=5，打印每轮判定）
    scripts/send_grasp_goal.py      发送抓取目标（cycles 可调）
    scripts/run_accept5.sh          仿真 5 连抓验收一条龙
    scripts/rm_offline_test.sh      真机离线联调全流程（中文引导，逐步确认）
    scripts/rm_workspace_probe.py   真机工作空间探测
    scripts/rm_arm_diag.sh          真机诊断
    scripts/patch_robomaster_sdk.sh DJI SDK 三处补丁（重装 SDK 后必须重跑）
  gazebo_grasp_plugin/     第三方抓取吸附插件（gazebo_grasp_plugin，in-tree 拷贝）
  gazebo_version_helpers/  上述插件的依赖
  robomaster_ros/          DJI EP ROS 2 驱动（git submodule: jeguzzi/robomaster_ros）
videos/                    仿真/真机 5 连抓演示视频
logs/real_machine/         真机 5 连抓 CSV 轨迹日志（state/traj/cycle 事件，全部 SUCCESS）
```

## 环境搭建

```bash
# ROS 2 Humble + Gazebo Classic + ros2_control 按官方文档安装后：
git clone --recurse-submodules <本仓库> && cd arm_grasp_ws
# DJI robomaster Python SDK（GitHub master）+ 依赖 + 三处补丁：
pip install numpy-quaternion av qrcode "numpy<2"
bash src/arm_grasp_sim/scripts/patch_robomaster_sdk.sh
colcon build && source install/setup.bash
export GAZEBO_MODEL_PATH=$PWD/src/robomaster_ros:$GAZEBO_MODEL_PATH   # 视觉 mesh 需要
export GAZEBO_PLUGIN_PATH=$PWD/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:$GAZEBO_PLUGIN_PATH
```

## 仿真运行（验收）

```bash
ros2 launch arm_grasp_sim grasp_sim.launch.py        # 一键启动完整系统
ros2 run arm_grasp_sim grasp_interface               # 另开终端：5 连抓验收，>=4/5 通过
# 或一条龙：bash src/arm_grasp_sim/scripts/run_accept5.sh
```

## 真机运行

电脑连接小车热点（RoboMaster 热点，车 = 192.168.2.1，连接后电脑断网属正常），然后：

```bash
bash src/arm_grasp_sim/scripts/rm_offline_test.sh    # 中文引导：连车→夹爪→臂动作→单抓→5 连抓
```

真机点位在 `config/grasp_real.yaml`：A=车头前缘 8cm（臂基座 x=0.20m），B=前缘 3cm（x=0.15m），
安全高度 0.10m。仿真→真机只换设备/通信/位置参数，任务逻辑（GraspCycle 状态机）完全一致。

## 注意事项（踩过的坑）

- **不要再给臂杆加回 mesh 碰撞体**：全部物理验证基线都是在无臂杆碰撞 mesh 下做的，加回会把方块推跑（0/5）。
- 仿真启动时控制器会把臂从 spawn 零位瞬移到 HOME 位（SetModelConfiguration），避免抬臂扫掠的视觉穿模。
- DJI SDK 进程对 SIGTERM 免疫，清理必须 SIGKILL；脚本任何 exit 路径都要清子进程。
- 真机可靠包线 x≤200mm（臂基座坐标），B 点内收即因此；电量 <50% 时臂保持精度下降。
