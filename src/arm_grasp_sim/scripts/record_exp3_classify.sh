#!/usr/bin/env bash
# 实验三 验证视频录制便利脚本（第二个终端，仿真/真机 launch 起好后再跑本脚本）
#
# 前置: 已 source ROS + 工作区, 且相机话题在出图(/top_camera/image_raw)。
# 默认输出: ~/videos/exp3_classify.mp4
#   - 仿真 6/6 验收: ros2 launch arm_grasp_sim classify_sim.launch.py 起好后跑本脚本
#   - 真机:         ros2 launch arm_grasp_sim classify_real.launch.py 起好后跑本脚本
#
# 停止: Ctrl-C -> 节点退出时释放 VideoWriter 并打印总帧数。
set -e

SRC_ROS="${1:-/opt/ros/humble/setup.bash}"
WS="${2:-$HOME/arm_grasp_ws}"
OUT="${3:-$HOME/videos/exp3_classify.mp4}"

source "$SRC_ROS"
source "$WS/install/setup.bash" 2>/dev/null || true

# WSL 软渲染: 相机出图必须
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe

echo "[record] 录制到 $OUT  (Ctrl-C 停止)"
ros2 run arm_grasp_sim classify_recorder.py \
  --ros-args -p out:="$OUT" -p fps:=30 -p draw:=true
