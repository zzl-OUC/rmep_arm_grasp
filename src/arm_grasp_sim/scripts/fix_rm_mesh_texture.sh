#!/usr/bin/env bash
# 修复 robomaster_description 网格贴图引用名写错的问题。
# ---------------------------------------------------------------------------
# 现象：gzclient 里底盘/臂不渲染（只剩桌面、料盒与夹爪 box visual）。
# 根因：7 个 .dae（arm_1 / arm_2 / arm_2_bar_1 / arm_2_bar_2 / arm_base /
#       endpoint_bracket / gripper_base）的 <init_from> 写成 `EP_all_01_b_png`
#       （点被写成下划线），而目录里的实际文件叫 `EP_all_01_b.png`。
#       OGRE 取不到贴图会连带整个材质组不出图。
# 这些 .dae 属于 src/robomaster_ros 这个 **submodule**，改动不会随父仓库上传，
# 所以用本脚本在克隆后一次性补上（幂等，可重复执行）。
# ---------------------------------------------------------------------------
set -u
WS="${1:-$HOME/arm_grasp_ws}"
for D in "$WS/src/robomaster_ros/robomaster_description/meshes" \
         "$WS/install/robomaster_description/share/robomaster_description/meshes"; do
  if [ ! -f "$D/EP_all_01_b.png" ]; then
    echo "跳过（目录或 png 不存在）: $D"; continue
  fi
  if [ -e "$D/EP_all_01_b_png" ] || [ -L "$D/EP_all_01_b_png" ]; then
    echo "已存在: $D/EP_all_01_b_png"
  else
    ln -s EP_all_01_b.png "$D/EP_all_01_b_png" && echo "已补符号链接: $D/EP_all_01_b_png"
  fi
done
