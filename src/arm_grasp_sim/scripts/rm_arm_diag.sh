#!/bin/bash
# EP 机械臂诊断: 对比"机器人以为自己到哪了"和"实际到哪了"
set +u
source /opt/ros/humble/setup.bash
source /home/underwater/arm_grasp_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RM_NAME=robomaster RM_MODEL=ep
read_arm() { timeout 8 ros2 topic echo --once /robomaster/arm_position 2>/dev/null | grep -E "x:|z:" | tr '\n' ' '; echo; }

echo "[0] 清理残留进程 ..."
pkill -f "[r]obomaster_ros" 2>/dev/null; pkill -f "[j]oint_state_publisher" 2>/dev/null; pkill -f "[r]obot_state_publisher" 2>/dev/null
sleep 1
pkill -9 -f "[r]obomaster_ros" 2>/dev/null; pkill -9 -f "[j]oint_state_publisher" 2>/dev/null; pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1
echo "    清理完成"

echo "[1] 检查网络 ..."; ping -c 2 -W 2 192.168.2.1 >/dev/null 2>&1 || { echo "!! 不通, 确认WiFi连着小车热点"; exit 1; }
echo "[2] 启动驱动 ..."
ros2 launch robomaster_ros ep.launch conn_type:=ap > /tmp/rm_diag_driver.log 2>&1 &
DPID=$!
for i in $(seq 1 18); do sleep 5; ros2 topic list 2>/dev/null | grep -q "/robomaster/arm_position" && break; done
ros2 topic list 2>/dev/null | grep -q "/robomaster/arm_position" || { echo "!! 驱动没连上"; tail -5 /tmp/rm_diag_driver.log; kill $DPID 2>/dev/null; pkill -9 -f "[r]obomaster_ros" 2>/dev/null; pkill -9 -f "[j]oint_state_publisher" 2>/dev/null; exit 1; }
echo "    已连接"

echo "[3] 当前臂位置(机器人认为的):"; read_arm
echo "[4] 舵机原始值(4路, 1024≈180度):"
timeout 8 ros2 topic echo --once /robomaster/servo_raw_state 2>/dev/null | grep -A5 "value" | head -6

echo ""
echo "接下来做两个相对移动测试, 每次只挪 2cm。观察臂的实际动作。"
read -p "按回车做测试1: 相对前进 +2cm ..."
ros2 action send_goal /robomaster/move_arm robomaster_msgs/action/MoveArm "{x: 0.02, z: 0.0, relative: true}" 2>&1 | grep -E "status|result" | head -2
sleep 2; echo "    机器人认为现在位置:"; read_arm

read -p "按回车做测试2: 相对抬高 +2cm ..."
ros2 action send_goal /robomaster/move_arm robomaster_msgs/action/MoveArm "{x: 0.0, z: 0.02, relative: true}" 2>&1 | grep -E "status|result" | head -2
sleep 2; echo "    机器人认为现在位置:"; read_arm

read -p "按回车做测试3: 绝对位置 (0.15, 0.10) ..."
ros2 action send_goal /robomaster/move_arm robomaster_msgs/action/MoveArm "{x: 0.15, z: 0.10, relative: false}" 2>&1 | grep -E "status|result" | head -2
sleep 2; echo "    机器人认为现在位置:"; read_arm

echo ""
echo "诊断结束。把上面每个读数记下来或直接回网告诉我即可。"
read -p "按回车关闭驱动 ..."
kill $DPID 2>/dev/null
sleep 1
pkill -9 -f "[r]obomaster_ros" 2>/dev/null
