#!/bin/bash
# RoboMaster EP 真机离线联调脚本（ap 热点模式）
# 用法: bash ~/rm_offline_test.sh
set +u
LOG=/home/underwater/rm_offline_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a $LOG) 2>&1

echo "=============================================="
echo " RoboMaster EP 真机联调  日志: $LOG"
echo "=============================================="

source /opt/ros/humble/setup.bash
source /home/underwater/arm_grasp_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RM_NAME=robomaster RM_MODEL=ep

echo "[第0步] 清理上次残留的驱动/控制器进程 ..."
pkill -f "[r]obomaster_ros" 2>/dev/null; pkill -f "[r]obot_state_publisher" 2>/dev/null
pkill -f "[j]oint_state_publisher" 2>/dev/null; pkill -f "[g]rasp_controller_real" 2>/dev/null
sleep 1
pkill -9 -f "[r]obomaster_ros" 2>/dev/null; pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
pkill -9 -f "[j]oint_state_publisher" 2>/dev/null; pkill -9 -f "[g]rasp_controller_real" 2>/dev/null
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1
echo "    清理完成 (残留进程已强杀, ros2 daemon 已重启)"

echo "[第1步] 检查到小车的网络 (ping 192.168.2.1) ..."
if ! ping -c 2 -W 2 192.168.2.1 >/dev/null 2>&1; then
  echo "!! 不通。请确认: 1) 电脑WiFi已连上小车的 RM-XXXX 热点  2) 小车已开机"
  exit 1
fi
echo "    网络 OK"

echo "[第2步] 启动驱动 (conn_type=ap) ..."
ros2 launch robomaster_ros ep.launch conn_type:=ap > /tmp/rm_driver.log 2>&1 &
DRIVER_PID=$!
echo "    驱动进程 PID=$DRIVER_PID，等待连接小车(最多90秒)..."
ok=0
for i in $(seq 1 18); do
  sleep 5
  if ros2 topic list 2>/dev/null | grep -q "/robomaster/arm_position"; then ok=1; break; fi
done
if [ "$ok" != "1" ]; then
  echo "!! 驱动连不上小车。请检查: 1) App里是否完成了机械臂校准 2) 是否只有一台设备连小车热点(手机App先断开)"
  echo "---- 驱动日志最后20行 ----"; tail -20 /tmp/rm_driver.log
  kill $DRIVER_PID 2>/dev/null; pkill -9 -f "[r]obomaster_ros" 2>/dev/null; pkill -9 -f "[j]oint_state_publisher" 2>/dev/null; pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
  exit 1
fi
echo "    驱动已连接，话题 /robomaster/arm_position 出现"

echo "[第3步] 读取机械臂当前位置 ..."
timeout 10 ros2 topic echo --once /robomaster/arm_position 2>/dev/null | grep -E "x:|y:|z:" | head -3
echo "    (如果能看到 x/z 数值，说明臂的反馈正常)"

echo ""
echo "=============================================="
echo " 下面要开始动了。请确认: 机械臂周围无障碍物,"
echo " 你手放在小车电源开关旁边, 随时可断电。"
echo "=============================================="
read -p "确认安全后按回车继续 (Ctrl+C 退出) ..."

echo "[第4步] 夹爪测试: 张开 -> 合拢 ..."
ros2 action send_goal /robomaster/gripper robomaster_msgs/action/GripperControl "{target_state: 1, power: 0.5}" 2>&1 | tail -2
sleep 1
ros2 action send_goal /robomaster/gripper robomaster_msgs/action/GripperControl "{target_state: 2, power: 0.5}" 2>&1 | tail -2
read -p "夹爪动了吗? 动了按回车继续; 没动按 Ctrl+C 退出 ..."

echo "[第5步] 机械臂小动作: 移到 (x=0.15, z=0.10) ..."
ros2 action send_goal /robomaster/move_arm robomaster_msgs/action/MoveArm "{x: 0.15, z: 0.10, relative: false}" 2>&1 | tail -3
read -p "臂动了吗? 动了按回车继续; 没动按 Ctrl+C 退出 ..."

echo "[第6步] 启动抓取控制器 (真机版) ..."
ros2 run arm_grasp_sim grasp_controller_real.py --ros-args --params-file /home/underwater/arm_grasp_ws/src/arm_grasp_sim/config/grasp_real.yaml > /tmp/grasp_real.log 2>&1 &
GC_PID=$!
sleep 5
if ! ros2 action list 2>/dev/null | grep -q "/grasp_cycle"; then
  echo "!! 抓取控制器没起来。日志:"; tail -10 /tmp/grasp_real.log; exit 1
fi
echo "    控制器就绪"

echo ""
echo "[第7步] 单次抓取测试: 请把目标物摆在 A 点(车头前缘正前方8cm)正中"
read -p "摆好后按回车开始抓取 ..."
ros2 action send_goal /grasp_cycle arm_grasp_interfaces/action/GraspCycle "{cycles: 1}" 2>&1 | tail -6
echo ""
read -p "单次抓取结果如何? 成功按回车进入 5 连抓验收; 有问题按 Ctrl+C 退出 ..."

echo "[第8步] 5 连抓验收: 每轮之间需要你把目标物从 B 框摆回 A 点十字正中"
for n in 1 2 3 4 5; do
  echo ""
  read -p "  第 $n/5 轮: 确认目标物已摆在 A 点正中后按回车开始 ..."
  ros2 action send_goal /grasp_cycle arm_grasp_interfaces/action/GraspCycle "{cycles: 1}" 2>&1 | grep -E "success|message" | head -4
done

echo ""
echo "=============================================="
echo " 测试结束。CSV 日志在 ~/grasp_logs/ 下。"
echo " 按回车关闭驱动 ..."
read
kill $GC_PID $DRIVER_PID 2>/dev/null
sleep 1
pkill -9 -f "[r]obomaster_ros" 2>/dev/null; pkill -9 -f "[g]rasp_controller_real" 2>/dev/null
echo "完成。现在可以把 WiFi 切回家里网络, 上线后告诉我一声即可(日志我会自己读)。"
