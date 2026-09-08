#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/underwater/arm_grasp_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/underwater/arm_grasp_ws

OUT=/tmp/accept_result.txt
: > $OUT
echo "=== launch start $(date) ===" >> $OUT

ros2 launch arm_grasp_sim grasp_sim.launch.py >> /tmp/launch.log 2>&1 &
LPID=$!
echo "launch pid=$LPID" >> $OUT

# 等待控制器与 state 服务就绪（launch 内 grasp_ctrl 延迟 15s + 余量）
sleep 40

# 探测 gazebo state 服务可达性（诚实校验的前提）
echo "=== service probe ===" >> $OUT
ros2 service list 2>/dev/null | grep -i gazebo >> $OUT
timeout 12 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: block}" >> $OUT 2>&1 || echo "get_entity_state 探测失败/超时" >> $OUT

# 发送 5 轮抓取目标
echo "=== send goal cycles=5 @ $(date) ===" >> $OUT
ros2 run arm_grasp_sim send_grasp_goal.py 5 >> $OUT 2>&1
echo "=== goal returned @ $(date) ===" >> $OUT

# 汇总每轮真实校验（来自控制器日志）
echo "=== controller verify log ===" >> $OUT
grep -E "方块距目标|CYCLE [0-9]+ 成功|方块未到 B|退化|服务不可达|无有效响应" /tmp/launch.log | tail -40 >> $OUT

# 收尾：杀掉仿真
pkill -f 'ros2 launch arm_grasp_sim' 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f 'gazebo_ros' 2>/dev/null
sleep 2
echo "=== DONE @ $(date) ===" >> $OUT
