#!/bin/bash
# 5 轮抓取验收（前台）
pkill -f "[g]zserver" 2>/dev/null; pkill -f "[g]zclient" 2>/dev/null
pkill -f "grasp_sim.launch" 2>/dev/null
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash 2>/dev/null
cd /home/underwater/arm_grasp_ws
source install/setup.bash 2>/dev/null
export GAZEBO_PLUGIN_PATH=/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:/home/underwater/arm_grasp_ws/install/gazebo_version_helpers/lib:$LD_LIBRARY_PATH

LOG=/tmp/sim_accept5.log
rm -f $LOG
echo "==== launch start $(date) ====" | tee $LOG
ros2 launch arm_grasp_sim grasp_sim.launch.py > $LOG 2>&1 &
LPID=$!
echo "launch pid=$LPID"

echo "==== wait for controllers + block ====" | tee -a $LOG
READY=0
for i in $(seq 1 60); do
  C=$(timeout 5 ros2 control list_controllers 2>/dev/null | grep -c "active")
  B=$(timeout 5 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: block}" 2>/dev/null | grep -cE "success[=:] ?[Tt]rue")
  echo "wait $i: controllers_active=$C block_ready=$B" | tee -a $LOG
  if [ "$C" -ge 3 ] && [ "$B" -ge 1 ]; then READY=1; break; fi
  sleep 2
done
echo "READY=$READY" | tee -a $LOG
if [ "$READY" = 1 ]; then
  echo "==== unpause physics ====" | tee -a $LOG
  timeout 10 ros2 service call /unpause_physics std_srvs/srv/Empty 2>&1 | tee -a $LOG
  sleep 2
  echo "==== send goal cycles=5 ====" | tee -a $LOG
  ros2 run arm_grasp_sim send_grasp_goal.py 5 2>&1 | tee -a /tmp/goal_accept5.log
else
  echo "NOT READY, skip goal" | tee -a $LOG
fi

sleep 20
echo "==== killing launch ====" | tee -a $LOG
kill $LPID 2>/dev/null
sleep 2
pkill -f grasp_sim.launch.py 2>/dev/null
pkill -f spawn_entity 2>/dev/null
pkill -f gazebo_ros 2>/dev/null
pkill -f gzserver 2>/dev/null
echo "==== DONE ====" | tee -a $LOG
echo "############ PER-STEP BLOCK LOG ############"
grep -E '\[LOG' $LOG
echo "############ CYCLE RESULT ############"
grep -E 'CYCLE [0-9]+ 成功|CYCLE [0-9]+ 轨迹完成但|距目标|GraspCycle 结果|success|message' $LOG /tmp/goal_accept5.log
echo "############ goal_accept5.log ############"
cat /tmp/goal_accept5.log
