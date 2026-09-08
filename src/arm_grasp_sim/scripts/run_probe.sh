#!/bin/bash
set +u
pkill -f "[g]zserver"; pkill -f "[X]vfb"; pkill -f "[d]iag_contact.launch"; pkill -f "list_controllers"
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export ROS_LOCALHOST_ONLY=1
Xvfb :99 -screen 0 1024x768x24 -nolisten unix -listen tcp >/tmp/xvfb.log 2>&1 &
sleep 3
source /opt/ros/humble/setup.bash
source /home/underwater/arm_grasp_ws/install/setup.bash
export GAZEBO_MODEL_PATH=/home/underwater/arm_grasp_ws/src/robomaster_ros:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:/home/underwater/arm_grasp_ws/install/gazebo_version_helpers/lib:$LD_LIBRARY_PATH
export DISPLAY=127.0.0.1:99
cd /home/underwater/arm_grasp_ws
LAUNCH_FILE=/home/underwater/arm_grasp_ws/src/arm_grasp_sim/launch/diag_contact.launch.py
ros2 launch $LAUNCH_FILE > /tmp/diag.log 2>&1 &
LAUNCH_PID=$!
echo "launch pid=$LAUNCH_PID"
for i in $(seq 1 90); do
  sleep 5
  if ros2 control list_controllers 2>/dev/null | grep -q "arm_vel_controller.*active"; then
    echo "controllers active after $((i*5))s"; break
  fi
done
sleep 5
echo "===== RUN PROBE ====="
python3 /tmp/probe_contact.py 2>&1 | tee /tmp/probe_out.txt
echo "===== PROBE DONE ====="
kill $LAUNCH_PID 2>/dev/null
pkill -f "[g]zserver"
