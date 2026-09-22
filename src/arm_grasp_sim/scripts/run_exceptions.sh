#!/usr/bin/env bash
# 实验三 异常场景验收批跑（实验要求三.6 / 四.4-四.6）
# 逐个场景启动一次仿真, 等状态机写出 summary, 归档 task_log.json / 识别日志 / 全节点日志。
#
# 用法: bash run_exceptions.sh
# 产物: ~/classify_logs/exceptions/<场景>/{task_log.json,grid_detections.log,launch.log}
#       ~/classify_logs/exceptions/_summary.txt

LOGROOT=$HOME/classify_logs/exceptions
FILTER=${1:-}          # 可选: 只跑名字含该子串的场景(调试用), 如 bash run_exceptions.sh S4
if [ -z "$FILTER" ]; then rm -rf "$LOGROOT"; fi
mkdir -p "$LOGROOT"
: > "$LOGROOT/_summary.txt"

# 场景表: 名称|blocks_preset|z_grasp|超时(s)|期望
SCENARIOS="
S1_empty_grid|empty_grid|0.0446|420|2 skipped_empty + 4 placed
S2_unknown_obj|unknown_obj|0.0446|600|1 skipped_unknown + 5 placed
S3_miss_grasp|miss_grasp|0.115|420|1 grasp_failed + 1 placed + 4 skipped_empty(该预设只放2件,任务不中断)
S4_unreachable|unreachable|0.005|330|2 unreachable(不重试)
"

cleanup() {
  pkill -9 -f gzserver 2>/dev/null
  pkill -9 -f gzclient 2>/dev/null
  pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f classify_grasp_server 2>/dev/null
  pkill -9 -f classify_task_node 2>/dev/null
  pkill -9 -f vision_classifier 2>/dev/null
  pkill -9 -f grid_mapper 2>/dev/null
  pkill -9 -f "ros2 topic echo" 2>/dev/null
  sleep 3
}

source /opt/ros/humble/setup.bash
source $HOME/arm_grasp_ws/install/setup.bash
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export GAZEBO_PLUGIN_PATH=$HOME/arm_grasp_ws/install/gazebo_grasp_plugin/lib/gazebo_grasp_plugin:$GAZEBO_PLUGIN_PATH

echo "$SCENARIOS" | while IFS='|' read -r NAME PRESET ZG TMO EXPECT; do
  [ -z "$NAME" ] && continue
  if [ -n "$FILTER" ]; then
    case "$NAME" in *"$FILTER"*) ;; *) continue ;; esac
  fi
  OUT=$LOGROOT/$NAME
  mkdir -p "$OUT"
  echo "================ $NAME (preset=$PRESET z_grasp=$ZG) ================"
  cleanup
  rm -f $HOME/classify_logs/task_log.json

  ros2 launch arm_grasp_sim classify_sim.launch.py \
      blocks_preset:=$PRESET z_grasp:=$ZG > "$OUT/launch.log" 2>&1 &
  LP=$!
  # 识别结果日志: 等 ROS daemon/话题就绪后再起(立即起会撞上 launch 而 rclpy 上下文被关)。
  # 用 probe_grid.py 抓 /grid_detections 的"反投影世界坐标 + 类别 + 分数"。
  ( sleep 34; python3 /tmp/probe_grid.py 8 > "$OUT/grid_detections.log" 2>&1 ) &
  EP=$!

  T0=$(date +%s); DONE=no
  while [ $(( $(date +%s) - T0 )) -lt $TMO ]; do
    if [ -f $HOME/classify_logs/task_log.json ] && \
       grep -q '"summary"' $HOME/classify_logs/task_log.json; then DONE=yes; break; fi
    sleep 5
  done
  EL=$(( $(date +%s) - T0 ))

  pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f gzserver 2>/dev/null
  kill $EP 2>/dev/null
  pkill -9 -f "ros2 topic echo" 2>/dev/null
  sleep 5
  cp $HOME/classify_logs/task_log.json "$OUT/task_log.json" 2>/dev/null
  # 各节点状态日志(launch 合并 stdout)
  grep -E "\[state\]|\[task\]|夹空|不可达|DONE|skipped|unreachable|GRASP_MISSED" \
      "$OUT/launch.log" > "$OUT/states.log" 2>/dev/null

  printf '%-16s preset=%-12s z_grasp=%-7s summary=%s 用时=%ss  期望: %s\n' \
      "$NAME" "$PRESET" "$ZG" "$DONE" "$EL" "$EXPECT" >> "$LOGROOT/_summary.txt"
  echo "---- $NAME summary=$DONE 用时=${EL}s ----"
  python3 -c "
import json,sys
try:
    d=json.load(open('$OUT/task_log.json'))
    print(json.dumps(d[-1], ensure_ascii=False))
except Exception as e:
    print('读取失败:', e)
"
done
echo "=========== 全部场景结束 ==========="
cat "$LOGROOT/_summary.txt"
