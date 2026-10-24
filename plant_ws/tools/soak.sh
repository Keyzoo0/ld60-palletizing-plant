#!/usr/bin/env bash
# Soak test with an RTF log: tools/soak.sh [plant_check args...]   (plant must already be running)
#   ~/.ld60_logs/plant_check.log   progress / PASS / FAIL
#   ~/.ld60_logs/rtf.log           real-time factor every 20 s
source /opt/ros/lyrical/setup.bash
source ~/plant_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
: > ~/.ld60_logs/rtf.log
(
  while true; do
    # cpu = seconds for a fixed Python loop (~0.07 on a cool laptop): shows host throttling next to the RTF
    cpu=$(python3 -c 'import time;t=time.time();sum(i*i for i in range(2*10**6));print("%.2f" % (time.time()-t))')
    echo "$(date +%H:%M:%S) cpu=$cpu $(timeout 10 gz topic -e -n 1 -t /world/plant/stats 2>/dev/null | grep real_time_factor)" >> ~/.ld60_logs/rtf.log
    sleep 20
  done
) &
logger=$!
ros2 run ld60_plant plant_check "$@" 2>&1 | tee ~/.ld60_logs/plant_check.log
kill $logger
