#!/usr/bin/env bash
# Start the LD-60 plant. Extra arguments go to ros2 launch, e.g.:  ./run_plant.sh gz_gui:=false
source /opt/ros/lyrical/setup.bash
source "$HOME/plant_ws/install/setup.bash"
export GALLIUM_DRIVER=d3d12
# stop anything left from a previous run (either project) so Gazebo and the ports are free
pkill -f 'ros2 launch' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
sleep 1
mkdir -p "$HOME/.ld60_logs"
ros2 launch ld60_plant plant.launch.py "$@" 2>&1 | tee "$HOME/.ld60_logs/plant_launch.log"
