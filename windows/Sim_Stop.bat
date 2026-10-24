@echo off
title Hentikan simulasi ROS 2
echo Menghentikan semua simulasi ROS 2 / Gazebo di WSL...
wsl -d Ubuntu-26.04 -- bash -c "pkill -INT -f 'ros2 launch' ; sleep 3 ; pkill -f 'gz sim' ; pkill -f rviz2 ; pkill -f rosbridge ; exit 0"
echo Selesai.
timeout /t 3 >nul
