#!/bin/bash
echo "This runs Kanaloa RobotX"
roslaunch master master.launch &
cd ~/sdgps_v5.3
build/main sylphase-usbgpsimu2 --antenna-position '[0.185, 0.13, 0]' ! tracker ! kf2 --decimation 10 ! listen-solution-tcp 1234 &
scripts/solution_ros_bridge _port:=1234 _child_frame_id:=/ins;
