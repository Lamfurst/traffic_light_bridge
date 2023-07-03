#!/bin/bash
# Parse command-line argument
# Check whethe to pub all traffic light info
PUB_ALL=$1

# launch reroute node in a new terminal
tmux split-window -h
tmux send-keys "source /home/artemis/Workspace/autowarefoundation/carla-autoware-universe/autoware/install/setup.bash" C-m
tmux send-keys "source /home/artemis/personal_autoware_ws/install/setup.bash" C-m

if [ "$PUB_ALL" == "false" ]; then
    tmux send-keys "ros2 launch launch/trafficlightbridge.launch.xml pub_all:=false" C-m
else
    tmux send-keys "ros2 launch launch/trafficlightbridge.launch.xml" C-m
fi