#!/bin/bash

ros2 run fkie_mas_daemon mas-remote-node.py  --name=/mas/_daemon_{HOST} --set_name=false --node_type=mas-daemon --package=fkie_mas_daemon
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run fkie_mas_daemon mas-remote-node.py  --name=/mas/_discovery_{HOST} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery
ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-7681 --command=ttyd --writable --port 7681 bash --pre_check_binary=true

cd ${ROS_WS}/src/fkie-multi-agent-suite/fkie_mas_gui
#ELECTRON_ENABLE_LOGGING=1 npm run dev -- --no-sandbox
ELECTRON_ENABLE_LOGGING=1 ./dist/mas-gui.AppImage --no-sandbox
