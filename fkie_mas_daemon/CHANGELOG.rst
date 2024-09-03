^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_daemon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2024-09-2)
------------------
* fkie_mas_daemon: fixed --force parameter.
* fkie_mas_daemon: use ROS_DOMAIN_ID environment to calculate websocket port in ROS1 and ROS2.
* Contributors: Alexander Tiderko

3.0.5 (2024-07-24)
------------------
* fkie_mas_daemon: added dynamic-reconfigure.py script
* Contributors: Alexander Tiderko

3.0.4 (2024-07-19)
------------------
* fkie_mas_daemon: changed return type of ros.launch.changed uri
* Contributors: Alexander Tiderko

3.0.2 (2024-07-15)
------------------
* fkie_mas_daemon: fixed warning about invalid ros name in subscriber node
* fkie_mas_daemon: fixed forward latched messages
* Contributors: Alexander Tiderko

3.0.0 (2024-07-05)
------------------
* fkie_mas_daemon: replaced crossbar by websocket
* fkie_mas_daemon: changed kill signal to SIGTERM to stop nodes in ROS2
* Contributors: Alexander Tiderko

2.0.0 (2024-01-24)
------------------
* fkie_mas_daemon: new version based on fkie_multimaster
* Contributors: Alexander Tiderko
