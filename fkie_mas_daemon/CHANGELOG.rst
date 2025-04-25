^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_daemon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.3.8 (25.04.2025)
------------------
* Fixed: launch nodes with args if it contains find-pkg-share

4.3.7 (23.04.2025)
------------------
* Use QoS of available topics on start a new publisher
* Fixed reloading of python launch files that use or modify environment variables

4.3.6 (10.04.2025)
------------------
* Added parameter change detection on reload file
* Fixed update of available screens
* Fixed call service, it was brocken after added action handling

4.3.5 (03.04.2025)
------------------
* Added more info to clear logs reply

4.3.4 (02.04.2025)
------------------
* Added support for action feedback subscription and send_goal call
* Added handle of proprietary service call for '%s/logger_list' to get list of all available logger names
* Fixed start nodes with complex executables, like 'ruby $(which gz) sim'
* Fixed launch configuration with OpaqueFunction

4.3.3 (28.03.2025)
------------------
* Handle set_env and unset_env of the launch files

4.3.2 (14.03.2025)
------------------
* Workaround for launch-prefix in jazzy

4.3.1 (06.03.2025)
------------------
* Use own method to get included files in XML format

4.3.0 (01.03.2025)
------------------
* Replaced get_participants service by latched participants topic for communication between daemon and discovery

4.2.4 (28.02.2025)
------------------
* Fixed split string arrays in parameter and publisher dialogs by a comma, but not if the comma is inside '"'
* Fixed launch if used find-pkg-share

4.2.3 (10.02.2025)
------------------
* fixed unload not existing files

4.2.2 (05.02.2025)
------------------
* fixed call service with useNow parameter

4.2.1 (03.02.2025)
------------------
* fixed start subscriber without qos (ROS2)

4.2.0 (30.01.2025)
------------------
* Added parameter to set the length of the arrays for topic echo
* Added error report for list parameters
* Check parameter after value was set

4.1.2 (28.01.2025)
------------------
Start subscriber using qos parameter

4.1.1 (24.01.2025)
------------------
Added test node for parameter
Fixed: get message struct data if sequence has defined length
Fixed: do not use 'now' for galactic version

4.1.0 (23.01.2025)
------------------
Added 'now' to publisher
Added descriptor to parameter
Propagate errors in parameter interface to the GUI

4.0.0 (21.01.2025)
------------------
* added new message ros.launch.get_message_types
* fixed kill node if None was given as signal
* added get_services and get_topics to websocket interface
* fixed kill all screens on shutdown

3.5.0 (14.01.2025)
------------------
* Added read-only parameter to FileItems that are reported to Websocket

3.4.0 (12.01.2025)
------------------
* changed communication with mas discovery
* fixed call service, which stops rclpy.spin() after call

3.3.2 (10.01.2025)
------------------
* added stop for execute process and visualization for screen processes
* fixed: show screens of not running nodes e.g. ExecuteProcess, see issue #4

3.3.1 (09.01.2025)
------------------
* fixed daemon for galactic

3.3.0 (08.01.2025)
------------------
* fixed call ros2 service

3.1.2 (2024-10-11)
------------------
* fkie_mas_daemon: fixed detection of nodelet manager for nodelets.

3.1.1 (2024-09-17)
------------------
* fkie_mas_daemon: fixed system node determination in ROS2.
* Contributors: Alexander Tiderko

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
