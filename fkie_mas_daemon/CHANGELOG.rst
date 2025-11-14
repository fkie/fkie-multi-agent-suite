^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_daemon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.7.1 (14.11.2025)
-------------------
* fixed error when loading 'capability groups' for included files
* added test launch files for capability group

4.7.0 (11.11.2025)
-------------------
* report the requester on each launch file action

4.6.11 (30.09.2025)
-------------------
* fixed crash of the daemon on exception while load_launch

4.6.10 (23.09.2025)
-------------------
* Fixed show_ros_log parameter

4.6.9 (16.09.2025)
------------------
* Fixed update lifecycle state of composable nodes
* Improved update ros2 state
* Fixed start node with multiple launch files

4.6.8 (15.09.2025)
------------------
* Fixed load launch files with arguments inside eval block

4.6.7 (10.09.2025)
------------------
* Fixed force refresh for nodes

4.6.6 (09.09.2025)
------------------
* Fixed start node witch multiple launch files

4.6.5 (05.09.2025)
------------------
* added rosout_to_diag script to daemon
* reduced rate of file modification events

4.6.4 (27.08.2025)
------------------
* Fixed update services
* Added additional checks for status changes when Zenoh is enabled

4.6.3 (01.08.2025)
------------------
* Fixed matching the diagnostic color to node witch is a part of another node

4.6.2 (31.07.2025)
------------------
* Implemented purge of all ros logs

4.6.1 (30.07.2025)
------------------
* Improved match of diagnostic messages to node names

4.6.0 (24.07.2025)
------------------
* Added /mas/diagnostic subscription witch forward only state of local nodes

4.5.10 (17.06.2025)
------------------
* Fixed reload files with new added launch arguments

4.5.9 (11.06.2025)
------------------
* Increased timeout for load composed nodes to 5 sec

4.5.8 (10.06.2025)
------------------
* Fixed load parameter of composable nodes
* Added support of YAML(base) structures in the MAS gui
* Added try catch around the get state blocks to avoid an invalid state of the daemon
* Fixed echo of odometry topics
* Fixed detect changes in included yaml files
* Replace environment variables in xacro files

4.5.7 (10.06.2025)
------------------
* Fixed reload launch files with included yaml files
* Fixed load xml launch file with execute process without namespace

4.5.6 (06.06.2025)
------------------
* Fixed loading parameters of composed nodes in xml launch files
* Fixed load python launch files with composable node list
* Fixed call commands with '{data: xyz}'

4.5.5 (04.06.2025)
------------------
* Fixed change detection of node configuration in referenced yaml files

4.5.4 (03.06.2025)
------------------
* Added support for ros state used with zenoh

4.5.3 (28.05.2025)
------------------
* Added script to restart running mas nodes and reload launch files
* Terminate all child processes when shutting down the screens started by mas
* Force update state after kill_screens
* Fixed set args for included launch files

4.5.2 (27.05.2025)
------------------
* Fixed reducing the size of the arrays for displaying the message in the echo tab
* Fixed calculation of the ros message size
* Fixed start mas commands in a screen

4.5.1 (26.05.2025)
------------------
* Fixed start mas nodes in a screen

4.5.0 (23.05.2025)
------------------
* Added source setup.bash on force refresh package list
* Changed screen configuration, added "shell -$SHEL" to source ros setup.bash defined in .bashrc
* Report warnings from ros2 daemon

4.4.8 (19.05.2025)
------------------
* Added autostart option to daemon
* Fixed adding included configuration files for observe

4.4.7 (14.05.2025)
------------------
* Fixed load composable nodes

4.4.6 (08.05.2025)
------------------
* Skip timer on start nodes if shift was pressed
* Fixed load launch files in ros galactic

4.4.5 (06.05.2025)
------------------
* Fixed load with default parameters
* Fixed remove timeouted daemons

4.4.4 (06.05.2025)
------------------
* Fixed exception on unknown hostname

4.4.3 (06.05.2025)
------------------
* Show launch arguments only of the top launch file while load

4.4.2 (05.05.2025)
------------------
* Fixed load parameter of composable nodes

4.4.1 (05.05.2025)
------------------
* Fixed remove outdated daemons

4.4.0 (02.05.2025)
------------------
* Use local addresses if no discovery node available, but a screen for a node was found
* Remove outdated discoverd daemons
* Added the possibility to search for a process of a node to stop

4.3.10 (30.04.2025)
------------------
* Fixed read capability group parameter of composable nodes
* Updated hostname detection for providers

4.3.9 (29.04.2025)
------------------
* Fixed: return real paths for included launch files

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
