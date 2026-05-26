^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_pylib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.7.0 (26.05.2026)
------------------
* Changed callback group for services
* Fixed code warnings

5.6.1 (29.04.2026)
------------------
* fixed SelfEncoder to include values with 0

5.6.0 (02.04.2026)
------------------
* Updated SubscriberFilter to reset stats for echo topic

5.5.0 (01.04.2026)
------------------
* added types for ros.path.get_list
* disabled watchdog check of the websocket

5.4.0 (25.03.2026)
------------------
* Added watchdog for websocket connection
* Add debug output to find blocking states

5.2.2 (23.03.2026)
------------------
* copied xml for ros1 to xml_ros1
* fixed read default and launch arguments for included files

5.2.1 (18.03.2026)
------------------
* fixed searching for included files in yaml
* fixed determine the arguments of included xml files

5.2.0 (16.03.2026)
------------------
* Include and real path are now two different attributes of the launch include file.

5.1.0 (26.02.2026)
------------------
* Added event for subscriptions to websocket

5.0.0 (05.12.2025)
------------------
* Added new types for lifecycle and composable nodes
* Fixed shutdown service clients if they are failed

4.6.0 (24.11.2025)
------------------
* Added id to RosTopic and RosService

4.5.0 (11.11.2025)
------------------
* Added requester to callback arguments if requester is in signature

4.4.2 (10.09.2025)
------------------
* Fixed determination of the port based on the ROS domain ID for ROS1

4.4.1 (28.05.2025)
------------------
* Added method for determining the child process of a given pid
* Replace paths in arguments when searching for included files
* Improved resolve arguments in xml launch files

4.4.0 (23.05.2025)
------------------
* Added new warning group id
* Added warnings attribute to LaunchContent

4.3.0 (02.05.2025)
------------------
* Added timer_period to launch interface of the node

4.2.2 (23.04.2025)
------------------
* Updated interface of LaunchPublishMessage to use QoS on start a new publisher

4.2.1 (28.02.2025)
------------------
* Fixed split string arrays in parameter and publisher dialogs by a comma, but not if the comma is inside '"'

4.2.0 (30.01.2025)
------------------
* Added parameter to set the length of the arrays for topic echo
* Added error report for list parameters
* Check parameter after value was set

4.1.1 (24.01.2025)
------------------
* Fixed set array parameter

4.1.0 (23.01.2025)
------------------
* Added descriptor to parameter
* Changed error handling in parameter interface

4.0.0 (21.01.2025)
------------------
* Added get_services and get_topics to websocket interface
* Changed get_node_list interface
* Added checks for undefined parameter and list
* Increased maximum size of a message sent via a web socket
* Added read-only parameter to file items
* Skip empty fields in send messages

3.2.0 (12.01.2025)
------------------
* Fixed call service while get/set ros2 parameter

3.1.0 (09.01.2025)
------------------
* Changed RosTopic structure
* Added IncompatibleQos to topic information
* Added LifeCycle to RosNode
* Added python3-netifaces dependency
* Fixed RosParameter

3.0.0 (2024-07-05)
------------------
* fkie_mas_pylib: replaced crossbar by websocket
* fkie_mas_pylib: fixed issue #1 with yaml dependency
* Contributors: Alexander Tiderko

2.0.0 (2024-01-24)
------------------
* fkie_mas_pylib: new version based on fkie_multimaster
* Contributors: Alexander Tiderko
