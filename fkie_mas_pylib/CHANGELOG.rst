^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_pylib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
