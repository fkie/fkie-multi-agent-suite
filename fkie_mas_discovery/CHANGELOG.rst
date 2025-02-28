^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_mas_discovery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.2 (28.02.2025)
------------------
* Added RMW_IMPLEMENTATION=rmw_fastrtps_cpp in code

4.0.1 (03.02.2025)
------------------
* fixed discovery name if ROS_IP is set

4.0.0 (21.01.2025)
------------------
* added get_services and get_topics to websocket interface

3.1.0 (12.01.2025)
------------------
* changed communication with mas daemon

3.0.1 (2024-07-16)
------------------
* fkie_mas_discovery: fixed start mas discovery in different networks, broken after switch to websockets
* Contributors: Alexander Tiderko

3.0.0 (2024-07-05)
------------------
* fkie_mas_discovery: replaced crossbar by websocket
* fkie_mas_discovery: addressed issue #2
* Contributors: Alexander Tiderko

2.0.0 (2024-01-24)
------------------
* fkie_mas_discovery: new version based on fkie_multimaster
* Contributors: Alexander Tiderko
