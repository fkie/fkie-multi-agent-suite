# Changelog

## 1.2.1 - 07.03.2024

Added save all files on close editor tabs or quit gui
Limit built-in terminal text update to 10hz
Modified processing of uint8[] message datatypes to send only the first 10 bytes. Changed default window size for topic statistics.

## 1.2.0 - 01.03.2024

Added panel to change log level 
Show info if daemon version is lower then gui's version
Prevent waiting for the timeout if daemon is not available
Fix transport of uint8[] datatypes in echo node

## 1.1.1 - 28.02.2024

Added tree view to editor
Added host color to editor
Improved visualization for recursive search results in editor
Show update info if new version of the gui is available.
Improved launch parameter dialog.

Fixed: delete provider
Fixed: do not show shutdown dialog on install update

## 1.1.0 - 21.02.2024

Added extend search for multiple keywords / OR / AND in nodes, topics, params and services
Added visualization for warnings in provider panel
Added settings to change font size globally and for terminal

Changed host color assignment to map hash instead of using remainder
Improved time difference calculation
Hide bottom panel on close of last terminal
Fixed: remove discovered provider on disconnect from discoverer
Fixed: disabled unregister for ROS2 nodes
Fixed: open terminal for sync time
Fixed: find service definition in ros2

## 1.0.0 - 13.02.2024

Add first version of Node Manager
