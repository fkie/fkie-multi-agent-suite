# Changelog

## 2.0.0 05.07.2024

Replaced crossbar by websocket
Reduced count of rerenders while update ROS state

## 1.4.4 - 06.05.2024

Show logging panel on click on snackbar message
Moved 'show details' button to description header in Log panel

Fixed: problems with load shared libraries while start in some environments
Fixed: start nodes with respawn argument in ros2

## 1.4.3 - 30.04.2024

Added left border to layout
Added: change detection for ros2

Fixed: visibility handling of bottom border on closing of last tab
Fixed: binary change detection if binary is a link

## 1.4.1 - 25.04.2024

Show delay to connected hosts
Added screen name to node details info
Added log paths to node details info
Added selected nodes count to node details info

Fixed: unique keys warnings
Fixed: recursive search in text editor
Fixed: crash after select node by click on launch file

## 1.4.0 - 24.04.2024

Added configuration parameter to show floating buttons, default false
Copy full file path on double click on the included file in editor explorer
Changed node id generation
Improved performance in topic view, node details panel and log/screen terminals
Improves the starting behavior, send ros.daemon.ready at an interval
Updated dependencies

Fixed: visualization error on not implemented get loggers in ros2
Fixed: shows nodelets/manager not only after a launch file was loaded
Fixed: get diagnostics in ros2
Fixed: nodes are not deselected after start or stop them in ros2
Fixed: remember view state (JSON tree) of received topic in topic echo

## 1.3.8 - 10.04.2024

Improved stop/kill nodes for ROS2
Updated dependencies, replaced deprecated xterm... by @xterm/...

Fixed not close the app after host shutdown
Fixed ros2 start/join with ROS DOMAIN ID different then zero

## 1.3.7 - 04.04.2024

Improved style of the message/service input dialog
Show complete launch path in the node details panel
Added: copy topic type on double click in topic panel

Fixed: parse/visualization of ros2 messages
Fixed: service call when exactly 1 non-default parameter value is given
Fixed: service call with boolean param
Fixed: a lot of eslint warnings

## 1.3.6 - 22.03.2024

Update daemon version on join if already connected
Changed extensions for launch file detection

Fixed: freeze on close while connecting

## 1.3.5 - 21.03.2024

Added button to show all nodes visible to each host
Added location to node info
Improved version detection of the daemon

Fixed: size of the clear button in the search bar
Fixed: local providers are displayed twice after the start
Fixed: visualization of the error message in snack bar

## 1.3.4 - 18.03.2024

Fix selection order in HostTreeView when using ctrl

## 1.3.3 - 15.03.2024

Added "!" as "not" to search bar
Added menu to change parameter type in parameter panel
Added link symbol to help menu
Improved service call for ros1
Check only once after start for new version
Do not ask for quit app if no providers available
Show no long info if service input is empty

Fixed: set type while set/change parameter in ros1
Fixed: log level view for c++ nodes
Fixed: history update in launch dialog

## 1.3.2 - 13.03.2024

Fixed: values in the parameter dialog of the launch file cannot be changed
Fixed: waning about invalid URL

## 1.3.1 - 13.03.2024

Reduced rate of diagnostic messages
Revert debounce on built-in terminal display
Fixed open log in external terminal
Fixed rate of latched messages

## 1.3.0 - 12.03.2024

Renamed tabs: Hosts -> Nodes, Providers -> Hosts
Revised start procedure and the start dialog
Added shortcuts Ctrl+{+,-} to change global font size

Fixed echo for topics with arrays in ros2

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

Add first version of MAS GUI
