# Changelog

## 4.2.14 - 03.04.2025

6ae788a fixed assign capability group to all affected nodes
1a37ef7 fixed restart nodes dialog. Now all changed nodes should be added to the question.

## 4.2.13 - 02.04.2025

Added support for action feedback subscription and send_goal call
Added handle of proprietary service call for '%s/logger_list' to get list of all available logger names
Added dynamic open and close brackets for xml launch files
Changed tab names (added option to remove namespaces)
Fixed start nodes with complex executables, like 'ruby $(which gz) sim'
Fixed launch configuration with OpaqueFunction
Fixed namespaces of services

## 4.2.12 - 28.03.2025

Added support for set_env and unset_env in launch file
Added view of changes when a new update is available
Fixed expand some type of arrays in echo dialog
Fixed links to included files were not updated when file were changed
Fixed “don't save” was not noted
Fixed editor context menu
Fixed open parameter if host is selected
Fixed version compare in update tab
Fixed version compare in install script
Improved python highlighter
Improved highlighter for ROS2 XML launch files
Improved highlighter for ROS1 XML launch files
Improved ROS1 suggestions in XML launch files
Improved ROS2 suggestions in XML launch files

## 4.2.11 - 21.03.2025

Fixed no visible service names

## 4.2.10 - 21.03.2025

Added check during update whether mas packages are in the workspace
Changed group behavior for topics, services and parameters
No downgrade to prerelease
Fixed: button to open parent file
Fixed: set sync parameter when starting from MAS-GUI, see issue #5
Fixed: open log in a red screen if screen not available

## 4.2.9 - 14.03.2025

Workaround for launch-prefix in jazzy
Fixed open humburger menus

## 4.2.8 - 14.03.2025

Updated readme
Added workflows to create debian packages for jazzy, humble, noetic and gui
Fixed docker build with new version using debian packages

## 4.2.7 - 06.03.2025

Improved ctrl+click to open included files
Use own method to get included files in XML format
Added highlighting for ros2 xml launch file
Added xml proposals for ros2
Force reload current path if package list was refreshed
Changed default state of visible node details

## 4.2.6 - 01.03.2025

Prepand RMW_IMPLEMENTATION=rmw_fastrtps_cpp for discovery node
Replaced get_participants service by latched participants topic for communication between daemon and discovery

## 4.2.5 - 28.02.2025

Added update check for latest github version if we don't use appImage
Added update of the debian packages via github
Added script to install debian packages from github
Added mas-gui arguments to update debian packages
Fixed show local nodes with cyclon dds
Fixed split string arrays in parameter and publisher dialogs by a comma, but not if the comma is inside '"'
Fixed launch if used find-pkg-share
Added RMW_IMPLEMENTATION=rmw_fastrtps_cpp in mas-discovery code

## 4.2.4 - 25.02.2025

Changed default ttyd port
Fixed ttyd port determination if not default port is used
Added script to create debian packages for all mas packages

## 4.2.3 - 10.02.2025

Extended hint for TTYD Daemon if ttyd is not available
Fixed unload not existing files
Fixed start if ttyd not available
Fixed daemon min version

## 4.2.2 - 05.02.2025

Added: show incompatible QoS warning at group level
Added: show warning if logger is not supported
Changed calculation of the abbreviation of the tab name
Fixed qos incompatibility check for jazzy
Fixed call service with useNow parameter

## 4.2.1 - 03.02.2025

Added SPAM group and capability_group for ros2 nodes
Fixed select current file in explorer of the file editor
Do not save empty parameters in the history of the start dialog
Fixed editor height when a hint is displayed
Fixed start subscriber without qos
Fixed discovery name if ROS_IP is set
Fixed file selection in editor

## 4.2.0 - 30.01.2025

Added parameter to set the length of the arrays for topic echo
Added error report for list parameters
Check parameter after value was set
Fixed sort items in package explorer
Fixed launch ros2 nodes with launch prefix
Fixed update boolean parameter state in the gui

## 4.1.3 - 28.01.2025

Fixed search in topics panel
Convert numbers to valid values before set parameter
Start subscriber with qos parameters
Added warning about wrong websockets version (in daemon)

## 4.1.2 - 24.01.2025

Fixed: show topics with different message type on same topic

## 4.1.1 - 24.01.2025

Improved publisher panel
Updated call service panel
Changed: add new tabs to the end
Fixed auto-reload after save
Fixed grouping of parameters
Fixed array parameter and edit in the parameter panel
Fixed: use also subscriber to create an echo dialog
Fixed search in create publisher und call service panel

## 4.1.0 - 23.01.2025

Added 'now' and stop button to publish panel
Use clipboard text in parameters only until new line
Added descriptor to parameter
Propagate errors in parameter interface to the GUI
Show 'install' warning in editor only if realpath is in install
Added dialog to select ros domain id on run external apps
Added simple logger support for ros2 nodes
Fixed quick start provider with different ros domain id

## 4.0.0 - 21.01.2025

Changed get_node_list interface
Added get_services and get_topics to websocket interface
Added the option to update to prerelease
Skip empty fields during communication with websocket
Added checks for undefined fields
Added context menu to searchBar, topics and services
Improves the update behavior of the service and topic panels
Added new message ros.launch.get_message_types
Depending on the version difference, set different colors for the notification about new daemon version
Changed shutdown of daemon and subscriber node
Increased maximum size of a message sent via a web socket
Fixed kill node if None was given as signal
Fixed kill all screens on shutdown

## 3.0.13 - 16.01.2025

Fixed open external terminal with different terminal

## 3.0.12 - 16.01.2025

Added a warning for the parameter list if there are many ROS nodes.
No automatic update of the parameter list when opening
Increased timeout to get parameter list
Added QoS visualization to each topic
Added package proposals to launch file editor

## 3.0.11 - 16.01.2025

Topic panel revised
fixed: ros1 interface to mas gui

## 3.0.10 - 15.01.2025

Changed json visualization to react18-json-view
Fixed topic visualization when one topic is part of another
Fixed starting the external terminal with ROS_DOMAIN_ID of the provider

## 3.0.9 - 14.01.2025

Changed: do not use 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' for mas-discovery

## 3.0.8 - 14.01.2025

Added visualization for read-only files and files in the installation folder
Added forced update of ros nodes
Changed the process for updating the ros status; is performed in a separate thread
Fixed deselection of nodes after starting/stopping
Fixed unrecognized file saves
Fixed close dialog when exiting the application
Fixed: restore external window before focus
Fixed: closes app although an item has been selected

## 3.0.7 - 13.01.2025

Ignore last changes

## 3.0.6 - 13.01.2025

Hot fix for render error

## 3.0.5 - 13.01.2025

Fixed: do not deselect node after start/stop
Fixed: fixed nodes cleanup if screen and launch info are gone

## 3.0.4 - 12.01.2025

Updated minimum daemon version
Fixed the problem where the automatic updates were stopped

## 3.0.3 - 10.01.2025

Reverted some type changes to fix the closing behavior of the app

## 3.0.2 - 10.01.2025

Added stop for execute process and visualization for screen processes
Bump version of some depended libraries

## 3.0.1 - 09.01.2025

Added dependecy python3-netifaces for mas_pylib
Updated docker files
Fixed: show screens of not running nodes e.g. ExecuteProcess, see issue #4
Fixed: daemon for galactic

## 3.0.0 - 08.01.2025

Changed: app icon and removed js build options
Changed: translated JavaScript to TypeScript
Changed: parameter panel
Changed: topic info structure
Added: show incompatible qos information in topics panel
Added: kill all parents, to handle the case if respawn script is used
Added: visualization and control for lifecycle ros2 nodes
Added: show the content of the parameter file in ros2 configuration details
Added: file selection dialog if a node is defined in multiple launch files
Added: warning if executable for a loaded node was not found
Added: parsing for included files for loaded ros2 python launch files
Added: search for child processes of the screen to kill the correct process
Added: signal handling to ros2 nodes
Updated: visualization of composable node and nodes with same GUID
Updated: the determination of the composable node for ROS2
Updated: readme and depended licenses
Fixed: call ros2 service
Fixed: error visualization if save file failed
Fixed: wheel scroll in parameter selection dialog
Fixed: reduced forced updates after ros service calls in ros2
Fixed: expand/collapse node groups only on icon or double click
Fixed: do not switch to node details panel on node updates
Fixed: start daemon nodes with default configuration
Bump version of systeminformation dependency

## 2.7.0 - 09.12.2024

Added: remember network id in connect dialog
Added: environment VITE\_ variables to join at start
Added: docker configuration files
Added: option to adjust local time to the robot
Added: auto-connect on start if daemon is running
Changed: open behavior of the connect dialog
Removed: sass dependency
Fixed: warning "findDOMNode is deprecated"
Fixed: X-Tree deprecation warning
Fixed: multiple runs for update and domain id check
Fixed: onClose warning
Fixed: import error in jazzy
Fixed: "invalid escape sequence" for python 3.12
Fixed: undefined ttyd port
Fixed: parameter handling for rolling
Fixed: open screens of a provider

## 2.6.10 - 07.11.2024

Changed description for setup ssh config in password dialog
Fixed: reporting changed paths for symlinks
Fixed: crash on open self included files
Fixed: save changes and not lost editor focus
Fixed: open screens if exactly two screens are available for a node
Fixed: load included files without arguments
Fixed: error while run remote node
Fixed: build error

## 2.6.9 - 17.10.2024

Added search bar to topic echo panel
Fixed: do not force start nodelet manager if it is already running and not selected to restart

## 2.6.8 - 14.10.2024

fixed: computer name contains dash which is not allowed in node names, issue #3

## 2.6.7 - 11.10.2024

Added script to bump version of the gui and daemon
fkie_mas_daemon: fixed detection of nodelet manager for nodelets

## 2.6.6 - 10.10.2024

Fixed: load start parameter of older version
Fixed: wait until all kill_on_stop fired before start nodes to ensure the nodelet manager start first.

## 2.6.5 - 10.10.2024

Changed connect dialog, replaced checkbox by button to save parameters
Fixed: restart if kill_on_stop is set

## 2.6.4 - 09.10.2024

Improved auto update
Small style and naming changes

## 2.6.3 - 09.10.2024

Fixed: host color for package explorer

## 2.6.2 - 09.10.2024

Start nodelet manager before nodelets also if not selected
Added handling for nm/kill_on_stop parameter
Force reload file and drop local changes
Catch errors while read ssh configuration
Fixed: selection of root item in explorer of the editor
Fixed: display of multiple screens in details panel
Fixed: font size of the node names with namespaces

## 2.6.1 - 08.10.2024

Load configuration files into the editor only on demand, show loading indicators
Expand all items from root to the selected item in explorer of the editor
Fixed: scrollbar in package explorer of the editor
Fixed: read old start configurations

## 2.6.0 - 07.10.2024

Use ssh keys instead of store credentials in keyring
Added copy button in connect dialog
Added further connect configuration parameter
On load launch file dialog: update the last open path each time the path is changed
Open the parent file and go to the location where the file is included, differentiate between multiple includes of the same file
Show all occurrences of included files in package explorer
All notifications can now be closed
Close launch file editor on double ESC
Changed settings dialog to settings and about tab, moved the icons to bottom right
Fixed: exception in package explorer with equal subdirectory names
Fixed: parse NaN values in topic messages (use JSON5)

## 2.5.11 - 27.09.2024

Added lint to configuration
Changed handling of collapse/expand in echo panel
Fixed: do not scroll control panel in echo tab

## 2.5.10 - 18.09.2024

Handling of capability groups revised
Added launch parameter to the node details view
Fixed: system node determination in ROS2

## 2.5.9 - 17.09.2024

Added parameter to move the group parameter in the namespace tree
Changed sort order for capability groups
Fixed: determination of group parameters for nodes if group parameter are defined globally

## 2.5.8 - 12.09.2024

Fixed rendering error caused by last change

## 2.5.7 - 12.09.2024

Fixed restart nodes on changed binary notification

## 2.5.6 - 12.09.2024

Do not display notifications about changed binary files in external windows

## 2.5.5 - 10.09.2024

Fixed: grouping by capability group, use group parameter in the namespace of the node
Fixed: node template for XML launch files
Fixed: parameter panel for ROS2
Fixed: open terminal for update

## 2.5.4 - 10.09.2024

Changed position of the provider terminal button

## 2.5.3 - 09.09.2024

Fixed: shortcuts in terminal

## 2.5.2 - 05.09.2024

Fix: eliminate -- inside XML comments

## 2.5.1 - 04.09.2024

Added host colors to editor, terminal, echo, publisher and service caller panels

## 2.5.0 - 04.09.2024

Service panel revised
Added optional ROS_MASTER_URI prefix while starting system nodes
Use ROS_DOMAIN_ID environment variable for network id while start ros1 system nodes
Added ros master uri to connect dialog
Added expand parameter groups on search
Replaced refresh buttons in parameter and topic panel
Fixed: detection of discovery state in ros2
Fixed: websocket ports calculation depending on network id and ros master uri
Fixed: hot reloading after change .jsx files
Fixed: undo of comment/uncomment text with "--" in xml files

## 2.4.5 - 02.09.2024

Optimized settings dialog

## 2.4.4 - 28.08.2024

NodeDetailsPanel cleaned up
Hide some parameter in web version
Terminal revised: new shortcuts, switchable search bar
_ Ctr+{+, - , 0}: change font size
_ Ctr+f: enable search bar
\_ Escape in search bar: close search bar

---

\_ Ctrl+Shift+C: copy selected text to clipboard \* Ctrl+D: close terminal
Version of the dependencies pumped up
Fixed: popout capability in browser
Fixed: warning while open screen

## 2.4.3 - 28.08.2024

Added selection dialog on actions with multiple nodes
Parameter tab revised
Fixed: progress bar in nodes tab
Fixed: Uncaught TypeError: styled_default is not a function

## 2.4.2 - 27.08.2024

Fixed: do not open external window on middle mouse click
Fixed: open terminal as tab or as external window

## 2.4.1 - 27.08.2024

Expand node groups on filter nodes
Expand topic groups on filter topics
Changed behaviour to show/hide extended topic info in topics panel
Prevent open as tab and external window at same time
Fixed: start of dynamic reconfiguration dialog
Fixed: open node launch configuration in external window

## 2.4.0 - 26.08.2024

Added: close tabs with middle mouse
Added: ctrl+shift+c to copy text in the terminal window
Added: options to open terminals (screen, log) in external window by default
Added: options to open subscriber in external window by default
Added: tooltips for log levels in logger panel
Added: configurations for open tab locations
Select nodes tab on close tab if it is in the same set

## 2.3.2 - 21.08.2024

Added option to open editor in external window by default
Reduced control buttons using key modifiers
Run refresh nodes callback also if not connected to provider
Prepand always "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" while start ros2 discovery node
Fixed: detection of running discovery node in ros2

## 2.3.1 - 26.07.2024

Add dialog to save changed file on close external editor
Added shift as button modification to open editor in a new window
Fixed: do not reload own saved files

## 2.3.0 - 25.07.2024

Added: open editor in external window
Select nodes tab if it is in the same set as the closed editor
Fixed: focus problem if a screen with rosbag play is open
Fixed: selection in draggable dialogs

## 2.2.0 - 24.07.2024

Added dynamic reconfigure for ROS1 nodes

## 2.1.9 - 23.07.2024

Automatically switch to LOG if the screen fails
Updated screen and log icons for tabs
Changed calculation of the abbreviation of the tab name
Disable Ctrl+C for log tab to be able to abort waiting log
Added option Ctrl+Shift+I to format XML files
Replace "--" in xml comments
Fixed: open screen on double click for ros2 nodes
Fixed: shortcuts for toggle comment and open command palette in editor

## 2.1.8 - 19.07.2024

Handle double click on a node:

```text
    - running: open screen
        + shift: open screen in external terminal
        + ctrl: stop the node

    - not running: open log
        + shift: open log in external terminal
        + ctrl: start the node
```

Updated readme

## 2.1.7 - 19.07.2024

Changed load button to primary color
Fixed navigation with shift+{home, end} in autocomplete inputs
Fixed issue with not imported class in package explorer

## 2.1.6 - 19.07.2024

fixed broken dialogs

## 2.1.5 - 19.07.2024

Close reload launch file alert if file was reloaded
Close "file changed" alert if the node was started manually
Added autoFocus to searchBar
Changed screen icon
Focus Terminal and Log on open

## 2.1.4 - 18.07.2024

Changed screen and log icons
Scroll to the end on log open

## 2.1.3 - 18.07.2024

Changed position of node control buttons to left side
Changed search bars to case insensetive
Changed position of external apps button
Removed table header for publisher and subscriber in node details panel
Fixed: warning in package manager
Fixed: security alerts of dependencies
Fixed: start of daemon with different daemon id in ros2

## 2.1.2 - 17.07.2024

Autohide menu bar
Moved copy button to menu in package explorer
Print error message if package was not found on shift+double click
Fixed: application icon for appImage
Fixed: get package list on connect

## 2.1.1 - 17.07.2024

Fixed: recursive search if some file models are not found
Fixed: error handling if some websocket methods are not registered
Fixed: host line height
Fixed: update parameter panel after first start

## 2.1.0 - 16.07.2024

Switched from webpack to electron-vite builder
Use abbreviations for closable tabs
Fixed: start mas discovery in different networks

## 2.0.2 - 15.07.2024

Added icons to overflow menu for topics and services in node details panel
Added info to service details
Added title to publisher, echo and service call panel
Changed show subscribers first in node details panel
Changed position of the control buttons in package explorer
Changed: use abbreviations for closable tabs
Improved host visualization
Fixed: copy message in publish dialog
Fixed: show start button on disconnected providers

## 2.0.1 - 08.07.2024

Fixed: call service in ros1 and ros2
Fixed: set logger level in ros1

## 2.0.0 - 05.07.2024

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
