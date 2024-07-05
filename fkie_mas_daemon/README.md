MAS Daemon is an instance which allows the MAS Gui a remote access to configuration files . Through the daemon the launch file can be edited, loaded and the containing nodes executed by MAS GUI.

The daemon instance is usually launched be MAS Gui through SSH connection. After that the MAS Gui communicates with daemon using WebSockets on port 35430+(ROS_DOMAIN_ID), __35685+(NetworkId) with ROS1__. These ports should be open in the firewall.

Beside offering remote configuration access to MAS Gui the daemon supports many other features, e.g. system monitoring, forwarding diagnostic messages or auto start/load of launchfiles.

The configuration is stored at *$HOME/.config/ros.fkie/mas_daemon.yaml* and can be changed through MAS Gui for each host.
