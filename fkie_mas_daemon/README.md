MAS Daemon is an instance which allows the MAS Gui a remote access to configuration files . Through the daemon the launch file can be edited, loaded and the containing nodes executed by Node Manager GUI.

The daemon instance is usually launched be MAS Gui through SSH connection. After that the MAS Gui communicates with daemon using [WAMP](https://wamp-proto.org/). It needs a running [WAMP Router](https://wamp-proto.org/implementations.html#routers). We use crossbar.

Beside offering remote configuration access to MAS Gui the daemon supports many other features, e.g. system monitoring, forwarding diagnostic messages or auto start/load of launchfiles.

The configuration is stored at *$HOME/.config/ros.fkie/mas_daemon.yaml* and can be changed through MAS Gui for each host.
