### MAS Daemon

MAS Daemon provides the MAS GUI with remote access to configuration files. It allows the GUI to edit and load launch files and to execute their nodes remotely.

The daemon is usually started by the MAS GUI through an SSH connection. Once running, the MAS GUI communicates with it via WebSockets using the following ports:


- **ROS 2:** `35430 + ROS_DOMAIN_ID`
- **ROS 1:** `35685 + NetworkId`

Make sure that the corresponding port is allowed by the firewall.

In addition to remote configuration access, MAS Daemon provides features such as:


- System monitoring
- Forwarding of diagnostic messages

- Automatic loading and starting of launch files

#### Environment Variables


- **`ROS_DOMAIN_ID`**  

  Determines the WebSocket port offset for ROS 2.


- **`MAS_COMPONENT_LOAD_TIMEOUT`** *(ROS 2 only)*  

  Defines how many seconds the autostart process waits for a composable node container's load service and its response.  
  The default value is `30`. Set it to `0` to wait indefinitely until the daemon is shut down.


- **`MAS_SYSTEM_DIAGNOSTIC`** *(ROS 2 only)*  

  Can be used to disable system monitoring.

#### Configuration

The daemon configuration is stored in:

`$HOME/.config/ros.fkie/mas_daemon.yaml`
