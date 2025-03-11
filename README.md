# Multi Agent Suite for ROS
[![Jazzy](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/jazzy_build.yml/badge.svg)](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/jazzy_build.yml)
[![Humble](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/humble_build.yml/badge.svg)](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/humble_build.yml)
[![Noetic](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/noetic_build.yml/badge.svg)](https://github.com/fkie/fkie-multi-agent-suite/actions/workflows/noetic_build.yml)

Based on the [FKIE Multimaster](https://github.com/fkie/multimaster_fkie), this suite provides a collection of packages for ROS 1 and 2 for discovering, synchronizing (ROS 1), monitoring, and managing nodes on different hosts using a graphical user interface.

> The suite replaces the FKIE Multimaster!

![mas overview](mas_overview.png)

## Install

The communication between the GUI and the Daemon (on each host) is based on WebSockets on port **35430+(ROS_DOMAIN_ID)**, 35430+255+(ROS_DOMAIN_ID)+101*(ROS_MASTER_URI_PORT-11311) with ROS1. These ports should be open in the firewall.

> In ROS2 we use a discovery node to get host information for each ROS node. Currently the discovery node depends on the **rmw_fastrtps_cpp** ROS library. When starting the discovery node **RMW_IMPLEMENTATION=rmw_fastrtps_cpp** should be prefixed with. If the MAS nodes are started via the graphical user interface, this is the case by default.

### Install debian packages from github

For Ubuntu 20.04, 22.04 and 24.04 there are Debian packages on Github that can be installed with the following command:

```bash
wget -qO - https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/master/install_mas_debs.sh | bash
```

### Run

```bash
mas-gui
```

## Alternative Install

Using AppImage and source build

### Install dependencies

You need a running [TTYD](https://github.com/tsl0922/ttyd) to show screen or log output of the nodes.

```bash
sudo snap install ttyd --classic
```

> A python3-websockets version >11 is required. You can install it using pip:

```bash
pip install "websockets>=12.0"
```


### Build ROS FKIE packages

You can run the following commands to setup a build from source:

```bash
cd ros_workspace/src
git clone https://github.com/fkie/fkie-multi-agent-suite.git fkie-multi-agent-suite
rosdep update
rosdep install -i --as-root pip:false --reinstall --from-paths fkie-multi-agent-suite
```

Then build all packages:

```bash
catkin build fkie_mas_meta
```

or

```bash
colcon build --packages-up-to fkie_mas_daemon
```

### Download GUI

```bash
curl -s https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases/latest | grep "browser_download_url.*mas-gui.AppImage" | cut -d : -f 2,3 | tr -d \" | wget --show-progress -i -
chmod +x ./mas-gui.AppImage
mv ./mas-gui.AppImage ~/.local/bin/.
```

### Run

```bash
mas-gui.AppImage
```
in Ubuntu 24.04
```bash
mas-gui.AppImage --no-sandbox
```

**For known issues and other build and launch options that affect GUI, see [fkie_mas_gui](https://github.com/fkie/fkie-multi-agent-suite/tree/master/fkie_mas_gui#readme)**

## Documentation

> We are working on the documentation for the new version. Until then you can still use the old one. Apart from the package names, all old functions should be supported.

- [multimaster_fkie](http://fkie.github.io/multimaster_fkie)
- [discovery](http://fkie.github.io/multimaster_fkie/master_discovery.html) -- `discovery using multicast or zeroconf`
- [synchronization](http://fkie.github.io/multimaster_fkie/master_sync.html) -- `master synchronization`
- [Node Manager GUI](http://fkie.github.io/multimaster_fkie/node_manager.html) -- `A GUI to manage the configuration on local and remote ROS masters`
- [Node Manager daemon](http://fkie.github.io/multimaster_fkie/node_manager_daemon.html) -- `Helper node allows an easy (auto)start of remote nodes and manage remote launch files`

For ROS interfaces and parameterization see the [ROS Wiki](http://www.ros.org/wiki/multimaster_fkie). For configuration details you can find example launch files in each package.

## License

MIT © [Fraunhofer FKIE](https://www.fkie.fraunhofer.de/en.html)
