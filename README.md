# Multi Agent Suite for ROS

Based on the [FKIE Multimaster](https://github.com/fkie/multimaster_fkie), this suite provides a collection of packages for ROS 1 and 2 for discovering, synchronizing (ROS 1), monitoring, and managing nodes on different hosts using a graphical user interface.

> The suite replaces the FKIE Multimaster!

![mas overview](mas_overview.png)

## Install

The communication between the GUI and the Daemon (on each host) is based on [WAMP](https://wamp-proto.org/). It needs a running [WAMP Router](https://wamp-proto.org/implementations.html#routers). We use crossbar.

### Install dependencies

The code have been tested with `Crossbar v22.2.1`:

```bash
sudo snap install crossbar
```

You need a running [TTYD](https://github.com/tsl0922/ttyd) to show screen or log output of the nodes.

```bash
sudo snap install ttyd --classic
```

In Linux, we need `libsecret-1-dev` to safely store SSH credentials.

```bash
sudo apt install libsecret-1-dev
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

## Documentation

> We are working on the documentation for the new version. Until then you can still use the old one. Apart from the package names, all old functions should be supported.

* [multimaster\_fkie](http://fkie.github.io/multimaster_fkie)
* [discovery](http://fkie.github.io/multimaster_fkie/master_discovery.html) -- `discovery using multicast or zeroconf`
* [synchronization](http://fkie.github.io/multimaster_fkie/master_sync.html) -- `master synchronization`
* [Node Manager GUI](http://fkie.github.io/multimaster_fkie/node_manager.html) -- `A GUI to manage the configuration on local and remote ROS masters`
* [Node Manager daemon](http://fkie.github.io/multimaster_fkie/node_manager_daemon.html) -- `Helper node allows an easy (auto)start of remote nodes and manage remote launch files`

For ROS interfaces and parameterization see the [ROS Wiki](http://www.ros.org/wiki/multimaster_fkie). For configuration details you can find example launch files in each package.

## License

MIT © [Fraunhofer FKIE](https://www.fkie.fraunhofer.de/en.html)
