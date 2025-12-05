# MAS GUI

Node Manager for ROS (1/2).

The MAS GUI is composed by two main parts: A Back-End component (daemon with WebSocket), and a Front-End Component (powered by [Electron](https://www.electronjs.org/) and [ReactJS](https://reactjs.org/)). Data exchange between the daemon and the Front-End takes place via the [WebSocket](../doc/websocket.md). The description of the message formats can be found [here](../doc/interface.md).

## Install

We need to first install dependencies and run both Front and Back-End components:

### Install dependencies

You need a running [TTYD](https://github.com/tsl0922/ttyd) to show screen or log output of the nodes.

```bash
sudo snap install ttyd --classic
```

#### Update nodejs to v22

The code have been tested with `NodeJS v22.21.0`:

```bash
curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### Server Side (Back-End)

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
colcon build --packages-up-to fkie_mas_meta
```

### Client Side (Front-End)

Only for developer mode: Download and install JavaScript module dependencies:

```bash
cd fkie_mas_gui
npm install
npm run build
```

## Usage

Run the app in developer mode:

```bash
npm run dev:ns
```

The app will start automatically the local daemon and master discovery nodes.

## Start only as node (without GUI)

Starts a server on default port 6274. You can change the port in **electron.vite.config.ts**.

```bash
npm run server:ns
```

Then open in browser <http://localhost:6274>

## Additional Tools

- To package the client component into an AppImage, run:

```bash
cd fkie_mas_gui
npm run build:linux
```

- update licenses of dependencies

```bash
npm install -g license-report
license-report --config=./license-report-config.json > src/renderer/deps-licenses.json
```

## Known Issues

- If you work behind a proxy, you might need to explicitly specify the `URL` and `PORT` before downloading the electron packages. The problem occurs when installing the package (`npm install`).

```bash
export ELECTRON_GET_USE_PROXY=true
export GLOBAL_AGENT_HTTPS_PROXY=http://XXX.XXX.XXX.XXX:XXXX
```

- `The SUID sandbox helper binary was found, but is not configured correctly. Rather than run without sandboxing I'm aborting now.`: You can check [https://github.com/electron/electron/issues/42510](https://github.com/electron/electron/issues/42510) for more details. Use `--no-sandbox` argument as workaround, e.g.:

```bash
npm run dev -- --no-sandbox
```

or

```bash
mas-gui.appimage --no-sandbox
```

- If you get the SFPT error: `getSftpChannel: Packet length XXXXXX exceeds max length of XXXXX`, it might be caused by an `echo` command on the `.bashrc` file. Make sure all `echo` commands are defined only for interactive shells. For instance in `.bashrc`:

```bash
...
# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

echo "your echo commands here!"
```

See related reports on [Stack Exchange](https://unix.stackexchange.com/questions/61580/sftp-gives-an-error-received-message-too-long-and-what-is-the-reason), [SSH2 GitHub](https://github.com/mscdex/ssh2/issues/509) or [StackOverflow](https://stackoverflow.com/questions/33409233/getting-received-too-large-sftp-packet-when-logging-in-with-root-using-winscp).

- _You don't see the correct output in external terminal_. Try to change your default terminal:

```bash
sudo update-alternatives --config x-terminal-emulator
```

## Maintainers

- Francisco Garcia
- [Carlos Tampier](carlos.tampier.cotoras@fkie.fraunhofer.de)
- [Alexander Tiderko](alexander.tiderko@fkie.fraunhofer.de)

## License

MIT © [Fraunhofer FKIE](https://www.fkie.fraunhofer.de/en.html)
