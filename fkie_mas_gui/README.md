# MAS GUI

Early stage development of a new Node Manager for ROS (1/2).

The MAS GUI is composed by two main parts: A Back-End component (daemon with WebSocket), and a Front-End Component (powered by [Electron](https://www.electronjs.org/) and [ReactJS](https://reactjs.org/)).

## Install

We need to first install dependencies and run both Front and Back-End components:

### Install dependencies

The code have been tested with `NodeJS v20.15.0`:

```bash
sudo apt install nodejs yarn libsecret-1-dev
```

> In Linux, we need `libsecret-1-dev` to safely store SSH credentials.

You need a running [TTYD](https://github.com/tsl0922/ttyd) to show screen or log output of the nodes.

```bash
sudo snap install ttyd --classic
```

#### Update yarn to latest version

```bash
sudo curl --compressed -o- -L https://yarnpkg.com/install.sh | bash
```

#### Update nodejs to v20

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
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
colcon build --packages-select fkie_mas_meta
```

### Client Side (Front-End)

Only for developer mode: Download and install JavaScript module dependencies:

```bash
cd fkie_mas_gui
npm install --legacy-peer-deps
npm run build
```

## Usage

Run the app in developer mode:

```bash
npm run dev
```

The app will start automatically the local daemon and master discovery nodes.

## Start only as node (without GUI)

> broken after switch to vite-electron

Set environment variable ELECTRON_RUN_AS_NODE before start.

```bash
export ELECTRON_RUN_AS_NODE=true
npm run dev
```

Then open in browser <http://localhost:6274>

## Supported interface URIs

| Interface URI                       | Type | Function                                                                 | Description                                                                              |
| ---------------------------------- | ---- | ------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- |
| ros.daemon.ready                   | PUB  | `=> {'status': bool, "timestamp": float}`                                | Sent by the daemon at an interval                                                        |
| ros.discovery.ready                | PUB  | `=> {'status': bool}`                                                    |                                                                                          |
| ros.daemon.get_version             | RPC  | `() => DaemonVersion`                                                    | see fkie_mas_pylib/interface/runtime_interface/DaemonVersion                              |
| ros.file.get                       | RPC  | `(path: str) => FileItem`                                                | see fkie_mas_pylib/interface/file_interface/FileItem                                      |
| ros.file.save                      | RPC  | `(file: FileItem) => number`                                             | write file content to providers file system. Return count of written bytes.              |
| ros.nodes.get_list                 | RPC  | `() => RosNode[]`                                                        | see fkie_mas_pylib/interface/runtime_interface/RosNode                                    |
| ros.nodes.changed                  | PUB  | `=> "timestamp": float`                                                  | Triggers when node changed (start, stop etc...)                                          |
| ros.provider.list                  | PUB  | `=> RosProvider[]`                                                       | see fkie_mas_pylib/interface/runtime_interface/RosProvider                                |
| ros.provider.get_list              | RPC  | `() => RosProvider[]`                                                    | Request the list of current providers                                                    |
| ros.provider.get_timestamp         | RPC  | `(float) => "timestamp": float, "diff": float`                           | Request current time [ms] of the provider and calculate difference to given timestamp         |
| ros.provider.get_diagnostics       | RPC  | `() => DiagnosticArray[]`                                                | Request all available diagnostics                                                        |
| ros.provider.diagnostics           | PUB  | `=> DiagnosticArray[]`                                                   | updates to diagnostics                                                                   |
| ros.provider.ros_clean_purge       | RPC  | `() => {result: bool, message: str}`                                     | clean ros log path                                                                       |
| ros.provider.shutdown              | RPC  | `() => {result: bool, message: str}`                                     | kill all screens started by provider                                                     |
| ros.provider.warnings              | PUB  | `=> SystemWarningGroup[]`                                                | see fkie_mas_pylib/interface/runtime_interface/SystemWarningGroup                         |
| ros.provider.get_warnings          | RPC  | `() => SystemWarningGroup[]`                                             | see fkie_mas_pylib/interface/file_interface/RosPackage                                    |
| ros.packages.get_list              | RPC  | `(clear_cache: bool) => RosPackage[]`                                    | see fkie_mas_pylib/interface/file_interface/RosPackage                                    |
| ros.path.get_list                  | RPC  | `(inputPath: str) => PathItem[]`                                         | see fkie_mas_pylib/interface/file_interface/PathItem                                      |
| ros.path.get_list_recursive        | RPC  | `(inputPath: str) => PathItem[]`                                         | Return all files/folders included in input path                                          |
| ros.path.get_log_paths             | RPC  | `(nodes: str[]) => RosPackage[]`                                         |                                                                                          |
| ros.path.clear_log_paths           | RPC  | `(nodes: str[]) => {node: str, result: bool, message: str}[]`            | Removes log files (ROS and screen) for given nodes                                       |
| ros.path.changed                   | PUB  | `=> {str: str, str: str[]}`                                              | Triggers when a file was changed.                                                        |
| ros.launch.call_service            | RPC  | `(request: LaunchCallService) => LaunchMessageStruct`                    | Call a service.                                                                          |
| ros.launch.load                    | RPC  | `(request: LaunchLoadRequest) => LaunchLoadReply`                        | Loads launch file by interface request                                                    |
| ros.launch.reload                  | RPC  | `(request: LaunchLoadRequest) => LaunchLoadReply`                        | Reloads launch file by interface request                                                  |
| ros.launch.unload                  | RPC  | `(request: LaunchFile) => LaunchLoadReply`                               | Unload a launch file                                                                     |
| ros.launch.get_list                | RPC  | `() => LaunchContent[]`                                                  |                                                                                          |
| ros.launch.start_node              | RPC  | `(request: LaunchNode) => LaunchNodeReply`                               |                                                                                          |
| ros.launch.changed                 | PUB  | `=> {}`                                                                  | Triggers when a lunch file was loaded/unloaded/changed                                   |
| ros.launch.get_included_files      | RPC  | `(request: LaunchIncludedFilesRequest) => LaunchIncludedFile[]`          | Returns all included files in given launch file.                                         |
| ros.launch.interpret_path          | RPC  | `(request: LaunchInterpretPathRequest) => LaunchInterpretPathReply[]`    | Returns for given text all detected include paths.                                       |
| ros.launch.get_msg_struct          | RPC  | `(msg_type: str) => LaunchMessageStruct`                                 | Returns for given message type a JSON object.                                            |
| ros.launch.get_srv_struct          | RPC  | `(srv_type: str) => LaunchMessageStruct`                                 | Returns for given service type a JSON object.                                            |
| ros.launch.publish_message         | RPC  | `(request: LaunchPublishMessage) => void`                                | Launch a publisher.                                                                      |
| ros.nodes.get_loggers              | RPC  | `(name/id: str) => LoggerConfig[]`                                       | Returns a list of logger supported by node                                               |
| ros.nodes.set_logger_level         | RPC  | `(name/id: str, logger: LoggerConfig[]) => {result: bool, message: str}` | Changes the logger configuration for given node                                          |
| ros.nodes.stop_node                | RPC  | `(name/id: str) => {result: bool, message: str}`                         | Stop a node using ROS                                                                    |
| ros.nodes.unregister               | RPC  | `(name: str) => {result: bool, message: str}`                            | Unregister all topics and services of a node using ROS                                   |
| ros.screen.kill_node               | RPC  | `(name: str) => {result: bool, message: str}`                            | Kill screen of a given node                                                              |
| ros.screen.get_list                | RPC  | `() => ScreensMapping[]`                                                 | Returns a list off all screens and their name converted to ROS node name                 |
| ros.screen.list                    | PUB  | `=> ScreensMapping[]`                                                    | Triggers when screens are changed                                                        |
| ros.subscriber.start               | RPC  | `(request: SubscriberNode) => bool`                                      | Start subscriber for given topic                                                         |
| ros.subscriber.stop                | RPC  | `(topic: str) => bool`                                                   | Stop subscriber for given topic                                                          |
| ros.subscriber.event.{TOPIC}       | PUB  | `=> SubscriberEvent[]`                                                   | Event on received message by subscriber. TOPIC is a topic name with replaced '/' by '\_' |
| ros.subscriber.filter.{TOPIC}      | SUB  | `=> SubscriberFilter`                                                    | Updates filter for subscribed message. TOPIC is a topic name with replaced '/' by '\_'   |
| ros.system.get_uri                 | RPC  | `() => str`                                                              | Format ROS_MASTER_URI [INTERFACE PORT]                                                    |
| ros.parameters.get_list            | RPC  | `() => RosParameter[]`                                                   | Return all parameters as list (including values and types)                               |
| ros.parameters.get_node_parameters | RPC  | `(nodes: str[]) => RosParameter[]`                                       | Return a parameter list for a given Node                                                 |
| ros.parameters.has_parameter       | RPC  | `(parameter_name: str) => bool`                                          | Check if a parameter exists                                                              |
| ros.parameters.set_parameter       | RPC  | `(parameter: RosParameter) => bool`                                      | Set the value of a parameter                                                             |
| ros.parameters.delete_parameters   | RPC  | `(parameters: str[]) => bool`                                            | Delete a list of parameters                                                              |

## Additional Tools

- To package the client component into an AppImage and zip package, run:

```bash
cd fkie_mas_gui
npm run build:linux
```

## Known Issues

- If you work behind a proxy, you might need to explicitly specify the `URL` and `PORT` before downloading the electron packages. The problem occurs when installing the package (`yarn install`).

```bash
export ELECTRON_GET_USE_PROXY=true
export GLOBAL_AGENT_HTTPS_PROXY=http://XXX.XXX.XXX.XXX:XXXX
```

- If you get the error: `release/app/node_modules/ssh2/lib/protocol/crypto/build/Release/sshcrypto.node: undefined symbol: ERR_set_mark`, you might need to force adding again the `ssh2` package on the `release/app` project:

```bash
cd release/app
npm add ssh2 --force
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

## Maintainers

- Francisco Garcia
- [Carlos Tampier](carlos.tampier.cotoras@fkie.fraunhofer.de)
- [Alexander Tiderko](alexander.tiderko@fkie.fraunhofer.de)

## License

MIT © [Fraunhofer FKIE](https://www.fkie.fraunhofer.de/en.html)
