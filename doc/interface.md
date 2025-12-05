# Interface description

The data is exchanged in JSON format. So-called URIs are used to specify the data. These are described below.

Data is sent and received via a WebSocket. To this end, the WebSocket has been expanded with mechanisms such as publish/subscribe and remote procedure calls. You find details [here](./websocket.md).

## Interface URIs

Each URI has one of the following types:

- `PUB`: Published by daemon. The message will be sent to all subscribers when it is published.
- `SUB`: Subscribed by daemon.
- `RPC`: Remote procedure call, with request and reply message.

| Interface URI                                                               | Type |
| --------------------------------------------------------------------------- | ---- |
| [ros.daemon.ready](#rosdaemonready-pub)                                     | PUB  |
| [ros.discovery.ready](#rosdiscoveryready-pub)                               | PUB  |
| [ros.daemon.get_version](#rosdaemonget_version-rpc)                         | RPC  |
| [ros.file.get](#rosfileget-rpc)                                             | RPC  |
| [ros.file.save](#rosfilesave-rpc)                                           | RPC  |
| [ros.nodes.get_list](#rosnodesget_list-rpc)                                 | RPC  |
| [ros.services.get_list](#rosservicesget_list-rpc)                           | RPC  |
| [ros.topics.get_list](#rostopicsget_list-rpc)                               | RPC  |
| [ros.nodes.changed](#rosnodeschanged-pub)                                   | PUB  |
| [ros.services.changed](#rosserviceschanged-pub)                             | PUB  |
| [ros.topics.changed](#rostopicschanged-pub)                                 | PUB  |
| [ros.provider.list](#rosproviderlist-pub)                                   | PUB  |
| [ros.provider.get_list](#rosproviderget_list-rpc)                           | RPC  |
| [ros.provider.get_timestamp](#rosproviderget_timestamp-rpc)                 | RPC  |
| [ros.provider.get_diagnostics](#rosproviderget_diagnostics-rpc)             | RPC  |
| [ros.provider.diagnostics](#rosproviderdiagnostics-pub)                     | PUB  |
| [ros.provider.ros_clean_purge](#rosproviderros_clean_purge-rpc)             | RPC  |
| [ros.provider.shutdown](#rosprovidershutdown-rpc)                           | RPC  |
| [ros.provider.warnings](#rosproviderwarnings-pub)                           | PUB  |
| [ros.provider.get_warnings](#rosproviderget_warnings-rpc)                   | RPC  |
| [ros.packages.get_list](#rospackagesget_list-rpc)                           | RPC  |
| [ros.path.get_list](#rospathget_list-rpc)                                   | RPC  |
| [ros.path.get_log_paths](#rospathget_log_paths-rpc)                         | RPC  |
| [ros.path.clear_log_paths](#rospathclear_log_paths-rpc)                     | RPC  |
| [ros.path.changed](#rospathchanged-pub)                                     | PUB  |
| [ros.launch.call_service](#launchcallservice)                               | RPC  |
| [ros.launch.load](#roslaunchload-rpc)                                       | RPC  |
| [ros.launch.reload](#roslaunchreload-rpc)                                   | RPC  |
| [ros.launch.unload](#roslaunchunload-rpc)                                   | RPC  |
| [ros.launch.get_list](#roslaunchget_list-rpc)                               | RPC  |
| [ros.launch.start_node](#roslaunchstart_node-rpc)                           | RPC  |
| [ros.launch.changed](#roslaunchchanged-pub)                                 | PUB  |
| [ros.launch.get_included_files](#roslaunchget_included_files-rpc)           | RPC  |
| [ros.launch.interpret_path](#roslaunchinterpret_path-rpc)                   | RPC  |
| [ros.launch.get_msg_struct](#roslaunchget_msg_struct-rpc)                   | RPC  |
| [ros.launch.get_srv_struct](#roslaunchget_srv_struct-rpc)                   | RPC  |
| [ros.launch.get_message_types](#roslaunchget_message_types-rpc)             | RPC  |
| [ros.launch.publish_message](#roslaunchpublish_message-rpc)                 | RPC  |
| [ros.nodes.get_loggers](#rosnodesget_loggers-rpc)                           | RPC  |
| [ros.nodes.set_logger_level](#rosnodesset_logger_level-rpc)                 | RPC  |
| [ros.nodes.stop_node](#rosnodesstop_node-rpc)                               | RPC  |
| [ros.nodes.unregister](#rosnodesunregister-rpc)                             | RPC  |
| [ros.screen.kill_node](#rosnodeskill_node-rpc)                              | RPC  |
| [ros.screen.get_list](#rosscreenget_list-rpc)                               | RPC  |
| [ros.screen.list](#rosscreenlist-pub)                                       | PUB  |
| [ros.subscriber.start](#rossubscriberstart-rpc)                             | RPC  |
| [ros.subscriber.stop](#rossubscriberstop-rpc)                               | RPC  |
| [ros.subscriber.event.{TOPIC}](#rossubscribereventtopic-pub)                | PUB  |
| [ros.subscriber.filter.{TOPIC}](#rossubscriberfiltertopic-sub)              | SUB  |
| [ros.system.get_uri](#rossystemget_uri-rpc)                                 | RPC  |
| [ros.parameters.get_list](#rosparametersget_list-rpc)                       | RPC  |
| [ros.parameters.get_node_parameters](#rosparametersget_node_parameters-rpc) | RPC  |
| [ros.parameters.set_parameter](#rosparametersset_parameter-rpc)             | RPC  |
| [ros.parameters.delete_parameters](#rosparametersdelete_parameters-rpc)     | RPC  |
| [ros.process.find_node](#rosprocessfind_node-rpc)                           | RPC  |
| [ros.process.kill](#rosprocesskill-rpc)                                     | RPC  |

## Message formats

> Arguments with `"name"?`are optional.

### ros.daemon.ready `PUB`

Sent by the daemon at an interval

```json
{"status": bool, "timestamp": float}
```

### ros.discovery.ready `PUB`

Sent by the daemon at an interval

```json
{"status": bool}
```

### ros.daemon.get_version `RPC`

`Request`: `empty`

`Reply`: [DaemonVersion](#daemonversion)

### ros.file.get `RPC`

Reads file content from providers file system.

`Request`: `str` _path_

`Reply`: [FileItem](#fileitem)

### ros.file.save `RPC`

Writes file content to providers file system. Return count of written bytes.

`Request`: [FileItem](#fileitem)

`Reply`: `int`

### ros.nodes.get_list `RPC`

If the specified value is `True`, a full update is forced.

`Request`: `bool` _force update_

`Reply`: [RosNode](#rosnode)[]

### ros.services.get_list `RPC`

A filter can be provided on the request.

`Request`: [RosTopicId](#rostopicid)[] _filter_

`Reply`: [RosService](#rosservice)[]

### ros.topics.get_list `RPC`

A filter can be provided on the request.

`Request`: [RosTopicId](#rostopicid)[] _filter_

`Reply`: [RosTopic](#rostopic)[]

### ros.nodes.changed `PUB`

Triggers when node changed (start, stop, ...)

```json
{"timestamp": float}
```

### ros.services.changed `PUB`

Triggers when list of topic descriptions changed

```json
{"timestamp": float}
```

### ros.topics.changed `PUB`

Triggers when list of service descriptions changed

```json
{"timestamp": float}
```

### ros.provider.list `PUB`

Triggers when list of discovered daemons changed

> **[RosProvider](#rosprovider)[]**

### ros.provider.get_list `RPC`

Requests the list of current providers

`Request`: `empty`

`Reply`: [RosProvider](#rosprovider)[]

### ros.provider.get_timestamp `RPC`

Requests current time [ms] of the provider and calculate difference to given timestamp.

`Request`: `float` _known timestamp_

`Reply`:

```json
{"timestamp": float, "diff": float}
```

### ros.provider.get_diagnostics `RPC`

Requests all available diagnostics.

`Request`: `empty`

`Reply`: [DiagnosticArray]()[]

### ros.provider.diagnostics `PUB`

Updates to diagnostics.

> **[DiagnosticArray]()[]**

### ros.provider.ros_clean_purge `RPC`

Cleans ros log folder.

`Request`: `empty`

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.provider.shutdown `RPC`

Kills all screens started by provider. Also MAS nodes.

`Request`: `empty`

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.provider.warnings `PUB`

Updates of warnings detected by daemon.

> **[SystemWarningGroup]()[]**

### ros.provider.get_warnings `RPC`

Requests all available diagnostics.

`Request`: `empty`

`Reply`: [SystemWarningGroup]()[]

### ros.packages.get_list `RPC`

Requests all packages. If provided parameter is True, daemon rebuild the list.

`Request`: `bool` _clear cache_

`Reply`: [RosPackage](#rospackage)[]

### ros.path.get_list `RPC`

Requests all items within the specified path.

`Request`: `str` _path_, `bool` _recursive_

`Reply`: [PathItem](#pathitem)[]

### ros.path.get_log_paths `RPC`

Requests paths of log files for given nodes.

`Request`: `str[]` _nodes_

`Reply`: [LogPathItem](#logpathitem)[]

### ros.path.clear_log_paths `RPC`

Removes log files (ROS and screen) for given nodes

`Request`: `str[]` _nodes_

`Reply`:

```json
{"node": str, "result": bool, "message": str}[]
```

### ros.path.changed `PUB`

Triggers when a file was changed

```json
{"eventType": str, "srcPath": str, "affected": str[]}
```

### ros.launch.call_service `RPC`

Calls a ROS service.

`Request`: [LaunchCallService](#launchcallservice)

`Reply`: [LaunchMessageStruct](#launchmessagestruct)

### ros.launch.load `RPC`

Loads the launch file.

`Request`: [LaunchLoadRequest](#launchloadrequest)

`Reply`: [LaunchLoadReply](#launchloadreply)

### ros.launch.reload `RPC`

Reloads the launch file.

`Request`: [LaunchLoadRequest](#launchloadrequest)

`Reply`: [LaunchLoadReply](#launchloadreply)

### ros.launch.unload `RPC`

Unloads the launch file.

`Request`: [LaunchFile](#launchfile)

`Reply`: [LaunchLoadReply](#launchloadreply)

### ros.launch.get_list `RPC`

Requests the currently loaded launch files.

`Request`: `empty`

`Reply`: [LaunchContent](#launchcontent)[]

### ros.launch.start_node `RPC`

`Request`: [LaunchNode](#launchnode)

`Reply`: [LaunchNodeReply](#launchnodereply)

### ros.launch.changed `PUB`

Triggers when a lunch file was loaded/unloaded/changed. Actions {`loaded`, `reloaded`, `unloaded`}

```json
{"path": str, "action": str, "requester": str}
```

### ros.launch.get_included_files `RPC`

Returns all included files in given launch file.

`Request`: [LaunchIncludedFilesRequest](#launchincludedfilesrequest)

`Reply`: [LaunchIncludedFile](#launchincludedfile)

### ros.launch.interpret_path `RPC`

Returns for given text all detected include paths.

`Request`: [LaunchInterpretPathRequest](#launchinterpretpathrequest)

`Reply`: [LaunchInterpretPathReply](#launchinterpretpathreply)

### ros.launch.get_msg_struct `RPC`

Returns for given message type a JSON object.

`Request`: `str` _message type_

`Reply`: [LaunchMessageStruct](#launchmessagestruct)

### ros.launch.get_srv_struct `RPC`

Returns for given service type a JSON object.

`Request`: `str` _service type_

`Reply`: [LaunchMessageStruct](#launchmessagestruct)

### ros.launch.get_message_types `RPC`

Returns a list of available ROS message types.

`Request`: `empty`

`Reply`: `str[]`

### ros.launch.publish_message `RPC`

Launch a ROS publisher as new node.

`Request`: [LaunchPublishMessage](#launchpublishmessage)

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.nodes.get_loggers `RPC`

Returns a list of logger supported by node.

`Request`: `str` _service type_, `str[]` _loggers_

`Reply`: [LoggerConfig](#loggerconfig)

### ros.nodes.set_logger_level `RPC`

Changes the logger configuration for given node.

`Request`: `str` _node name_, [LoggerConfig](#loggerconfig) _loggers_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.nodes.stop_node `RPC`

Stops the given ROS node.

`Request`: `str` _node name_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.nodes.unregister `RPC`

Unregister all topics and services of a node (ROS1 only)

`Request`: `str` _node name_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.nodes.kill_node `RPC`

Kills the node or the screen of a given node

`Request`: `str` _node name_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.screen.get_list `RPC`

Returns a list off all screens and their name converted to ROS node name.

`Request`: `empty`

`Reply`: [ScreensMapping](#screensmapping)

### ros.screen.list `PUB`

Triggers when screens are changed.

[ScreensMapping](#screensmapping)

### ros.subscriber.start `RPC`

Starts subscriber node for given topic.

`Request`: [SubscriberNode](#subscribernode)

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.subscriber.stop `RPC`

Stops subscriber for given topics.

`Request`: `str`_topic name_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.subscriber.event.{TOPIC} `PUB`

Event on received message by subscriber. TOPIC is a topic name with replaced '/' by '\_'

[SubscriberEvent](#subscriberevent)[]

### ros.subscriber.filter.{TOPIC} `SUB`

Updates filter for subscribed message. TOPIC is a topic name with replaced '/' by '\_'

[SubscriberFilter](#subscriberfilter)

### ros.system.get_uri `RPC`

Only ROS1! Format ROS_MASTER_URI [INTERFACE PORT]

`Request`: `empty`

`Reply`: `str`

### ros.parameters.get_list `RPC`

Returns all parameters as list (including values and types)

`Request`: `empty`

`Reply`:

```json
{"params": RosParameter[], "errors": str[]}
```

[RosParameter](#rosparameter)

### ros.parameters.get_node_parameters `RPC`

Return a parameter list for a given nodes.

`Request`: `str[]` _nodes_

`Reply`:

```json
{"params": RosParameter[], "errors": str[]}
```

[RosParameter](#rosparameter)

### ros.parameters.set_parameter `RPC`

Set the value of a parameter

`Request`: `str` _name_, `str`, _type_. `str`_value_, `str`_node name_

`Reply`:

```json
{"result": bool, "message": str, "value"?: str, "value_type"?: str}
```

### ros.parameters.delete_parameters `RPC`

Deletes a list of parameters.

`Request`: `str[]` _parameter names_, `str`_node name_

`Reply`:

```json
{"result": bool, "message": str}
```

### ros.process.find_node `RPC`

Searches for a process which belongs to the ros 2 node.

`Request`: `str` _node name_

`Reply`:

```json
{
  "result": bool,
  "message": str,
  "processes": {
    "pid": int,
    "cmdLine": str
  }[]
}
```

### ros.process.kill `RPC`

Kills the process

`Request`: `int` _process id_

`Reply`:

```json
{"result": bool, "message": str}
```

## Data Types

### DaemonVersion

```json
{"version": str, "date": str}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/DaemonVersion.ts)

### FileItem

```json
{
  "path": str,
  "mtime"?: float,
  "size"?: int,
  "readonly"?: bool,
  "value"?: str,
  "encoding"?: str
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/file_interface.py), [GUI](./src/renderer/models/FileItem.ts)

### RosNode

```json
{
  "id": str,
  "is_container": bool,
  "container_name": str,
  "name": str,              // full node name with namespace
  "namespace": str,
  "status": str,            // e.g. "running"
  "pid": int,               // process id
  "process_ids": str[],     // process ids of the screen('s) or processes found by __node:={basename} and __ns:={namespace}
  "node_API_URI"?: str,     // the uri of the node api (ROS1)
  "masteruri"?: str,        // only ROS1
  "location": str,          // address of the running
  "is_local": bool,         // True if the node is running on the same host as the MAS daemon.
  "publishers": RosTopicId[],
  "subscribers": RosTopicId[],
  "services": RosTopicId[],
  "screens": str[],
  "parameters": RosParameter[],
  "system_node": bool,
  "enclave": str
}
```

[RosTopicId](#rostopicid), [RosParameter](#rosparameter)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosNode.ts)

### RosTopicId

```json
{
  "name": str,
  "msg_type": str
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosTopicId.ts)

### RosParameter

```json
{
  "node": str,  // full name of the node
  "name": str,
  "value": Union[int, float, bool, str, List, Dict],
  "type"?: str,
  "readonly": bool,
  "description"?: str,
  "additional_constraints"?: str,
  "floating_point_range": RosParameterRange[],
  "integer_range": RosParameterRange[]
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosParameter.ts)

[RosParameterRange](#rosparameterrange)

### RosParameterRange

```json
{
  "from_value": Union[int, float, None],
  "to_value": Union[int, float, None],
  "step": Union[int, float, None]
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosParameter.ts)

### RosService

```json
{
  "id": str,
  "name": str,
  "srv_type": str,
  "masteruri"?: str,
  "service_API_URI"?: str,
  "provider": str[],
  "requester": str[],
  "location": str,   // usually the location of the node offering this service
  "is_request": bool
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosService.ts)

### RosTopic

```json
{
  "id": str,
  "name": str,
  "msg_type": str,
  "publisher": EndpointInfo[],
  "subscriber": EndpointInfo[],
}
```

[EndpointInfo](#endpointinfo)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosTopic.ts)

### EndpointInfo

```json
{
  "node_id": str,
  "qos"?: RosQos,
  "incompatible_qos": IncompatibleQos[],
}
```

[RosQos](#rosqos), [IncompatibleQos](#incompatibleqos)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosTopic.ts)

### IncompatibleQos

```json
{
  "node_id": str,
  "compatibility": str,
  "reason": str
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosTopic.ts)

### RosQos

```json
{
  "durability": int,  // Default: DURABILITY.VOLATILE
  "history": int,     // Default: HISTORY.KEEP_LAST
  "depth": int,       // Default: 10
  "liveliness": int,  // LIVELINESS.SYSTEM_DEFAULT
  "reliability": int, // RELIABILITY.RELIABLE
  "deadline": RosDuration,
  "liveliness_lease_duration": RosDuration,
  "lifespan": RosDuration,
  "avoid_ros_namespace_conventions": bool
}
```

[RosDuration](#rosduration)

```json
{
  "RELIABILITY": {
    "SYSTEM_DEFAULT": 0, // Implementation specific default
    "RELIABLE": 1, // Guarantee that samples are delivered, may retry multiple times.
    "BEST_EFFORT": 2, // Attempt to deliver samples, but some may be lost if the network is not robust
    "UNKNOWN": 3
  },

  "HISTORY": {
    "SYSTEM_DEFAULT": 0, // Implementation default for history policy
    "KEEP_LAST": 1, // Only store up to a maximum number of samples, dropping oldest once max is exceeded
    "KEEP_ALL": 2, // Store all samples, subject to resource limits
    "UNKNOWN": 3
  },

  "DURABILITY": {
    "SYSTEM_DEFAULT": 0, // Implementation specific default
    "TRANSIENT_LOCAL": 1, // The rmw publisher is responsible for persisting samples for “late-joining” subscribers
    "VOLATILE": 2, // Samples are not persistent
    "UNKNOWN": 3
  },

  "LIVELINESS": {
    "SYSTEM_DEFAULT": 0,
    "AUTOMATIC": 1, // The signal that establishes a Topic is alive comes from the ROS rmw layer.
    "MANUAL_BY_NODE": 2, // Deprecated: Explicitly asserting node liveliness is required in this case.
    // The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
    // on the Topic or an explicit signal from the application to assert liveliness on the Topic
    // will mark the Topic as being alive.
    // Using `3` for backwards compatibility.
    "MANUAL_BY_TOPIC": 3,
    "UNKNOWN": 4
  }
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosQos.ts)

### RosDuration

```json
{
  "sec": int,
  "nanosec": int,
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/RosDuration.ts)

### RosProvider

```json
{
  "ros_version": str,
  "ros_distro": str,
  "ros_domain_id": str,
  "rmw_implementation": str,
  "name": str,
  "host": str,        // port of the interface server (websocket or crossbar-wamp)
  "port": int,
  "type": str,        // websocket, crossbar-wamp
  "masteruri": str,   // only ROS1
  "origin": bool,     // True if the provider represents itself, otherwise it is a discovered provider.
  "hostnames": str[], // All known hostnames for this provides, e.g. IPv4 IPv6 or names.
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/providers/RosProviderState.ts)

### DiagnosticArray

```json
{
  "timestamp": float,
  "status": DiagnosticStatus[]
}
```

[DiagnosticStatus](#diagnosticstatus)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/Diagnostics.ts)

### DiagnosticStatus

```json
{
  "level": int,       // "OK": 0, "WARN": 1, "ERROR": 2, "STALE": 3
  "name": str,        // a description of the test/component reporting
  "message": str,     // a description of the status
  "hardware_id": str, // a hardware unique string
  "values": [
    {
      "key": str,
      "value": str
    }
  ]
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/Diagnostics.ts)

### SystemWarningGroup

```json
{
  "id": str,  // see IDs
  "warnings": SystemWarning[]
}
```

**IDs**:
`'ADDR_MISMATCH'`, `'RESOLVE_FAILED'`, `'UDP_SEND'`, `'EXCEPTION'`, `'TIME_JUMP'`, `'ROS_STATE_CHECK'`

[SystemWarning](#SystemWarning)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/SystemWarningGroup.ts)

### SystemWarning

```json
{
  "msg": str,     // short warning message
  "details": str, // long description
  "hint": str     // note on the possible solution
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/SystemWarning.ts)

### RosPackage

```json
{
  "name": str,
  "path": str
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/file_interface.py), [GUI](./src/renderer/models/RosPackage.ts)

### PathItem

```json
{
  "path": str,      // absolute path of the file or directory
  "mtime": float,   // time of last modification of path. The return value is a number giving the number of seconds since the epoch
  "size": int,      // size, in bytes, of path
  "path_type": str  // one of types {file, dir, symlink, package}
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/file_interface.py), [GUI](./src/renderer/models/PathItem.ts)

### LogPathItem

```json
{
  "node": str,                // complete node name
  "screen_log": str,          // the absolute path to the screen log file.
  "screen_log_exists": bool,  // False if the file does not exists.
  "ros_log": str,             // the absolute path to the ros log file.
  "ros_log_exists": bool,     // False if the file does not exists.
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/file_interface.py), [GUI](./src/renderer/models/LogPathItem.ts)

### LaunchCallService

```json
{
  "service_name": str,    // the ROS service name.
  "srv_type": str,        // Type of the request message.
  "data": {},             // structure of the ROS request message as dictionary.
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchCallService.ts)

### LaunchMessageStruct

```json
{
  "msg_type": str,    // Type of the message.
  "data": {},        // structure of the ROS message as dictionary. Each field is at least described by MessageField.
  "valid": bool,             // True if the data for the message type was loaded successfully.
  "error_msg": str
}
```

#### MessageField

```json
{
  "type": str,
  "name": str,  // slot name
  "def": 'MessageField or []',
  "default_value": [],
  "is_array": bool
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchCallService.ts)

### LaunchLoadRequest

```json
{
  "ros_package": str,       // ROS package name
  "launch": str,            // launch file in the package path. If the package contains more then one launch file with same name you have to specify the sub-path with launch file.
  "path": str,              // if set, this will be used instead of package/launch
  "args": LaunchArgument[], // arguments to load the launch file. If args are empty but the launch file needs them, the reply status has code PARAMS_REQUIRED and args list will be filled with requested args.
  "force_first_file": bool, // if True, use first file if more than one was found in the package.
  "request_args": bool,     // If True, the launch file will not be loaded, only launch arguments are requested.
  "masteruri": str,         // starts nodes of this file with specified ROS_MASTER_URI. If host is empty, the nodes are started on the host specified by hostname of the masteruri.
  "host": str               // start nodes of this file on specified host.
}
```

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchLoadRequest.ts)

### LaunchArgument

```json
{
  "name": str,
  "value": str,
  "default_value": Any,
  "description": str,
  "choices": str[]
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchArgument.ts)

### LaunchLoadReply

```json
{
  "status": LaunchReturnStatus,
  "paths": str[],
  "args": LaunchArgument[],
  "changed_nodes": str[]
}
```

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchLoadReply.ts)

### LaunchReturnStatus

```json
{
  "code": str,  // see Codes
  "msg": str
}
```

**Codes**:
`'OK'`
`'ERROR'`
`'ALREADY_OPEN'`
`'MULTIPLE_BINARIES'`
`'MULTIPLE_LAUNCHES'`
`'PARAMS_REQUIRED'`
`'FILE_NOT_FOUND'`
`'NODE_NOT_FOUND'`
`'PACKAGE_NOT_FOUND'`
`'CONNECTION_ERROR'`

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchReturnStatus.ts)

### LaunchFile

```json
{
  "path": str,
  "masteruri": str,
  "host": str
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchFile.ts)

### LaunchContent

```json
{
  "path": str,                          // full path of the launch file
  "args": LaunchArgument[],             // arguments used to load the launch file.
  "masteruri": str,                     // starts nodes of this file with specified ROS_MASTER_URI. If host is empty, the nodes are started on the host specified by hostname of the masteruri. Only ROS1!
  "host": str,                          // if not empty, the nodes of this launch file are launched on specified host.
  "nodes": str[],                       // list of node names.
  "parameters": RosParameter[],         // ROS1 only. Parameter for all nodes
  "associations": LaunchAssociations[]  // Associated nodes that should be started before the node itself
}
```

[RosParameter](#rosparameter), [LaunchAssociations](#launchassociations)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchContent.ts)

### LaunchAssociations

```json
{
  "node": str,       // node (full name)
  "nodes": str[],    // list with associated nodes (full name).
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchAssociations.ts)

### LaunchNode

```json
{
  "name": str,                // full name of the ros node exists in the launch file.
  "opt_binary": str,          // the full path of the binary. Used in case of multiple binaries in the same package.
  "opt_launch": str,          // full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
  "loglevel": str,
  "logformat": str,
  "masteruri": str',          // ROS1 only
  "reload_global_param": bool,// ROS1 only
  "cmd_prefix": str,          // custom command prefix. It will be prepended before launch prefix
  "ignore_timer": bool        // force to ignore timer specified in launch file
}
```

[LaunchReturnStatus](#launchreturnstatus)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchNode.ts)

### LaunchNodeReply

```json
{
  "name": str,
  "status": LaunchReturnStatus, // the status of the start process.
  "paths": str[],               // a list of paths with binaries for a node, only if MULTIPLE_BINARIES is returned.
  "launch_files": str[]         // a list with names launch files, only if MULTIPLE_LAUNCHES is returned.
}
```

[LaunchReturnStatus](#launchreturnstatus)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchNodeReply.ts)

### LaunchIncludedFilesRequest

```json
{
  "path": str,              // file to parse
  "recursive": bool,        // True to read recursive.
  "unique": bool,           // True to ignore files included multiple times.
  "pattern": str[],         // pattern to change include detection.
  "search_in_ext": str,     // search only for files with given extensions.
  "args": LaunchArgument[]  // use include launch arguments.
}
```

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchIncludedFilesRequest.ts)

### LaunchIncludedFile

```json
{
  "path": str,                // current reading file.
  "line_number": int,         // line number of the occurrence. If `unique` is True the line number is zero.
  "inc_path": str,            // resolved path.
  "exists": bool,             // True if resolved path exists.
  "raw_inc_path": str,        // representation of included file without resolved arg and find statements.
  "rec_depth": int,           // depth of recursion. if `unique` is True the depth is zero
  "args": LaunchArgument[],   // a list with arguments forwarded within include tag for 'inc_path'.
  "default_inc_args": LaunchArgument[], // a list with default arguments defined in 'inc_path'.
  "size": int,                // size of the file in bytes.
  "conditional_excluded": bool // if True the included file is not loaded be current configuration.
}
```

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchIncludedFile.ts)

### LaunchInterpretPathRequest

```json
{
  "text": str,              // line in the launch config.
  "args": LaunchArgument[]  // a list of the arguments used load the launch file.
}
```

[LaunchArgument](#launchargument)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchInterpretPathRequest.ts)

### LaunchInterpretPathReply

```json
{
  "text": str,              // line in the launch config.
  "status": str,            // the status of the parsing. One of the codes of LaunchReturnStatus
  "path": str,              // the path of the configuration file containing the text.
  "exists": bool,           // True if detected include path exists.
  "args": LaunchArgument[]  // a list of the arguments used load the launch file.
}
```

[LaunchArgument](#launchargument), [LaunchReturnStatus](#launchreturnstatus)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchInterpretPathReply.ts)

### LaunchPublishMessage

```json
{
  "topic_name": str,            // the ROS topic name.
  "msg_type": str, *,           // Type of the message.
  "data": {},                   // structure of the ROS message as dictionary.
  "rate": float,                // publishing rate (hz), only if once and latched is False.
  "once": bool,                 // publish one message and exit.
  "latched": bool,              // enable latching.
  "verbose": bool,              // print verbose output.
  "use_rostime": bool,          // use rostime for time stamps, else walltime is used.
  "substitute_keywords": bool,  // When publishing with a rate, performs keyword ('now' or 'auto') substitution for each message.
  "qos": RosQos                 // Quality of service subscription options (Only ROS2).
}
```

[RosQos](#rosqos)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/launch_interface.py), [GUI](./src/renderer/models/LaunchPublishMessage.ts)

### LoggerConfig

```json
{
  "level": str,   // LogLevelType
  "name": str,    // name of the logger
}
```

**LogLevelType**: `UNKNOWN`, `DEBUG`, `ÌNFO`, `WARN`, `ERROR`, `FATAL`

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/LoggerConfig.ts)

### ScreensMapping

```json
{
  "name": str,      // node name
  "screens": str[], // list of the screen names associated with given node.
}
```

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/ScreensMapping.ts)

### SubscriberNode

```json
{
  "topic": str,               // Name of the ROS topic to listen to (e.g. '/chatter').
  "message_type": str,        // Type of the ROS message (e.g. 'std_msgs/msg/String'). (Only ROS2)
  "tcp_no_delay": bool,       // use the TCP_NODELAY transport hint when subscribing to topics (Only ROS1)
  "use_sim_time": bool,       // Enable ROS simulation time (Only ROS2)
  "filter": SubscriberFilter, // filters applied to received message
  "qos": RosQos               // ROS2 only
}
```

[RosQos](#rosqos), [SubscriberFilter](#subscriberfilter)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/SubscriberNode.ts)

### SubscriberFilter

```json
{
  "no_data": bool,        // report only statistics without message content.
  "no_arr": bool,         // exclude arrays.
  "no_str": bool,         // exclude string fields.
  "hz": float,            // rate to forward messages. Ignored on latched topics. Disabled by 0. Default: 1
  "window": int,          // window size, in # of messages, for calculating rate
  "arrayItemsCount": int  // reduce arrays to this count. Default: 15
}
```

[RosQos](#rosqos)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/SubscriberFilter.ts)

### SubscriberEvent

```json
{
  "topic": str,         // Name of the ROS topic to listen to (e.g. '/chatter').
  "message_type": str,  // Type of the ROS message (e.g. 'std_msgs/msg/String')
  "latched": bool,
  "data": Dict,         // structure of the ROS message as dictionary.
  "count": int,         // Count of received messages since the start.
  "rate": float,
  "bw": float,
  "bw_min": float,
  "bw_max": float,
  "delay": float,
  "delay_min": float,
  "delay_max": float,
  "size": float,
  "size_min": float,
  "size_max": float
}
```

[RosQos](#rosqos)

Definitions: [Daemon](../fkie_mas_pylib/fkie_mas_pylib/interface/runtime_interface.py), [GUI](./src/renderer/models/SubscriberEvent.ts)
