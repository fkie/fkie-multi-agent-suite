import { TResult } from "@/types";

/**
 * ProviderLaunchConfiguration models launch configuration to start ROS system nodes
 */
class ProviderLaunchConfiguration {
  /** Name of the provider, usually set after the provider was launched. */
  providerName?: string;

  providerId?: string;

  host: string;

  /** If zero the port is depending on ROS version. */
  port: number = 0;

  /** Currently support websocket and 'crossbar-wamp' */
  type: string = "websocket";

  /** Use secure connection */
  useSSL: boolean = false;

  /** ROS version as string of {'1', '2'} */
  rosVersion: string = "1";

  networkId: number = import.meta.env.VITE_ROS_DOMAIN_ID ? parseInt(import.meta.env.VITE_ROS_DOMAIN_ID) : 0;

  daemon: {
    enable: boolean;
  } = { enable: true };

  discovery: {
    enable: boolean;
    group?: string;
    heartbeatHz?: number;
    robotHosts?: string[];
  } = { enable: true, robotHosts: [], heartbeatHz: 0.5 };

  sync: {
    enable: boolean;
    doNotSync: string[];
    syncTopics: string[];
  } = { enable: false, doNotSync: [], syncTopics: [] };

  terminal: {
    enable: boolean;
    port?: number;
    path?: string;
  } = { enable: true, path: "ttyd", port: 7681 };

  force: {
    stop: boolean;
    start: boolean;
  } = { stop: false, start: false };
  // forceStop = false;
  // forceStart = false;

  /** Try to connect on start. */
  autoConnect: boolean = true;

  /** Start system nodes on failed connection */
  autostart: boolean = false;

  ros1MasterUri: {
    enable: boolean;
    uri: string;
  } = { enable: false, uri: "default" };

  respawn: boolean = false;

  /**
   * Class Constructor
   *
   * @param {string} host - Parameter name
   * @param {string | number | boolean | string[]} value - Parameter value
   */
  constructor(host: string, rosVersion: string, port: number = 0) {
    this.host = host;
    this.rosVersion = rosVersion;
    this.port = port;
  }

  name: () => string = () => {
    return this.providerName ? this.providerName : this.host;
  };

  public terminalStartCmd: () => TResult = () => {
    const portNumber = this.terminal.port || 7681;
    const ttydPath = this.terminal.path || "ttyd";
    const ttydCmd = `${ttydPath} --writable --port ${portNumber} bash`;
    let cmd = "";
    if (this.rosVersion === "1") {
      cmd = `rosrun fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true;`;
    } else if (this.rosVersion === "2") {
      cmd = `ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true;`;
    } else {
      return {
        result: false,
        message: "Could not start [ttyd], ROS is not available",
      } as TResult;
    }
    return { result: true, message: cmd } as TResult;
  };

  public toRos1MasterUriPrefix: (ros1MasterUri: { enable: boolean; uri: string }) => string = (ros1MasterUri) => {
    let rosMasterUriPrefix = "";
    if (ros1MasterUri && ros1MasterUri.enable && ros1MasterUri.uri.length > 0 && ros1MasterUri.uri !== "default") {
      rosMasterUriPrefix = `ROS_MASTER_URI=${ros1MasterUri.uri.replace("{HOST}", this.host)} `;
    }
    return rosMasterUriPrefix;
  };

  /** Generate start command for a master discovery node */
  public masterDiscoveryStartCmd: () => TResult = () => {
    // Spawn a new Node Manager Master sync node
    const hostSuffix = "{HOST}";
    let dName = this.rosVersion === "2" ? `discovery_${hostSuffix}` : "mas_discovery";
    if (this.rosVersion === "2") {
      if (!dName.startsWith("_")) {
        dName = `_${dName}`;
      }
    }
    // uses ROS1 method
    let cmdMasterDiscovery: string | null = null;
    const namespace = this.rosVersion === "2" ? "/mas" : "";
    const nameArg = `--name=${namespace}/${dName}`;

    let domainPrefix = "";
    if (this.networkId !== undefined && this.networkId !== 0) {
      domainPrefix = `ROS_DOMAIN_ID=${this.networkId} `;
    }
    const forceArg = this.force.start ? "--force " : "";
    if (this.rosVersion === "1") {
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.ros1MasterUri);
      // networkId shift the default multicast port (11511)
      const dPort = `${11511 + this.networkId || 0}`;
      const dGroup = this.discovery.group || "226.0.0.0";
      const dHeartbeat = this.discovery.heartbeatHz;
      const dRobotHosts = this.discovery.robotHosts ? `_robot_hosts:=[${this.discovery.robotHosts}]` : "";
      cmdMasterDiscovery = `${ros1MasterUriPrefix}${domainPrefix}rosrun fkie_mas_daemon mas-remote-node.py ${
        this.respawn ? "--respawn" : ""
      } ${forceArg}${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery _mcast_port:=${dPort} _mcast_group:=${dGroup} ${dRobotHosts} _heartbeat_hz:=${dHeartbeat}`;
    } else if (this.rosVersion === "2") {
      cmdMasterDiscovery = `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ${domainPrefix}ros2 run fkie_mas_daemon mas-remote-node.py ${
        this.respawn ? "--respawn" : ""
      } ${forceArg}${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery`;
    } else {
      return {
        result: false,
        message: "Could not start [mas_discovery], ROS is not available",
      } as TResult;
    }

    // combine commands and execute
    const cmd = `${cmdMasterDiscovery}`;
    return { result: true, message: cmd } as TResult;
  };

  /** Generate start command for a master sync node */
  public masterSyncStartCmd: () => TResult = () => {
    if (this.rosVersion === "1") {
      // Spawn a new Node Manager Master sync node
      const dName = "mas_sync";
      // uses ROS1 method
      const namespace = "";
      const nameArg = `--name=${namespace}/${dName}`;
      const doNotSyncParam =
        this.sync.doNotSync && this.sync.doNotSync?.length > 0
          ? `_do_not_sync:=[${this.sync.doNotSync?.toString()}]`
          : " ";
      const syncTopicsParam =
        this.sync.syncTopics && this.sync.syncTopics?.length > 0
          ? `_sync_topics:=[${this.sync.syncTopics?.toString()}]`
          : " ";
      let cmdMasterSync = "";
      const forceArg = this.force.start ? "--force " : "";
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.ros1MasterUri);
      cmdMasterSync = `${ros1MasterUriPrefix}rosrun fkie_mas_daemon mas-remote-node.py  ${
        this.respawn ? "--respawn" : ""
      } ${forceArg}${nameArg} --set_name=false --node_type=mas-sync --package=fkie_mas_sync ${doNotSyncParam} ${syncTopicsParam}`;
      // combine commands and execute
      const cmd = `${cmdMasterSync}`;
      return { result: true, message: cmd } as TResult;
    }
    return {
      result: false,
      message: "Could not start [mas_sync], It is available only for ROS1",
    } as TResult;
  };

  /** Generate start command for a Daemon Node */
  public daemonStartCmd: () => TResult = () => {
    const hostSuffix = "{HOST}";
    let dName = this.rosVersion === "2" ? `daemon_${hostSuffix}` : "mas_daemon";
    if (this.rosVersion === "2") {
      if (!dName.startsWith("_")) {
        dName = `_${dName}`;
      }
    }
    let daemonType = "mas-daemon";
    const namespace = this.rosVersion === "2" ? "/mas" : "";
    const nameArg = `--name=${namespace}/${dName}`;
    if (this.rosVersion === "2") {
      daemonType = "mas-daemon";
    }
    const forceArg = this.force.start ? "--force " : "";
    // uses ROS1 method
    let cmdDaemon = `fkie_mas_daemon mas-remote-node.py ${
      this.respawn ? "--respawn" : ""
    } ${forceArg}${nameArg} --set_name=false --node_type=${daemonType} --package=fkie_mas_daemon`;

    let domainPrefix = "";
    if (this.networkId !== undefined && this.networkId !== 0) {
      domainPrefix = `ROS_DOMAIN_ID=${this.networkId} `;
    }
    if (this.rosVersion === "1") {
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.ros1MasterUri);
      cmdDaemon = `${ros1MasterUriPrefix}${domainPrefix}rosrun ${cmdDaemon}`;
    } else if (this.rosVersion === "2") {
      cmdDaemon = `${domainPrefix}ros2 run ${cmdDaemon}; `;
    } else {
      return {
        result: false,
        message: "Could not start [mas-daemon], ROS is not available",
      } as TResult;
    }
    return { result: true, message: cmdDaemon } as TResult;
  };

  /** Generate start command for a Dynamic Reconfigure Node */
  public dynamicReconfigureClientCmd: (nodeName: string, ros1MasterUri: string) => TResult = (
    nodeName,
    ros1MasterUri
  ) => {
    const nameArg = `--name=/dynamic_reconfigure${nodeName}`;
    const envArg = `ROS_MASTER_URI=${ros1MasterUri}`;
    const cmd = `${envArg} rosrun fkie_mas_daemon mas-remote-node.py ${nameArg} --node_type=dynamic-reconfigure.py --package=fkie_mas_daemon ${nodeName} `;
    return { result: true, message: cmd } as TResult;
  };
}

export default ProviderLaunchConfiguration;
