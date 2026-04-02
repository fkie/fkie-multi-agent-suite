import { TResult } from "@/types";
import { generateUniqueId } from "../utils";

export const RMW_SELECTIONS = [
  "RMW_IMPLEMENTATION",
  "rmw_connextdds",
  "rmw_cyclonedds_cpp",
  "rmw_fastrtps_cpp",
  "rmw_zenoh_cpp",
] as const;

export type RmwSelection = (typeof RMW_SELECTIONS)[number];

export type TProviderLaunchParams = {
  id: string;

  name: string | undefined;

  /** Name of the provider, usually set after the provider was launched. */
  providerName: string | undefined;

  providerId: string | undefined;

  host: string;

  /** If zero the port is depending on ROS version. */
  port: number;

  /** Currently support websocket */
  type: string;

  /** Use secure connection */
  useSSL: boolean;

  /** ROS version as string of {'1', '2'} */
  rosVersion: string;

  domainId: number;

  networkId?: number; // deprecated

  rmw: {
    current: RmwSelection;
    selected: RmwSelection;
    forceUse: boolean;
    overrideZeno: boolean;
  };

  daemon: {
    enable: boolean;
  };

  discovery: {
    enable: boolean;
    group?: string;
    heartbeatHz?: number;
    robotHosts?: string[];
    respawn: boolean;
  };

  sync: {
    enable: boolean;
    doNotSync: string[];
    syncTopics: string[];
  };

  terminal: {
    enable: boolean;
    port?: number;
    path?: string;
  };

  force: {
    stop: boolean;
    start: boolean;
  };

  /** Try to connect on start. */
  autoConnect: boolean;

  /** Start system nodes on failed connection */
  autostart: boolean;

  ros1MasterUri: {
    enable: boolean;
    uri: string;
  };

  respawn: boolean;

  zenohConfigOverride: string;
};

/**
 * ProviderLaunchConfiguration models launch configuration to start ROS system nodes
 */
export default class ProviderLaunchConfiguration {
  params: TProviderLaunchParams;

  constructor(
    params: TProviderLaunchParams = {
      id: generateUniqueId(),
      name: undefined,
      providerName: undefined,
      providerId: undefined,
      host: "localhost",
      port: 0,
      domainId: -1,
      type: "websocket",
      useSSL: false,
      rosVersion: "2",
      rmw: {
        current: "RMW_IMPLEMENTATION",
        selected: "RMW_IMPLEMENTATION",
        forceUse: false,
        overrideZeno: true,
      },
      daemon: { enable: true },
      discovery: { enable: true, robotHosts: [], heartbeatHz: 0.5, respawn: true },
      sync: { enable: false, doNotSync: [], syncTopics: [] },
      terminal: { enable: true, path: "ttyd", port: 8681 },
      force: { stop: false, start: false },
      autoConnect: true,
      autostart: false,
      ros1MasterUri: { enable: false, uri: "default" },
      respawn: false,
      zenohConfigOverride: "",
    }
  ) {
    this.params = params;
  }

  name: () => string = () => {
    return this.params.providerName ? this.params.providerName : this.params.host;
  };

  public terminalStartCmd(): TResult {
    const portNumber = this.params.terminal.port || 8681;
    const ttydPath = this.params.terminal.path || "ttyd";
    const ttydCmd = `${ttydPath} --writable --port ${portNumber} bash`;
    let cmd = "";
    if (this.params.rosVersion === "1") {
      cmd = `rosrun fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true;`;
    } else if (this.params.rosVersion === "2") {
      cmd = `ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true;`;
    } else {
      return {
        result: false,
        message: "Could not start [ttyd], ROS is not available",
      } as TResult;
    }
    return { result: true, message: cmd } as TResult;
  }

  public toRos1MasterUriPrefix(ros1MasterUri: { enable: boolean; uri: string }): string {
    let rosMasterUriPrefix = "";
    if (ros1MasterUri?.enable && ros1MasterUri.uri.length > 0 && ros1MasterUri.uri !== "default") {
      rosMasterUriPrefix = `ROS_MASTER_URI=${ros1MasterUri.uri.replace("{HOST}", this.params.host)} `;
    }
    return rosMasterUriPrefix;
  }

  /** Generate start command for a master discovery node */
  public masterDiscoveryStartCmd(): TResult {
    // Spawn a new Node Manager Master sync node
    const hostSuffix = "{HOST}";
    let dName = this.params.rosVersion === "2" ? `discovery_${hostSuffix}` : "mas_discovery";
    if (this.params.rosVersion === "2") {
      if (!dName.startsWith("_")) {
        dName = `_${dName}`;
      }
    }
    // uses ROS1 method
    let cmdMasterDiscovery: string | null = null;
    const namespace = this.params.rosVersion === "2" ? "/mas" : "";
    const nameArg = `--name=${namespace}/${dName}`;

    const forceArg = this.params.force.start ? "--force " : "";
    if (this.params.rosVersion === "1") {
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.params.ros1MasterUri);
      // domainId shift the default multicast port (11511)
      const dPort = `${11511 + this.params.domainId || 0}`;
      const dGroup = this.params.discovery.group || "226.0.0.0";
      const dHeartbeat = this.params.discovery.heartbeatHz;
      const dRobotHosts = this.params.discovery.robotHosts ? `_robot_hosts:=[${this.params.discovery.robotHosts}]` : "";
      cmdMasterDiscovery = `${ros1MasterUriPrefix}${this.domainPrefix()} rosrun fkie_mas_daemon mas-remote-node.py ${
        this.params.respawn ? "--respawn" : ""
      } ${forceArg}${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery _mcast_port:=${dPort} _mcast_group:=${dGroup} ${dRobotHosts} _heartbeat_hz:=${dHeartbeat}`;
    } else if (this.params.rosVersion === "2") {
      cmdMasterDiscovery = `${this.getEnvPrefix()}ros2 run fkie_mas_daemon mas-remote-node.py ${
        this.params.respawn || this.params.discovery.respawn ? "--respawn" : ""
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
  }

  public fixStringArray(val: string | number | boolean | string[]): string | number | boolean | string[] {
    if (Array.isArray(val)) {
      return val.map((v) => `'${v}'`).join();
    }
    return val;
  }

  /** Generate start command for a master sync node */
  public masterSyncStartCmd(): TResult {
    if (this.params.rosVersion === "1") {
      // Spawn a new Node Manager Master sync node
      const dName = "mas_sync";
      // uses ROS1 method
      const namespace = "";
      const nameArg = `--name=${namespace}/${dName}`;
      const doNotSyncParam =
        this.params.sync.doNotSync && this.params.sync.doNotSync?.length > 0
          ? `_do_not_sync:=[${this.fixStringArray(this.params.sync.doNotSync)?.toString()}]`
          : " ";
      const syncTopicsParam =
        this.params.sync.syncTopics && this.params.sync.syncTopics?.length > 0
          ? `_sync_topics:=[${this.fixStringArray(this.params.sync.syncTopics)?.toString()}]`
          : " ";
      let cmdMasterSync = "";
      const forceArg = this.params.force.start ? "--force " : "";
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.params.ros1MasterUri);
      cmdMasterSync = `${ros1MasterUriPrefix}${this.domainPrefix()} rosrun fkie_mas_daemon mas-remote-node.py  ${
        this.params.respawn ? "--respawn" : ""
      } ${forceArg}${nameArg} --set_name=false --node_type=mas-sync --package=fkie_mas_sync ${doNotSyncParam} ${syncTopicsParam}`;
      // combine commands and execute
      const cmd = `${cmdMasterSync}`;
      return { result: true, message: cmd } as TResult;
    }
    return {
      result: false,
      message: "Could not start [mas_sync], It is available only for ROS1",
    } as TResult;
  }

  /** Generate start command for a Daemon Node */
  public daemonStartCmd(): TResult {
    const hostSuffix = "{HOST}";
    let dName = this.params.rosVersion === "2" ? `daemon_${hostSuffix}` : "mas_daemon";
    if (this.params.rosVersion === "2") {
      if (!dName.startsWith("_")) {
        dName = `_${dName}`;
      }
    }
    let daemonType = "mas-daemon";
    const namespace = this.params.rosVersion === "2" ? "/mas" : "";
    const nameArg = `--name=${namespace}/${dName}`;
    if (this.params.rosVersion === "2") {
      daemonType = "mas-daemon";
    }
    const forceArg = this.params.force.start ? "--force " : "";
    // uses ROS1 method
    let cmdDaemon = `fkie_mas_daemon mas-remote-node.py ${
      this.params.respawn ? "--respawn" : ""
    } ${forceArg}${nameArg} --set_name=false --node_type=${daemonType} --package=fkie_mas_daemon`;

    if (this.params.rosVersion === "1") {
      const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(this.params.ros1MasterUri);
      cmdDaemon = `${ros1MasterUriPrefix}${this.domainPrefix()} rosrun ${cmdDaemon}`;
    } else if (this.params.rosVersion === "2") {
      cmdDaemon = `${this.getEnvPrefix()}ros2 run ${cmdDaemon}; `;
    } else {
      return {
        result: false,
        message: "Could not start [mas-daemon], ROS is not available",
      } as TResult;
    }
    return { result: true, message: cmdDaemon } as TResult;
  }

  /** Generate start command for a Dynamic Reconfigure Node */
  public dynamicReconfigureClientCmd(nodeName: string, ros1MasterUri: string): TResult {
    const nameArg = `--name=/dynamic_reconfigure${nodeName}`;
    const envArg = `ROS_MASTER_URI=${ros1MasterUri}`;
    const cmd = `${envArg}${this.domainPrefix()} rosrun fkie_mas_daemon mas-remote-node.py ${nameArg} --node_type=dynamic-reconfigure.py --package=fkie_mas_daemon ${nodeName} `;
    return { result: true, message: cmd } as TResult;
  }

  public getEnvPrefix() {
    const prefix = this.getEnv().join(" ");
    if (!prefix) return "";
    return ` ${prefix} `;
  }

  public getEnv() {
    const result: string[] = [
      this.domainPrefix(),
      this.rmwPrefix(),
      this.getZenohRouterCheck(),
      this.getZenohOverride(),
    ].filter((entry) => !!entry);
    return result;
  }

  public domainPrefix(): string {
    if (this.params.domainId >= 0) {
      return `ROS_DOMAIN_ID=${this.params.domainId}`;
    }
    return "";
  }

  public rmwPrefix(): string {
    if (this.params.rmw.forceUse && this.params.rmw.selected !== "RMW_IMPLEMENTATION") {
      return `RMW_IMPLEMENTATION=${this.params.rmw.selected}`;
    }
    return "";
  }

  public getZenohRouterCheck(): string {
    if (!this.params.rmw.overrideZeno || this.params.rmw.current !== "rmw_zenoh_cpp") return "";

    return "ZENOH_ROUTER_CHECK_ATTEMPTS=-1";
  }

  public getZenohOverride(): string {
    if (!this.params.rmw.overrideZeno || this.params.rmw.current !== "rmw_zenoh_cpp") return "";

    const zenohPort = 7448 + this.params.domainId || 0;
    let envParam = "";
    if (this.params.zenohConfigOverride) {
      envParam = `ZENOH_CONFIG_OVERRIDE='${this.params.zenohConfigOverride.replace("${ZENOH_PORT}", `${zenohPort}`)}'`;
    }
    return envParam;
  }
}
