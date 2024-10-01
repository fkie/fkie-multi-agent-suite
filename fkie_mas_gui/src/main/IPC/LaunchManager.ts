import { LaunchManagerEvents, TCredential, TLaunchManager, TResult } from "@/types";
import { ipcMain } from "electron";
import log from "electron-log";
import { ARGUMENTS, getArgument, hasArgument } from "../CommandLineInterface";
import CommandExecutor from "./CommandExecutor";
import { ROSInfo } from "./ROSInfo";

/**
 * Start system nodes of the multi agent suite.
 */
export default class LaunchManager implements TLaunchManager {
  commandExecutor: CommandExecutor;

  rosInfo: ROSInfo;

  respawn: boolean;

  constructor() {
    this.commandExecutor = new CommandExecutor();
    this.rosInfo = new ROSInfo();
    this.respawn = false; // respawn nodes

    log.debug(`ROS Info: ${this.rosInfo.toString()}`);
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(
      LaunchManagerEvents.startTerminalManager,
      (_event, rosVersion: string, credential: TCredential, port?: number) => {
        return this.startTerminalManager(rosVersion, credential, port);
      }
    );

    ipcMain.handle(
      LaunchManagerEvents.startDaemon,
      (
        _event,
        rosVersion: string,
        credential: TCredential,
        name?: string,
        networkId?: number,
        ros1MasterUri?: string,
        forceStart?: boolean
      ) => {
        return this.startDaemon(
          rosVersion,
          credential,
          name,
          networkId,
          ros1MasterUri,
          forceStart
        );
      }
    );

    ipcMain.handle(
      LaunchManagerEvents.startMasterDiscovery,
      (
        _event,
        rosVersion: string,
        credential: TCredential,
        name?: string,
        port?: number,
        group?: string,
        heartbeatHz?: number,
        robotHosts?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) => {
        return this.startMasterDiscovery(
          rosVersion,
          credential,
          name,
          port,
          group,
          heartbeatHz,
          robotHosts,
          ros1MasterUri,
          forceStart
        );
      }
    );

    ipcMain.handle(
      LaunchManagerEvents.startMasterSync,
      (
        _event,
        rosVersion: string,
        credential: TCredential,
        name?: string,
        doNotSync?: string[],
        syncTopics?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) => {
        return this.startMasterSync(
          rosVersion,
          credential,
          name,
          doNotSync,
          syncTopics,
          ros1MasterUri,
          forceStart
        );
      }
    );

    ipcMain.handle(
      LaunchManagerEvents.startDynamicReconfigureClient,
      (_event, name: string, rosMasterUri: string, credential?: TCredential | null) => {
        return this.startDynamicReconfigureClient(name, rosMasterUri, credential);
      }
    );
  }

  /** Try to start a Terminal manager (default TTYD) */
  public startTerminalManager: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    port?: number
  ) => Promise<TResult> = (rosVersion = null, credential = null, port = undefined) => {
    let version = rosVersion;
    if (!version) {
      version = this.rosInfo.version === "1" ? "1" : "2";
      log.debug(`use ROS version: ${version}`);
    }
    const portNumber = port || getArgument(ARGUMENTS.TTYD_PORT);

    const ttydCmd = `${this.getPathTTY()} --writable --port ${portNumber} bash`;

    let cmd = "";
    if (version === "1") {
      cmd = `rosrun fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else if (version === "2") {
      cmd = `ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else {
      return Promise.resolve({
        result: false,
        message: "Could not start [ttyd], ROS is not available",
      });
    }

    log.info(`Starting [ttyd-${portNumber}]: [${cmd}]`);
    return this.commandExecutor.exec(credential, cmd);
  };

  /**
 * Returns the path of the suitable executable for the current platform
 *
 * @return {string} Path to the executable
 */
  private getPathTTY: () => string = () => {
    if (hasArgument(ARGUMENTS.TTYD_PATH)) {
      const path = getArgument(ARGUMENTS.TTYD_PATH);
      if (path) return path;
    }

    // TODO Add executables and paths for windows and MAC
    return "ttyd";
  };

  public toRos1MasterUriPrefix: (ros1MasterUri: string | undefined, credential: TCredential | null) => string = (
    ros1MasterUri,
    credential
  ) => {
    let rosMasterUriPrefix = "";
    if (ros1MasterUri && ros1MasterUri.length > 0 && ros1MasterUri !== "default") {
      rosMasterUriPrefix = `ROS_MASTER_URI=${ros1MasterUri.replace("{HOST}", credential ? credential.host : "localhost")} `;
    }
    return rosMasterUriPrefix;
  };

  /** Try to start a master discovery node */
  public startMasterDiscovery: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    networkId?: number,
    group?: string,
    heartbeatHz?: number,
    robotHosts?: string[],
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    networkId = undefined,
    group = undefined,
    heartbeatHz = undefined,
    robotHosts = undefined,
    ros1MasterUri = undefined,
    forceStart = undefined
  ) => {
      let versStr = rosVersion;
      if (!versStr) {
        versStr = this.rosInfo.version === "1" ? "1" : "2";
        log.debug(`use ROS version: ${versStr}`);
      }
      // Spawn a new Node Manager Master sync node
      const hostSuffix = "{HOST}";
      let dName = name || versStr === "2" ? `discovery_${hostSuffix}` : "mas_discovery";
      if (versStr === "2") {
        if (!dName.startsWith("_")) {
          dName = `_${dName}`;
        }
      }
      // uses ROS1 method
      let cmdMasterDiscovery: string | null = null;
      const namespace = versStr === "2" ? "/mas" : "";
      const nameArg = `--name=${namespace}/${dName}`;

      let domainPrefix = "";
      if (networkId !== undefined && networkId !== 0) {
        domainPrefix = `ROS_DOMAIN_ID=${networkId} `;
      }
      const forceArg = forceStart ? "--force " : "";
      if (versStr === "1") {
        const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(ros1MasterUri, credential);
        // networkId shift the default multicast port (11511)
        const dPort = `${Number(getArgument(ARGUMENTS.DISCOVERY_MCAST_PORT)) + (networkId || 0)}`;
        const dGroup = group || getArgument(ARGUMENTS.DISCOVERY_MCAST_GROUP);
        const dHeartbeat = heartbeatHz || getArgument(ARGUMENTS.DISCOVERY_HEARTBEAT_HZ);
        const dRobotHosts = robotHosts ? `_robot_hosts:=[${robotHosts}]` : "";
        cmdMasterDiscovery = `${ros1MasterUriPrefix}${domainPrefix}rosrun fkie_mas_daemon mas-remote-node.py ${this.respawn ? "--respawn" : ""
          } ${forceArg}${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery _mcast_port:=${dPort} _mcast_group:=${dGroup} ${dRobotHosts} _heartbeat_hz:=${dHeartbeat};`;
      } else if (versStr === "2") {
        cmdMasterDiscovery = `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ${domainPrefix}ros2 run fkie_mas_daemon mas-remote-node.py ${this.respawn ? "--respawn" : ""
          } ${forceArg}${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery`;
      } else {
        return Promise.resolve({
          result: false,
          message: "Could not start [mas_discovery], ROS is not available",
        });
      }

      // combine commands and execute
      const cmd = `${cmdMasterDiscovery}`;
      log.info(`Starting Master-Discovery: [${cmd}]`);
      return this.commandExecutor.exec(credential, cmd);
    };

  /** Try to start a master sync node */
  public startMasterSync: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    doNotSync?: string[],
    syncTopics?: string[],
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    doNotSync = undefined,
    syncTopics = undefined,
    ros1MasterUri = undefined,
    forceStart = undefined
  ) => {
      let versStr = rosVersion;
      if (!versStr) {
        versStr = this.rosInfo.version === "1" ? "1" : "2";
        log.debug(`use ROS version: ${versStr}`);
      }
      if (versStr === "1") {
        // Spawn a new Node Manager Master sync node
        const dName = name || "mas_sync";
        // uses ROS1 method
        const namespace = "";
        const nameArg = `--name=${namespace}/${dName}`;
        const doNotSyncParam = doNotSync && doNotSync?.length > 0 ? `_do_not_sync:=[${doNotSync?.toString()}]` : " ";
        const syncTopicsParam = syncTopics && syncTopics?.length > 0 ? `_sync_topics:=[${syncTopics?.toString()}]` : " ";
        let cmdMasterSync = "";
        const forceArg = forceStart ? "--force " : "";
        if (versStr === "1") {
          const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(ros1MasterUri, credential);
          cmdMasterSync = `${ros1MasterUriPrefix}rosrun fkie_mas_daemon mas-remote-node.py  ${this.respawn ? "--respawn" : ""
            } ${forceArg}${nameArg} --set_name=false --node_type=mas-sync --package=fkie_mas_sync ${doNotSyncParam} ${syncTopicsParam};`;
        }
        // combine commands and execute
        const cmd = `${cmdMasterSync}`;
        log.info(`Starting Master-Sync: [${cmd}]`);
        return this.commandExecutor.exec(credential, cmd);
      }
      return Promise.resolve({
        result: false,
        message: "Could not start [mas_sync], It is available only for ROS1",
      });
    };

  /** Try to start a Daemon Node */
  public startDaemon: (
    rosVersion?: string | null,
    credential?: TCredential | null,
    name?: string,
    networkId?: number,
    ros1MasterUri?: string,
    forceStart?: boolean
  ) => Promise<TResult> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    networkId = 0,
    ros1MasterUri = undefined,
    forceStart = false
  ) => {
      let versStr = rosVersion;
      if (!versStr) {
        versStr = this.rosInfo.version === "1" ? "1" : "2";
        log.debug(`use ROS version: ${versStr}`);
      }
      // Spawn Daemon node
      const hostSuffix = "{HOST}";
      let dName = name || versStr === "2" ? `daemon_${hostSuffix}` : "mas_daemon";
      if (versStr === "2") {
        if (!dName.startsWith("_")) {
          dName = `_${dName}`;
        }
      }
      let daemonType = "mas-daemon";
      const namespace = versStr === "2" ? "/mas" : "";
      const nameArg = `--name=${namespace}/${dName}`;
      if (versStr === "2") {
        daemonType = "mas-daemon";
      }
      const forceArg = forceStart ? "--force " : "";
      // uses ROS1 method
      let cmdDaemon = `fkie_mas_daemon mas-remote-node.py ${this.respawn ? "--respawn" : ""
        } ${forceArg}${nameArg} --set_name=false --node_type=${daemonType} --package=fkie_mas_daemon`;

      let domainPrefix = "";
      if (networkId !== undefined && networkId !== 0) {
        domainPrefix = `ROS_DOMAIN_ID=${networkId} `;
      }
      if (versStr === "1") {
        const ros1MasterUriPrefix = this.toRos1MasterUriPrefix(ros1MasterUri, credential);
        cmdDaemon = `${ros1MasterUriPrefix}${domainPrefix}rosrun ${cmdDaemon}`;
      } else if (versStr === "2") {
        cmdDaemon = `${domainPrefix}ros2 run ${cmdDaemon}; `;
      } else {
        return Promise.resolve({
          result: false,
          message: "Could not start [mas-daemon], ROS is not available",
        });
      }

      log.info(`Starting Master-Daemon: ${cmdDaemon}`);
      return this.commandExecutor.exec(credential, cmdDaemon);
    };

  /** Try to start a Dynamic Reconfigure Node */
  public startDynamicReconfigureClient: (
    name: string,
    rosMasterUri: string,
    credential?: TCredential | null
  ) => Promise<TResult> = async (name, rosMasterUri, credential = null) => {
    const nameArg = `--name=/dynamic_reconfigure${name}`;
    const envArg = `ROS_MASTER_URI=${rosMasterUri}`;
    const cmd = `${envArg} rosrun fkie_mas_daemon mas-remote-node.py ${nameArg} --node_type=dynamic-reconfigure.py --package=fkie_mas_daemon ${name}; `;

    log.info(`Starting Dynamic Reconfigure: ${cmd}`);
    try {
      const result = await this.commandExecutor.exec(credential, cmd);
      return result;
    } catch (error) {
      return Promise.resolve({ result: false, message: `${error}` });
    }
  };
}

