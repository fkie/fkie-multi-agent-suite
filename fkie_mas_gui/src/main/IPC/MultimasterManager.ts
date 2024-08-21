import log from "electron-log";
import { ARGUMENTS, getArgument } from "../CommandLineInterface";
import { ICredential } from "../models/ICredential";
import CommandExecutor from "./CommandExecutor";
import { ROSInfo } from "./ROSInfo";
import TerminalManager from "./TerminalManager";

/**
 * Class MultimasterManager: Handles multimaster nodes
 */
class MultimasterManager {
  terminalManager: TerminalManager;

  commandExecutor: CommandExecutor;

  rosInfo: ROSInfo;

  respawn: boolean;

  constructor() {
    this.terminalManager = new TerminalManager();
    this.commandExecutor = new CommandExecutor();
    this.rosInfo = new ROSInfo();
    this.respawn = false; // respawn nodes

    log.debug(`ROS Info: ${this.rosInfo.toString()}`);
  }

  /**
   * Try to start a Terminal manager (default TTYD)
   *
   * @param {ICredential} credential - Credential to be used
   * @param {number} port - TTYD Port
   * @return {string} execution result
   */
  public startTerminalManager: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    port?: number
  ) => Promise<{ result: boolean; message: string }> = (rosVersion = null, credential = null, port = undefined) => {
    return this.terminalManager.spawnTerminal(rosVersion, credential, port);
  };

  /**
   * Try to start a master discovery node
   *
   * @param {ICredential} credential - Credential to be used
   * @param {string} name - Node name
   * @param {number} port - Port
   * @param {string} group - Multicast group
   * @param {number} heartbeatHz - Heartbeat in HZ
   * @return {bool} execution result
   */
  public startMasterDiscovery: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    name?: string,
    port?: number,
    group?: string,
    heartbeatHz?: number,
    robotHosts?: string[]
  ) => Promise<{ result: boolean; message: string }> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    port = undefined,
    group = undefined,
    heartbeatHz = undefined,
    robotHosts = undefined
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

    if (versStr === "1") {
      // port shift the default multicast port (11511)
      const dPort = `${Number(getArgument(ARGUMENTS.DISCOVERY_MCAST_PORT)) + (port || 0)}`;
      const dGroup = group || getArgument(ARGUMENTS.DISCOVERY_MCAST_GROUP);
      const dHeartbeat = heartbeatHz || getArgument(ARGUMENTS.DISCOVERY_HEARTBEAT_HZ);
      const dRobotHosts = robotHosts ? `_robot_hosts:=[${robotHosts}]` : "";
      cmdMasterDiscovery = `rosrun fkie_mas_daemon mas-remote-node.py ${
        this.respawn ? "--respawn" : ""
      } ${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery _mcast_port:=${dPort} _mcast_group:=${dGroup} ${dRobotHosts} _heartbeat_hz:=${dHeartbeat};`;
    } else if (versStr === "2") {
      let domainPrefix = "";
      if (port !== undefined && port !== 0) {
        domainPrefix = `ROS_DOMAIN_ID=${port} `;
      }
      cmdMasterDiscovery = `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ${domainPrefix}ros2 run fkie_mas_daemon mas-remote-node.py ${
        this.respawn ? "--respawn" : ""
      } ${nameArg} --set_name=false --node_type=mas-discovery --package=fkie_mas_discovery`;
    } else {
      return Promise.resolve({
        result: false,
        message: "Could not start [mas_discovery], ROS is not available",
      });
    }

    // combine commands and execute
    const cmd = `${cmdMasterDiscovery}`;
    log.info(`Starting Multimaster-Discovery: [${cmd}]`);
    return this.commandExecutor.exec(credential, cmd);
  };

  /**
   * Try to start a master sync node
   *
   * @param {ICredential} credential - Credential to be used
   * @param {string} name - Node name
   * @param {string[]} doNotSync - a list with topics to ignore while syncing
   * @param {string[]} syncTopics - a list with topics to sync
   * @return {bool} execution result
   */
  public startMasterSync: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    name?: string,
    doNotSync?: string[],
    syncTopics?: string[]
  ) => Promise<{ result: boolean; message: string }> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    doNotSync = undefined,
    syncTopics = undefined
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
      if (versStr === "1") {
        cmdMasterSync = `rosrun fkie_mas_daemon mas-remote-node.py  ${
          this.respawn ? "--respawn" : ""
        } ${nameArg} --set_name=false --node_type=mas-sync --package=fkie_mas_sync ${doNotSyncParam} ${syncTopicsParam};`;
      }
      // combine commands and execute
      const cmd = `${cmdMasterSync}`;
      log.info(`Starting Multimaster-Sync: [${cmd}]`);
      return this.commandExecutor.exec(credential, cmd);
    }
    return Promise.resolve({
      result: false,
      message: "Could not start [mas_sync], It is available only for ROS1",
    });
  };

  /**
   * Try to start a Daemon Node
   *
   * @param {ICredential} credential - Credential to be used
   * @param {string} name - Node name
   * @return {bool} execution result
   */
  public startMultimasterDaemon: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    name?: string,
    networkId?: number
  ) => Promise<{ result: boolean; message: string }> = (
    rosVersion = null,
    credential = null,
    name = undefined,
    networkId = 0
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
    // uses ROS1 method
    let cmdDaemon = `fkie_mas_daemon mas-remote-node.py ${
      this.respawn ? "--respawn" : ""
    } ${nameArg} --set_name=false --node_type=${daemonType} --package=fkie_mas_daemon`;

    let port = rosVersion === "2" ? 35430 : 35685;
    if (networkId !== undefined) {
      port += networkId;
    }
    if (versStr === "1") {
      cmdDaemon = `rosrun ${cmdDaemon} --port ${port}; `;
    } else if (versStr === "2") {
      if (networkId !== 0 && networkId !== undefined) {
        cmdDaemon = `ROS_DOMAIN_ID=${networkId} ros2 run ${cmdDaemon} --port ${port}; `;
      } else {
        cmdDaemon = `ros2 run ${cmdDaemon}; `;
      }
    } else {
      return Promise.resolve({
        result: false,
        message: "Could not start [mas-daemon], ROS is not available",
      });
    }

    log.info(`Starting Multimaster-Daemon: ${cmdDaemon}`);
    return this.commandExecutor.exec(credential, cmdDaemon);
  };

  /**
   * Try to start a Dynamic Reconfigure Node
   *
   * @param {ICredential} credential - Credential to be used
   * @param {string} name - Node name
   * @return {bool} execution result
   */
  public startDynamicReconfigureClient: (
    name: string,
    rosMasterUri: string,
    credential?: ICredential | null
  ) => Promise<{ result: boolean; message: string }> = async (name, rosMasterUri, credential = null) => {
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

  /**
   * Start background processes required for using multimaster LOCALLY
   */
  public startBackgroundProcesses = async (): Promise<void> => {
    try {
      log.info(await this.startTerminalManager());
      log.info(await this.startMultimasterDaemon());
      log.info(await this.startMasterDiscovery());
    } catch (error) {
      log.error(`startBackgroundProcesses Error: ${error}`);
    }
  };
}

export default MultimasterManager;
