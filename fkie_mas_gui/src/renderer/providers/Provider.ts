/* eslint-disable max-classes-per-file */
import { TSystemInfo, JSONObject, TResultData } from "@/types";
import { emitCustomEvent } from "react-custom-events";
import { TagColors } from "../components/UI/Colors";
import { DEFAULT_BUG_TEXT, ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import {
  DaemonVersion,
  DiagnosticArray,
  FileItem,
  LaunchArgument,
  LaunchCallService,
  LaunchContent,
  LaunchFile,
  LaunchIncludedFile,
  LaunchIncludedFilesRequest,
  LaunchLoadReply,
  LaunchLoadRequest,
  LaunchMessageStruct,
  LaunchNode,
  LaunchNodeInfo,
  LaunchNodeReply,
  LaunchPublishMessage,
  LogPathItem,
  LoggerConfig,
  PathEvent,
  PathItem,
  ProviderLaunchConfiguration,
  Result,
  RosNode,
  RosNodeStatus,
  RosPackage,
  RosParameter,
  RosService,
  RosTopic,
  ScreensMapping,
  SubscriberEvent,
  SubscriberFilter,
  SubscriberNode,
  SystemWarningGroup,
  URI,
} from "../models";
import { delay, generateUniqueId } from "../utils";
import CmdTerminal from "./CmdTerminal";
import CmdType from "./CmdType";
import { ConnectionState } from "./ConnectionState";
import ProviderConnection, { TProviderTimestamp, TResultClearPath, TResultStartNode } from "./ProviderConnection";
import { RosProviderState } from "./RosProviderState";
import CrossbarIOConnection from "./crossbar_io/CrossbarIOConnection";
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_DELAY,
  EVENT_PROVIDER_DISCOVERED,
  EVENT_PROVIDER_LAUNCH_LIST,
  EVENT_PROVIDER_LAUNCH_LOADED,
  EVENT_PROVIDER_NODE_STARTED,
  EVENT_PROVIDER_PATH_EVENT,
  EVENT_PROVIDER_REMOVED,
  EVENT_PROVIDER_ROS_NODES,
  EVENT_PROVIDER_SCREENS,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX,
  EVENT_PROVIDER_TIME_DIFF,
  EVENT_PROVIDER_WARNINGS,
} from "./eventTypes";
import {
  EventProviderActivity,
  EventProviderDelay,
  EventProviderDiscovered,
  EventProviderLaunchList,
  EventProviderLaunchLoaded,
  EventProviderNodeStarted,
  EventProviderPathEvent,
  EventProviderRemoved,
  EventProviderRosNodes,
  EventProviderScreens,
  EventProviderState,
  EventProviderSubscriberEvent,
  EventProviderTimeDiff,
  EventProviderWarnings,
} from "./events";
import WebsocketConnection from "./websocket/WebsocketConnection";

type TProviderDaemonReady = {
  status: boolean;
  timestamp: number;
};

type TProviderDiscoveryReady = {
  status: boolean;
};

export type TConCallback = {
  uri: string;
  callback: (msg: JSONObject) => void;
};

interface IProvider {
  className: string;
  getCallbacks: () => TConCallback[];
  updateDaemonInit: () => void;
}

/**
 * Provider base class to connect with a MAS daemon
 */
export default class Provider implements IProvider {
  static defaultType: string = "websocket";

  className: string = "Provider";

  type: string = Provider.defaultType;

  IGNORED_NODES: string[] = [];

  /**
   * Unique Identifier
   */
  id: string;

  /** Name of the provider applied by the user. */
  userName: string = "";

  /** ROS version running on the host provided by this provider. */
  rosVersion: string;

  connection: ProviderConnection = new ProviderConnection();

  isLocalHost: boolean = false;

  /** Indicates if provider was created by user or discovered.
   * undefined: created by user
   * string[]: list of providers which discovered this provider
   * empty list: This provider was discovered, but should be removed as the parent provider is disconnected.
   */
  discovered: string[] | undefined = undefined;

  /** State of the connection to remote provider. */
  connectionState: ConnectionState = ConnectionState.STATES.UNKNOWN;

  /** State provided from ROS provider */
  rosState: RosProviderState = new RosProviderState();

  daemonVersion: DaemonVersion = new DaemonVersion("", "");

  /**
   * True if the provider has access to a node manager's daemon
   */
  daemon: boolean = false;

  /**
   * True if the provider has access to a node manager's master discovery
   */
  discovery: boolean = false;

  /**
   * Launch files loaded on provider
   */
  launchFiles: LaunchContent[] = [];

  rosNodes: RosNode[] = [];

  /**
   * Package list
   */
  packages?: RosPackage[];

  /** Providers discovered by this provider.
   * For each provider in this list an event (EVENT_PROVIDER_DISCOVERED) will be emitted. */
  // eslint-disable-next-line no-use-before-define
  remoteProviders: Provider[] = [];

  screens: ScreensMapping[] = [];

  systemInfo: TSystemInfo = {};

  systemEnv: object = {};

  /** All known hostnames for this provides, e.g. IPv4 IPv6 or names */
  hostnames: string[] = [];

  /** Time difference in milliseconds to local host. */
  timeDiff: number = 0;

  /** Time difference in seconds calculated if daemon.ready received. */
  currentDelay: number = 0;

  /** Warnings reported by the provider. */
  warnings: SystemWarningGroup[] = [];

  /**
   * External logger
   */
  logger: ILoggingContext | null;

  settings: ISettingsContext;

  callAttempts: number = 10;

  delayCallAttempts: number = 1000;

  errorDetails: string = "";

  startConfiguration: ProviderLaunchConfiguration | null = null;

  showRemoteNodes: boolean = false;

  // started echo topics to the received echo events
  private echoTopics: string[] = [];

  /**
   * Keep tracks of running async request, to prevent multiple executions
   */
  private currentRequestList = new Set();

  /**
   * constructor that initializes a new instance of a provider object.
   *
   * @param {ISettingsContext} settings - External settings
   * @param {string} host - IP address or hostname of a remote server on remote host.
   * @param {string} rosVersion - ROS version as string of {'1', '2'}
   * @param {number} port - Port of a remote server on remote host. If zero, it depends on the ros version.
   * @param {ILoggingContext | null} logger - External logger
   */
  constructor(
    settings: ISettingsContext,
    host: string,
    rosVersion: string,
    port: number = 0,
    useSSL: boolean = false,
    logger: ILoggingContext | null = null
  ) {
    this.logger = logger;
    this.settings = settings;
    this.rosVersion = rosVersion;
    this.hostnames = [host];
    this.id = `${host}-${generateUniqueId()}`;
    if (this.type === CrossbarIOConnection.type) {
      this.connection = new CrossbarIOConnection(
        host,
        rosVersion,
        port,
        useSSL,
        this.onCloseConnection,
        this.onOpenConnection,
        logger
      );
    } else {
      this.connection = new WebsocketConnection(
        host,
        rosVersion,
        port,
        useSSL,
        this.onCloseConnection,
        this.onOpenConnection,
        logger
      );
    }
  }

  public getCallbacks: () => TConCallback[] = () => {
    return [
      { uri: URI.ROS_PROVIDER_LIST, callback: this.updateProviderList },
      { uri: URI.ROS_DAEMON_READY, callback: this.callbackDaemonReady },
      { uri: URI.ROS_DISCOVERY_READY, callback: this.callbackDiscoveryReady },
      { uri: URI.ROS_LAUNCH_CHANGED, callback: this.updateRosNodes },
      { uri: URI.ROS_NODES_CHANGED, callback: this.updateRosNodes },
      { uri: URI.ROS_PATH_CHANGED, callback: this.callbackChangedFile },
      { uri: URI.ROS_SCREEN_LIST, callback: this.callbackScreensUpdate },
      { uri: URI.ROS_PROVIDER_WARNINGS, callback: this.callbackProviderWarnings },
      { uri: URI.ROS_PROVIDER_DIAGNOSTICS, callback: this.callbackDiagnosticsUpdate },
    ];
  };

  public updateDaemonInit: () => void = async () => {
    this.getDaemonVersion()
      .then((dv) => {
        if (this.isAvailable()) {
          this.daemonVersion = dv;
          this.updateSystemWarnings();
          this.updateTimeDiff();
          this.getProviderSystemInfo();
          this.getProviderSystemEnv();
          // this.updateRosNodes({});
          this.updateDiagnostics(null);
          // this.getPackageList();  <- this request is done by package explorer
          this.updateScreens(null);
          // this.launchGetList();
          this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          this.updateProviderList();
          return true;
        }
        return false;
      })
      .catch((error) => {
        this.setConnectionState(ConnectionState.STATES.ERRORED, `Daemon no reachable: ${error}`);
        return false;
      });
  };

  public url: () => string = () => {
    return this.connection.uri;
  };

  public name: () => string = () => {
    let name = this.userName;
    if (!name) {
      name = this.rosState.name ? this.rosState.name : this.connection.host;
    }
    return name;
  };

  public host: () => string = () => {
    return this.connection.host;
  };

  public setName: (name: string) => void = (name: string) => {
    this.userName = name;
  };

  /** Creates a command string to open screen or log in a terminal */
  public cmdForType: (
    type: CmdType,
    nodeName: string,
    topicName: string,
    screenName: string,
    cmd: string
  ) => Promise<CmdTerminal> = async (type, nodeName = "", topicName = "", screenName = "", cmd = "") => {
    const result = new CmdTerminal();
    if (type === CmdType.SCREEN && screenName === "") {
      type = CmdType.LOG;
    }
    switch (type) {
      case CmdType.CMD:
        result.cmd = `${cmd}`;
        break;
      case CmdType.SCREEN:
        if (screenName && screenName.length > 0) {
          result.cmd = `screen -d -r ${screenName}`;
          result.screen = screenName;
        } else {
          // search screen with node name
          let createdScreenName = "";
          if (this.rosState.ros_version === "1") {
            createdScreenName = nodeName.substring(1).replaceAll("_", "__").replaceAll("/", "_");
          } else if (this.rosState.ros_version === "2") {
            createdScreenName = nodeName.substring(1).replaceAll("/", ".");
          }
          result.cmd = `screen -d -r $(ps aux | grep "/usr/bin/SCREEN" | grep "${createdScreenName}" | awk '{print $2}')`;
          result.screen = createdScreenName;
        }
        break;
      case CmdType.LOG:
        // eslint-disable-next-line no-case-declarations
        const logPaths = await this.getLogPaths([nodeName]);
        if (logPaths.length > 0) {
          // `tail -f ${logPaths[0].screen_log} \r`,
          result.cmd = `${this.settings.get("logCommand")} ${logPaths[0].screen_log}`;
          result.log = logPaths[0].screen_log;
        }
        break;
      case CmdType.ECHO:
        if (this.rosState.ros_version === "1") {
          result.cmd = `rostopic echo ${topicName}`;
        } else if (this.rosState.ros_version === "2") {
          result.cmd = `ros2 topic echo ${topicName}`;
        }
        break;
      case CmdType.TERMINAL:
        result.cmd = cmd ? `${cmd} \r` : ``;
        break;
      default:
        break;
    }
    return Promise.resolve(result);
  };

  public setConnectionState: (state: ConnectionState, details: string) => void = async (
    state: ConnectionState,
    details: string = ""
  ) => {
    // ignore call with no state changes
    if (this.connectionState === state) return;

    // update to new state
    const oldState = this.connectionState;
    this.connectionState = state;
    this.errorDetails = details;
    // send an event to inform all listener about new state
    emitCustomEvent(EVENT_PROVIDER_STATE, new EventProviderState(this, state, oldState, details));
    // get provider details depending on new state
    if (state === ConnectionState.STATES.CONNECTING && oldState === ConnectionState.STATES.STARTING) {
      // TODO: delay if use crossbar-wamp server
      // delay(3000);
    } else if (state === ConnectionState.STATES.SERVER_CONNECTED) {
      this.registerCallbacks()
        .then(() => {
          if (this.isAvailable()) {
            this.setConnectionState(ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED, "");
            return true;
          }
          return false;
        })
        .catch((error) => {
          this.setConnectionState(ConnectionState.STATES.ERRORED, error);
        });
    } else if (state === ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED) {
      // wait until daemon.ready received
      // backup for backward compatibility => call updateDaemonInit
      this.updateDaemonInit();
    } else if (state === ConnectionState.STATES.CONNECTED) {
      // if connected callbackDaemonReady calls updateDaemonInit()
    }
  };

  /**
   * Initializes the ROS provider
   *
   * @return {Promise} True is a connection to a WAMP Router succeeded
   */
  public init: () => Promise<boolean> = () => {
    if (this.connection.connected()) return Promise.resolve(true);

    this.setConnectionState(ConnectionState.STATES.CONNECTING, "");

    const result = this.connection.open();
    return Promise.resolve(result);
  };

  /**
   * Close the connection to the WAMP Router
   */
  public close: () => void = async () => {
    let closeError = "";
    await this.connection.close().catch((error) => {
      console.error(`Provider: error when close:`, error);
      closeError = error;
    });
    this.setConnectionState(
      ConnectionState.STATES.CLOSED,
      closeError ? `closed with error: ${JSON.stringify(closeError)}` : "closed"
    );
  };

  /**
   * Reconnect the connection to the WAMP Router
   */
  public reconnect: () => void = async () => {
    if (this.connection.connected()) return Promise.resolve(true);

    this.setConnectionState(ConnectionState.STATES.CONNECTING, "reconnect");

    return this.connection.open();
  };

  /**
   * Check if the provider is available
   *
   * @return {boolean} true is available
   */
  public isAvailable: () => boolean = () => {
    return this.connection.connected();
  };

  /**
   * Check if the provider is ready
   *
   * @return {boolean} true is available
   */
  public isReady: () => boolean = () => {
    return this.daemon && this.discovery;
  };

  public hasActiveRequests: () => boolean = () => {
    return this.currentRequestList.size > 0;
  };

  /** return cleaned version string or empty string if no version is available. */
  public getDaemonReleaseVersion: () => string = () => {
    return this.daemonVersion.version.split("-")[0].replace("v", "");
  };

  /**
   * Request a list of available providers using uri URI.ROS_PROVIDER_GET_LIST
   *
   * @return {Provider} Empty
   */
  public updateProviderList: () => Promise<boolean> = async () => {
    const rosProviders: RosProviderState[] = await this.makeCall(URI.ROS_PROVIDER_GET_LIST, [], true).then(
      (value: TResultData) => {
        if (value.result) {
          return value.data as RosProviderState[];
        }
        this.logger?.error(`Provider [${this.name()}]: Error at updateProviderList()`, `${value.message}`);
        this.setConnectionState(
          ConnectionState.STATES.ERRORED,
          `Provider [${this.name()}]: updateProviderList() result: ${value.message}`
        );
        return [];
      }
    );
    if (rosProviders.length > 0) {
      this.logger?.debug(`Providers updated for [${this.name()}]`, ``);
      if (this.rosVersion === "1") {
        this.discovery = true;
      }

      // Update list of providers
      let oldRemoteProviders = this.remoteProviders;
      this.remoteProviders = [];
      rosProviders.forEach((p: RosProviderState) => {
        // the remote provider list should contain at least the details to itself (origin === true)
        if (p.origin) {
          // apply remote attributes to new current provider
          this.rosState = p;
          // TODO: visualize warning if hosts are not equal
          if (
            this.connection.host !== "localhost" &&
            this.connection.host !== p.host &&
            !p.hostnames.includes(this.connection.host)
          ) {
            // connected to remote using address which is not in hostnames received from remote provider.
            // we add it to local stats
            // TODO add warning about unknown address to ProviderPanel
            if (p.host) {
              this.hostnames.push(p.host);
            }
          }
          // copy all new addresses to local hostnames
          this.hostnames = [...new Set([...this.hostnames, ...p.hostnames])];
        } else {
          // add provider discovered by the contacted provider
          // add discovered provider
          const np = new Provider(
            this.settings,
            p.host,
            p.ros_version ? p.ros_version : "2",
            p.port,
            false, // TODO get useSSL from settings
            this.logger
          );
          oldRemoteProviders = oldRemoteProviders.filter((orp) => orp.url() !== np.url());
          np.rosState = p;
          np.discovered = [this.id];
          this.remoteProviders.push(np);
        }
      });
      this.remoteProviders.forEach((np) => {
        emitCustomEvent(EVENT_PROVIDER_DISCOVERED, new EventProviderDiscovered(np, this));
      });
      oldRemoteProviders.forEach((p) => {
        emitCustomEvent(EVENT_PROVIDER_REMOVED, new EventProviderRemoved(p, this));
      });
      this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  /**
   * Get content of the file from provider.
   *
   * @return {Promise<{ file: FileItem; error: string }>}
   */
  public getFileContent: (path: string) => Promise<{ file: FileItem; error: string }> = async (path) => {
    const error: string = "";
    const fileItem: FileItem | null = await this.makeCall(URI.ROS_FILE_GET, [path], false).then(
      (value: TResultData) => {
        if (value.result) {
          const result = value.data as FileItem;
          result.host = this.host();
          return result;
        }
        this.logger?.error(`Provider [${this.name()}]: can not get content for ${path}: `, `${value.message}`, false);
        return null;
      }
    );
    if (fileItem) {
      return Promise.resolve({ file: fileItem, error: "" });
    }
    return Promise.resolve({
      file: new FileItem(this.connection.host, path),
      error,
    });
  };

  /**
   * Save content of the file in providers file system.
   *
   * @return {Promise<{ bytesWritten: number; error: string }>}
   */
  public saveFileContent: (file: FileItem) => Promise<{ bytesWritten: number; error: string }> = async (file) => {
    let error: string = "";
    const bitesWritten: number = await this.makeCall(URI.ROS_FILE_SAVE, [file], false).then((value: TResultData) => {
      if (value.result) {
        return value.data as number;
      }
      error = value.message as string;
      this.logger?.error(`Provider [${this.name()}]: can not save content to ${file.path}: `, `${error}`, false);
      return -1;
    });
    if (bitesWritten) {
      return Promise.resolve({
        bytesWritten: bitesWritten,
        error: "",
      });
    }
    return Promise.resolve({
      bytesWritten: -1,
      error,
    });
  };

  /**
   * Get daemon version.
   *
   * @return {Promise<DaemonVersion>}
   */
  public getDaemonVersion: () => Promise<DaemonVersion> = async () => {
    let error: string = "";
    const daemonVersion: DaemonVersion | null = await this.makeCall(URI.ROS_DAEMON_VERSION, [], true).then(
      (value: TResultData) => {
        if (value.result) {
          console.log(`value as DaemonVersion: ${JSON.stringify(value.data as DaemonVersion)}`);
          return value.data as DaemonVersion;
        }
        error = value.message;
        this.logger?.error(`Provider [${this.name()}]: Error at getDaemonVersion()`, `${error}`, false);
        this.daemon = false;
        return null;
      }
    );
    if (daemonVersion) {
      this.daemonVersion = daemonVersion;
      this.daemon = true;
      return Promise.resolve(this.daemonVersion);
    }
    throw Error(`Provider [${this.name()}]: getDaemonVersion() result: ${error}`);
  };

  /**
   * Get system info from provider using the WAMP uri 'ros.provider.get_system_info'
   *
   * @return {Promise<any>}
   */
  public getProviderSystemInfo: () => Promise<object> = async () => {
    const systemInfo = await this.makeCall(URI.ROS_PROVIDER_GET_SYSTEM_INFO, [], true).then((value: TResultData) => {
      if (value.result) {
        this.systemInfo = value.data as object;
        return this.systemInfo;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getProviderSystemInfo()`, `${value.message}`);
      return {};
    });
    return Promise.resolve(systemInfo);
  };

  /**
   * Get system environment from provider.
   *
   * @return {Promise<any>}
   */
  public getProviderSystemEnv: () => Promise<object> = async () => {
    const systemEnv = await this.makeCall(URI.ROS_PROVIDER_GET_SYSTEM_ENV, [], true).then((value: TResultData) => {
      if (value.result) {
        this.systemEnv = value.data as object;
        return this.systemEnv;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getProviderSystemEnv()`, `${value.message}`);
      return {};
    });
    return Promise.resolve(systemEnv);
  };

  private calcTimeDiff(startTs: number, endTs: number, remoteTs: number) {
    // try to remove JavaScript andNetwork delay
    const diffStartEnd = endTs - startTs;
    const diffToStart = remoteTs - startTs;
    const diffToEnd = endTs - remoteTs;
    let result = Math.abs(Math.abs(diffToStart) + Math.abs(diffToEnd) - diffStartEnd) / 2.0;
    if (diffToStart < 0) {
      result *= -1.0;
    }
    return result;
  }

  /**
   * Updates the time difference in milliseconds to the provider using the WAMP uri URI.ROS_PROVIDER_GET_TIMESTAMP
   * Emit event on successful update.
   */
  public updateTimeDiff: () => Promise<boolean> = async () => {
    const startTs = Date.now();
    const providerResponse: TProviderTimestamp = await this.makeCall(
      URI.ROS_PROVIDER_GET_TIMESTAMP,
      [startTs],
      true
    ).then((value: TResultData) => {
      if (value.result) {
        return value.data as TProviderTimestamp;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at updateTimeDiff()`, `${value.message}`, false);
      return {
        timestamp: 0,
        diff: 0,
      };
    });
    if (providerResponse.timestamp > 0) {
      const endTs = Date.now();
      this.timeDiff = this.calcTimeDiff(startTs, endTs, providerResponse.timestamp);
      this.logger?.debug(
        `Time difference to [${this.name()}]: approx. ${this.timeDiff}, returned from daemon: ${providerResponse.diff}`,
        ``
      );
      emitCustomEvent(EVENT_PROVIDER_TIME_DIFF, new EventProviderTimeDiff(this, this.timeDiff));
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  /**
   * Get list of available nodes using the uri URI.ROS_NODES_GET_LIST
   *
   * @return {Promise<RosNode[]>} Returns a list of ROS nodes
   */
  public getNodeList: () => Promise<RosNode[]> = async () => {
    interface IRosNode {
      id: string;
      name: string;
      status: string;
      namespace: string;
      node_API_URI: string;
      pid: number;
      masteruri: string;
      location: string;
      publishers: RosTopic[];
      subscribers: RosTopic[];
      services: RosService[];
      screens: string[];
      system_node: boolean;
      parent_id: string | null;
    }
    const rawNodeList = await this.makeCall(URI.ROS_NODES_GET_LIST, [], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as unknown as IRosNode[];
      }
      if (`${value.message}`.includes("wamp.error.no_such_procedure")) {
        this.discovery = false;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getNodeList()`, `${value.message}`);
      return [];
    });

    if (rawNodeList) {
      // cast incoming object into a proper representation
      const nodeList = new Map<string, RosNode>();
      try {
        if (rawNodeList) {
          rawNodeList.forEach((n: IRosNode) => {
            let status = RosNodeStatus.UNKNOWN;
            if (n.status && n.status === "running") status = RosNodeStatus.RUNNING;
            else if (n.status && n.status === "inactive") status = RosNodeStatus.INACTIVE;
            else if (n.status && n.status === "not_monitored") status = RosNodeStatus.NOT_MONITORED;
            else if (n.status && n.status === "dead") status = RosNodeStatus.DEAD;
            else status = RosNodeStatus.UNKNOWN;

            const rn = new RosNode(
              n.id,
              n.name,
              n.namespace,
              n.node_API_URI,
              status,
              n.pid,
              n.masteruri,
              n.location,
              n.system_node
            );

            if (n.parent_id) {
              rn.parent_id = n.parent_id;
            }

            // Add Array elements
            n.publishers.forEach((s: RosTopic) => {
              rn.publishers.set(s.name, new RosTopic(s.name, s.msgtype, s.publisher, s.subscriber));
            });

            n.subscribers.forEach((s: RosTopic) => {
              rn.subscribers.set(s.name, new RosTopic(s.name, s.msgtype, s.publisher, s.subscriber));
            });

            n.services.forEach((s: RosService) => {
              rn.services.set(
                s.name,
                new RosService(s.name, s.srvtype, s.masteruri, s.service_API_URI, s.provider, s.location, s.requester)
              );
            });

            // add screens
            // TODO: Filter screens that belongs to the same master URI
            rn.screens = n.screens;
            nodeList.set(n.id, rn);
          });

          this.logger?.debug(`Nodes updated for [${this.name()}]`, ``);
          if (this.rosVersion === "1") {
            this.discovery = rawNodeList.length > 0;
          }
          return Array.from(nodeList.values());
        }
      } catch (error) {
        this.logger?.error(`Provider [${this.name()}]: Error at getNodeList()`, `${error}`);
        return Promise.resolve([]);
      }
    }

    return Promise.resolve([]);
  };

  /**
   * Return the URI.ROS_SYSTEM_GET_URI
   */
  public getSystemUri: () => Promise<string> = async () => {
    const result = await this.makeCall(URI.ROS_SYSTEM_GET_URI, [], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as string;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getSystemUri()`, `${value.message}`);
      return "";
    });
    return result;
  };

  /* File manager */

  /**
   * Get list of available packages using the uri 'ros.packages.get_list'
   *
   * @return {Promise<RosPackage[]>} Returns a list of ROS packages
   */
  public getPackageList: () => Promise<RosPackage[]> = async () => {
    const comparePackages = (a: RosPackage, b: RosPackage) => {
      if (a.path < b.path) {
        return -1;
      }
      if (a.path > b.path) {
        return 1;
      }
      return 0;
    };
    this.packages = [];
    const result = await this.makeCall("ros.packages.get_list", [], true).then((value: TResultData) => {
      if (value.result) {
        const rosPackageList: RosPackage[] = value.data as RosPackage[];
        const packageList: RosPackage[] = [];
        rosPackageList?.forEach((p: RosPackage) => {
          packageList.push(new RosPackage(p.name, p.path));
        });
        return packageList.sort(comparePackages);
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getPackageList()`, `${value.message}`);
      return [];
    });
    this.packages = result;
    return Promise.resolve(result);
  };

  /** Tries to determine the package name for given path */
  public getPackageName: (path: string) => string = (path) => {
    if (this.packages) {
      const packages = this.packages.filter((rosPackage) => {
        return path.startsWith(rosPackage.path.endsWith("/") ? rosPackage.path : `${rosPackage.path}/`);
      });
      return packages.length > 0 ? `${packages[0]?.name}` : "";
    }
    return "";
  };

  /**
   * Get list of files available for a given path
   *
   * @param {string} path - Folder path to retrieve content from.
   * @return {Promise<PathItem[]>} Returns a list of [PathItem] elements
   */
  public getPathList: (path: string) => Promise<PathItem[]> = async (path) => {
    const result = await this.makeCall(URI.ROS_PATH_GET_LIST_RECURSIVE, [path], true).then((value: TResultData) => {
      if (value.result) {
        const fileList: PathItem[] = [];
        const uniquePaths: string[] = [];
        (value.data as PathItem[]).forEach((p: PathItem) => {
          if (!uniquePaths.includes(p.path)) {
            fileList.push(new PathItem(p.path, p.mtime, p.size, p.type, this.connection.host));
            uniquePaths.push(p.path);
          }
        });
        return fileList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getPackageList()`, `${value.message}`);
      return [];
    });
    return Promise.resolve(result);
  };

  /**
   * Get the path of log files for all [nodes]
   *
   * @param {string[]} nodes - List of node names to get log files
   * @return {Promise<LogPathItem>} Returns a list of [PathItem] elements
   */
  public getLogPaths: (nodes: string[]) => Promise<LogPathItem[]> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PATH_GET_LOG_PATHS, [nodes], true).then((value: TResultData) => {
      if (value.result) {
        const logPathList: LogPathItem[] = [];
        (value.data as LogPathItem[]).forEach((p: LogPathItem) => {
          logPathList.push(new LogPathItem(p.node, p.screen_log, p.screen_log_exists, p.ros_log, p.ros_log_exists));
        });
        return logPathList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getLogPaths()`, `${value.message}`);
      return [];
    });

    return Promise.resolve(result);
  };

  /**
   * Clear the path of log files for given nodes
   *
   * @param {string[]} nodes - List of node names to clear log files
   * @return {Promise<TResultClearPath[]>} Returns a list of [PathItem] elements
   */
  public clearLogPaths: (nodes: string[]) => Promise<TResultClearPath[]> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PATH_CLEAR_LOG_PATHS, [nodes], true).then((value: TResultData) => {
      if (value.result) {
        const logPathList: TResultClearPath[] = [];
        (value.data as TResultClearPath[]).forEach((p: TResultClearPath) => {
          logPathList.push({
            node: p.node,
            result: p.result,
            message: p.message,
          });
        });
        return logPathList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at clearLogPaths()`, `${value.message}`);
      return [];
    });
    return Promise.resolve(result);
  };

  /**
   * Clear the path of log files for given nodes
   *
   * @return {Promise<Result>} Returns a list of [PathItem] elements
   */
  public rosCleanPurge: () => Promise<Result> = async () => {
    const result = await this.makeCall(URI.ROS_PROVIDER_ROS_CLEAN_PURGE, [], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as Result;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at rosCleanPurge()`, `${value.message}`);
      return {
        result: false,
        message: `Provider [${this.name()}]: Error at rosCleanPurge(): ${value.message}`,
      };
    });
    return Promise.resolve(result);
  };

  /**
   * Terminate all running subprocesses (ROS, TTYD) of the provider
   *
   * @return {Promise<Result>}
   */
  public shutdown: () => Promise<Result> = async () => {
    const result = await this.makeCall(URI.ROS_PROVIDER_SHUTDOWN, [], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as Result;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at shutdown()`, `${value.message}`);
      return {
        result: false,
        message: `Provider [${this.name()}]: Error at shutdown(): ${value.message}`,
      };
    });
    return Promise.resolve(result);
  };

  /* Launch servicer */

  /**
   * Load a new launch file into launch servicer
   *
   * @param {LaunchLoadRequest} request - Launch request
   * @return {Promise<LaunchLoadReply>} Returns a LaunchLoadReply
   */
  public launchLoadFile: (request: LaunchLoadRequest, reload: boolean) => Promise<LaunchLoadReply | null> = async (
    request,
    reload
  ) => {
    let result: LaunchLoadReply | null = null;

    if (reload)
      result = await this.makeCall(URI.ROS_LAUNCH_RELOAD, [request], true).then((value: TResultData) => {
        if (value.result) {
          const parsed = value.data as LaunchLoadReply;
          const loadReply = new LaunchLoadReply(parsed.status, parsed.paths, parsed.args, parsed.changed_nodes);
          return loadReply;
        }
        this.logger?.error(`Provider [${this.name()}]: Error at launchLoadFile()`, `${value.message}`);
        return null;
      });
    else {
      result = await this.makeCall(URI.ROS_LAUNCH_LOAD, [request], true).then((value: TResultData) => {
        if (value.result) {
          const parsed = value.data as LaunchLoadReply;
          const loadReply = new LaunchLoadReply(parsed.status, parsed.paths, parsed.args, parsed.changed_nodes);
          return loadReply;
        }
        this.logger?.error(`Provider [${this.name()}]: Error at launchLoadFile()`, `${value.message}`);
        return null;
      });
    }
    return Promise.resolve(result);
  };

  /**
   * Unload a launch file from the launch servicer
   *
   * @param {LaunchFile} request - Launch file to be removed
   * @return {Promise<LaunchLoadReply>} Returns a LaunchLoadReply
   */
  public launchUnloadFile: (request: LaunchFile) => Promise<LaunchLoadReply | null> = async (request) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_UNLOAD, [request], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as LaunchLoadReply;
        const loadReply = new LaunchLoadReply(parsed.status, parsed.paths, parsed.args, parsed.changed_nodes);
        return loadReply;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at launchUnloadFile()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Updates the screens. This is called on message received by subscribed topic and also by request.
   * On request the msgs list should be null. In this case the list is requested by this method.
   */
  public updateScreens: (msgs: ScreensMapping[] | null) => Promise<boolean> = async (msgs = null) => {
    let screens = msgs;
    if (screens === null) {
      screens = await this.getScreenList();
      if (screens === null) {
        return Promise.resolve(false);
      }
    }
    this.screens = screens;
    // update the screens
    this.screens.forEach((s) => {
      const matchingNode = this.rosNodes.find((node) => node.id === s.name);
      if (matchingNode) {
        // update = true;
        matchingNode.screens = s.screens;
      }
    });
    emitCustomEvent(EVENT_PROVIDER_SCREENS, new EventProviderScreens(this, screens));
    return Promise.resolve(true);
  };

  /**
   * Updates the diagnostics. This is called on message received by subscribed topic and also by request.
   * On request the msgs list should be null. In this case the list is requested by this method.
   */
  public updateDiagnostics: (msg: DiagnosticArray | null) => Promise<boolean> = async (msg = null) => {
    let diags = msg;
    if (diags === null) {
      diags = await this.getDiagnostics();
      if (diags === null) {
        return Promise.resolve(false);
      }
    }
    // update the screens
    diags.status.forEach((status) => {
      const matchingNode = this.rosNodes.find((node) => node.id === status.name);
      if (matchingNode) {
        matchingNode.diagnosticLevel = status.level;
        matchingNode.diagnosticMessage = status.message;
        if (matchingNode.diagnosticStatus.length === 0) {
          matchingNode.diagnosticStatus.push(status);
        } else {
          // do not add the same value
          const lastStatus = matchingNode.diagnosticStatus[matchingNode.diagnosticStatus.length - 1];
          if (
            lastStatus.level !== status.level ||
            lastStatus.message !== status.message ||
            lastStatus.values.length !== status.values.length
          ) {
            matchingNode.diagnosticStatus.push(status);
          }
        }
      }
    });
    // emitCustomEvent(
    //   EVENT_PROVIDER_DIAGNOSTICS,
    //   new EventProviderDiagnostics(this, diags),
    // );
    return Promise.resolve(true);
  };

  public toNamespace: (name: string) => { namespace: string; level: number } = (name) => {
    const rest = name.split("/").slice(0, -1);
    return { namespace: rest.join("/"), level: rest.length - 1 };
  };

  /**
   * Get the list of nodes loaded in launch files. update launch files into provider object
   *
   * @return {Promise<LaunchContent>} Returns a list of LaunchContent elements
   */
  public updateLaunchContent: () => Promise<boolean> = async () => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_LIST, [], true).then((value: TResultData) => {
      if (value.result) {
        const parsedList = value.data as LaunchContent[];
        const launchList: LaunchContent[] = [];

        if (parsedList) {
          parsedList.forEach((parsed: LaunchContent) => {
            // filter only the launch files associated to the provider
            if (this.rosState.masteruri && parsed.masteruri !== this.rosState.masteruri) {
              return;
            }
            launchList.push(
              new LaunchContent(
                parsed.path,
                parsed.args,
                parsed.masteruri,
                parsed.host,
                parsed.nodes,
                parsed.parameters.map((p) => new RosParameter(p.name, p.value, p.type, this.id)),
                parsed.associations
              )
            );
          });
        }
        this.launchFiles = launchList;
        emitCustomEvent(EVENT_PROVIDER_LAUNCH_LIST, new EventProviderLaunchList(this, this.launchFiles));

        const capabilityGroupParamName = `/${this.settings.get("capabilityGroupParameter")}`;
        // update nodes
        // Add nodes from launch files to the list of nodes
        this.launchFiles.forEach((launchFile) => {
          launchFile.nodes.forEach((launchNode: LaunchNodeInfo) => {
            // get parameter of a node and determine the capability group parameter
            const nodeParameters: RosParameter[] = [];
            let nodeGroup: { namespace?: string; name?: string } = {};
            let groupParameterFound = false;
            let nodesParametersFound = false;
            const uniqueNodeName = launchNode.unique_name ? launchNode.unique_name : "";
            if (uniqueNodeName) {
              const capabilityGroupOfNode = `${uniqueNodeName}${capabilityGroupParamName}`;
              // update parameters
              launchFile.parameters.forEach((p: RosParameter) => {
                if (nodesParametersFound) {
                  // skip parse further parameter if we found one and next was not in node namespace
                  // assumption: parameters are sorted
                  return;
                } else if (p.name.startsWith(uniqueNodeName)) {
                  nodeParameters.push(p);
                  if (p.name === capabilityGroupOfNode) {
                    // update capability group
                    groupParameterFound = true;
                    const ns = launchNode.node_namespace ? launchNode.node_namespace : "";
                    nodeGroup = { namespace: ns, name: `{${p.value}}` };
                  }
                } else if (nodeParameters.length > 0) {
                  // we found one parameter of the node, but current parameter is not in node namespace => skip all further parameter
                  // assumption: parameters are sorted
                  nodesParametersFound = true;
                } else if (!groupParameterFound && p.name.endsWith(capabilityGroupParamName)) {
                  // use capability group parameter in the higher level namespace until we found one in the node namespace
                  const { namespace } = this.toNamespace(p.name);
                  if (uniqueNodeName.startsWith(namespace)) {
                    nodeGroup = { namespace: namespace, name: `{${p.value}}` };
                  }
                }
              });

              let associations: string[] = [];
              launchFile.associations.forEach((item) => {
                if (item.node === uniqueNodeName) {
                  associations = item.nodes;
                }
              });

              // if node exist (it is running), only update the associated launch file
              let nodeIsRunning = false;
              const iNode = this.rosNodes.findIndex((n) => {
                return n.name === uniqueNodeName;
              });
              if (iNode >= 0) {
                this.rosNodes[iNode].launchPaths.add(launchFile.path);
                this.rosNodes[iNode].parameters.set(launchFile.path, nodeParameters);
                this.rosNodes[iNode].launchInfo = launchNode;
                this.rosNodes[iNode].associations = associations;
                if (nodeGroup.name) {
                  this.rosNodes[iNode].capabilityGroup = nodeGroup;
                }
                nodeIsRunning = true;
              }

              if (!nodeIsRunning) {
                // if node is not running add it as inactive node
                const n = new RosNode(
                  uniqueNodeName,
                  uniqueNodeName,
                  launchNode.node_namespace ? launchNode.node_namespace : "",
                  "",
                  RosNodeStatus.INACTIVE
                );
                n.launchPaths.add(launchFile.path);
                n.parameters.set(launchFile.path, nodeParameters);
                n.launchInfo = launchNode;
                // idGlobal should be the same for life of the node on remote host
                n.idGlobal = `${this.id}${n.id.replaceAll("/", ".")}`;
                n.providerName = this.name();
                n.providerId = this.id;
                n.associations = associations;
                if (nodeGroup.name) {
                  n.capabilityGroup = nodeGroup;
                }
                this.rosNodes.push(n);
              }
            }
          });
        });

        // set tags for nodelets/composable and other tags)
        const composableManagers: string[] = [];
        this.rosNodes.forEach((n) => {
          // Check if this is a nodelet/composable and assign tags accordingly.
          let composableParent = n?.parent_id || n.launchInfo?.composable_container;
          if (composableParent) {
            composableParent = composableParent.split("|").slice(-1).at(0);
            if (composableParent) {
              if (!composableManagers.includes(composableParent)) {
                composableManagers.push(composableParent);
              }
              n.tags = [
                {
                  text: "Nodelet",
                  color: TagColors[composableManagers.indexOf(composableParent) % TagColors.length],
                },
              ];
            }
          }
        });
        // Assign tags to the found nodelet/composable managers.
        composableManagers.forEach((managerId) => {
          const node = this.rosNodes.find((n) => n.id === managerId || n.name === managerId);
          if (node) {
            node.tags = [
              {
                text: "Manager",
                color: TagColors[composableManagers.indexOf(node.id) % TagColors.length],
              },
            ];
          }
        });

        this.daemon = true;
        emitCustomEvent(EVENT_PROVIDER_ROS_NODES, new EventProviderRosNodes(this, this.rosNodes));
        return true;
      }

      if (`${value.message}`.includes("wamp.error.no_such_procedure")) {
        this.daemon = false;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at updateLaunchContent()`, `${value.message}`);
      this.launchFiles = [];
      return false;
    });
    return Promise.resolve(result);
  };

  /**
   * Returns all included files in given launch file.
   *
   * @param {LaunchIncludedFilesRequest} request - Launch file to be requested
   * @return {Promise<LogPathItem>} Returns a list of [PathItem] elements
   */
  public launchGetIncludedFiles: (request: LaunchIncludedFilesRequest) => Promise<LaunchIncludedFile[] | null> = async (
    request
  ) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_INCLUDED_FILES, [request], true).then(
      (value: TResultData) => {
        if (value.result) {
          const parsedList = value.data as LaunchIncludedFile[];
          const launchList: LaunchIncludedFile[] = [];

          if (parsedList) {
            parsedList.forEach(
              (lf: {
                path: string;
                line_number: number;
                inc_path: string;
                exists: boolean;
                raw_inc_path: string;
                rec_depth: number;
                args: LaunchArgument[];
                default_inc_args: LaunchArgument[];
                size: number;
              }) => {
                launchList.push(
                  new LaunchIncludedFile(
                    this.connection.host,
                    lf.path,
                    lf.line_number,
                    lf.inc_path,
                    lf.exists,
                    lf.raw_inc_path,
                    lf.rec_depth,
                    lf.args,
                    lf.default_inc_args,
                    lf.size
                  )
                );
              }
            );
          }
          return launchList;
        }

        this.logger?.error(`Provider [${this.name()}]: Error at launchGetIncludedFiles()`, `${value.message}`);
        return null;
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Returns a messages struct for given message type.
   *
   * @param {string} request - Topic type to be requested
   * @return {Promise<LaunchMessageStruct>} Returns a message struct
   */
  public getMessageStruct: (request: string) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_MSG_STRUCT, [request], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as LaunchMessageStruct;
        const response = new LaunchMessageStruct(parsed.msg_type, parsed.data, parsed.valid, parsed.error_msg);
        if (response.valid) {
          return response;
        }
        this.logger?.error(`Can't parse message: ${request}`, parsed.error_msg);
        return null;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getMessageStruct()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Returns a messages struct for given service type.
   *
   * @param {string} request - Service type to be requested
   * @return {Promise<LaunchMessageStruct>} Returns a message struct
   */
  public getServiceStruct: (request: string) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_SRV_STRUCT, [request], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as LaunchMessageStruct;
        const response = new LaunchMessageStruct(parsed.msg_type, parsed.data, parsed.valid, parsed.error_msg);
        if (response.valid) {
          return response;
        }
        this.logger?.error(`Can't parse service: ${request}`, parsed.error_msg);
        return null;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getServiceStruct()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Start Publisher
   *
   * @param {LaunchPublishMessage} request - Launch request
   * @return {Promise<Boolean>} Returns true if success
   */
  public publishMessage: (request: LaunchPublishMessage) => Promise<boolean> = async (request) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_PUBLISH_MESSAGE, [request], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as boolean;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at publishMessage()`, `${value.message}`);
      return false;
    });
    return result;
  };

  /**
   * Call Service
   *
   * @param {LaunchCallService} request - Launch request
   * @return {Promise<LaunchMessageStruct | null>} Returns true if success
   */
  public callService: (request: LaunchCallService) => Promise<LaunchMessageStruct | null> = async (request) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_CALL_SERVICE, [request], true).then((value: TResultData) => {
      if (value.result) {
        if (!value.data) {
          return null;
        }
        const parsed = value.data as LaunchMessageStruct;
        const response = new LaunchMessageStruct(parsed.msg_type, parsed.data, parsed.valid, parsed.error_msg);
        return response;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at callService()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  private generateSubscriberUri: (topic: string) => string = (topic) => {
    return `${URI.ROS_SUBSCRIBER_EVENT_PREFIX}.${topic.replaceAll("/", "_")}`;
  };

  /**
   * Initializes new providers using message callback from provider server
   * First closes all existing providers and then create new ones.
   */
  private callbackNewSubscribedMessage: (msg: JSONObject) => void = (msg) => {
    this.logger?.debugInterface(URI.ROS_SUBSCRIBER_EVENT_PREFIX, msg, "", this.id);

    // ignore empty and duplicated messages
    if (msg.length === 0) {
      return;
    }

    try {
      const msgParsed: SubscriberEvent = msg as unknown as SubscriberEvent;
      emitCustomEvent(
        `${EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX}_${msgParsed.topic}`,
        new EventProviderSubscriberEvent(this, msgParsed)
      );
    } catch (error) {
      this.logger?.error(`[updateSubscribedMessage] Could not parse message ${msg}`, `Error: ${error}`);
    }
  };

  /**
   * Start subscriber node for given topic
   *
   * @param {SubscriberNode} subscriber - Launch request
   * @return {Promise<boolean>} Returns true if success
   */
  public startSubscriber: (request: SubscriberNode) => Promise<boolean> = async (request) => {
    const hasEcho = this.echoTopics.includes(request.topic);
    if (hasEcho) {
      return Promise.resolve(true);
    }
    this.echoTopics.push(request.topic);
    const result = await this.makeCall(URI.ROS_SUBSCRIBER_START, [request], true).then((value: TResultData) => {
      if (value.result) {
        const parsed: boolean = value.data as boolean;
        return parsed;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at startSubscriber()`, `${value.message}`);
      return false;
    });
    if (result) {
      const cbTopic = this.generateSubscriberUri(request.topic);
      this.registerCallback(cbTopic, this.callbackNewSubscribedMessage);
      this.logger?.debug(
        `Started subscriber node for '${request.topic} [${request.message_type}]' on '${this.name()}'`,
        ""
      );
    }
    return Promise.resolve(result);
  };

  /**
   * Stop subscriber node for given topic
   *
   * @param {string} topic - topic name
   * @return {Promise<Result>} Returns true if success
   */
  public stopSubscriber: (topic: string) => Promise<Result> = async (topic) => {
    const hasTopic = this.echoTopics.includes(topic);
    if (hasTopic) {
      this.echoTopics = this.echoTopics.filter((e) => e !== topic);
      const cbTopic = this.generateSubscriberUri(topic);
      await this.connection.closeSubscription(cbTopic).catch((err) => {
        this.logger?.error(`Provider [${this.name()}]: close subscription failed`, `${err}`);
      });
    }
    const result = await this.makeCall(URI.ROS_SUBSCRIBER_STOP, [topic], true).then((value: TResultData) => {
      if (value.result) {
        const res = value.data as Result;
        return res;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at stopSubscriber()`, `${value.message}`);
      return new Result(false, value.message as string);
    });
    return Promise.resolve(result);
  };

  /**
   * Update the filter for given topic
   */
  public updateFilterRosTopic: (topicName: string, msg: SubscriberFilter) => Promise<Result> = async (
    topicName,
    msg
  ) => {
    const cbTopic = `${URI.ROS_SUBSCRIBER_FILTER_PREFIX}.${topicName.replaceAll("/", "_")}`;
    this.logger?.debug(`Provider: (${this.name()}) Publish to: [${cbTopic}]`, "");
    const result = await this.connection.publish(cbTopic, JSON.parse(JSON.stringify(msg)));
    return result;
  };

  /**
   * Start a ROS node using a given provider and node object
   */
  public startNode: (node: RosNode) => Promise<TResultStartNode> = async (node: RosNode) => {
    if (node.providerId !== this.id) {
      return {
        success: false,
        message: `Inconsistent provider (${node.providerName} vs. ${this.name()})`,
        details: DEFAULT_BUG_TEXT,
        response: null,
      };
    }

    // TODO: Add log level and format?
    const request = new LaunchNode(
      node.name, // name
      "", // opt_binary
      `${node.launchPath}`, // opt_launch
      "", // log level
      "", // log format
      node.masteruri?.length > 0 ? node.masteruri : `${this.rosState.masteruri}`, // masteruri
      true, // reload global parameters
      "" // cmd
    );

    const result = await this.makeCall(URI.ROS_LAUNCH_START_NODE, [request], true).then((value: TResultData) => {
      emitCustomEvent(EVENT_PROVIDER_NODE_STARTED, new EventProviderNodeStarted(this, node));
      if (value.result) {
        const parsed = value.data as LaunchNodeReply;
        const response = new LaunchNodeReply(parsed.name, parsed.status, parsed.paths, parsed.launch_files);

        if (!response) {
          return {
            success: false,
            message: `Invalid return. Node: ${node.id}`,
            details: DEFAULT_BUG_TEXT,
            response: null,
          };
        }

        if (response.status.code === "OK") {
          return {
            success: true,
            message: `Node started: [${node.id}]`,
            details: "",
            response,
          };
        }

        if (response.status.code === "NODE_NOT_FOUND") {
          return {
            success: false,
            message: `Could not start node [${node.name}]: Node not found`,
            details: `Node: ${node.id} Details: ${response.status.msg}`,
            response,
          };
        }

        if (response.status.code === "MULTIPLE_LAUNCHES") {
          return {
            success: false,
            message: `Could not start node [${node.name}]: Multiple launches found`,
            details: `Node: ${node.id} Details: ${response.status.msg}`,
            response,
          };
        }

        if (response.status.code === "MULTIPLE_BINARIES") {
          return {
            success: false,
            message: `Could not start node [${node.name}]: Multiple binaries found`,
            details: `Node: ${node.id} Details: ${response.status.msg}`,
            response,
          };
        }

        if (response.status.code === "CONNECTION_ERROR") {
          return {
            success: false,
            message: `Could not start node [${node.name}]: Connection error`,
            details: `Node: ${node.id} Details: ${response.status.msg}`,
            response,
          };
        }

        if (response.status.code === "ERROR") {
          return {
            success: false,
            message: `Could not start node: ${node.name}`,
            details: `Node: ${node.id} Details: ${response.status.msg}`,
            response,
          };
        }
        return {
          success: false,
          message: `Start Node: Invalid response`,
          details: `Node: ${node.id} Details: ${response.status.msg}`,
          response,
        };
      }
      return {
        success: false,
        message: "Invalid message from [ros.launch.start_node]",
        details: value.message as string,
        response: null,
      };
    });
    return Promise.resolve(result);
  };

  /**
   * Stop a node given a name
   *
   * @param {string} id - Unique ID of the node to be stopped
   * @return {Promise<Result>} Returns a result
   */
  public stopNode: (id: string) => Promise<Result> = async (id) => {
    const result = await this.makeCall(URI.ROS_NODES_STOP_NODE, [id], false).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as Result;
        if (parsed.result) {
          return parsed;
        }
        // use debug because of spamming messages when nodes does not run
        this.logger?.debug(`${parsed.message}`, parsed.message);
        return parsed;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at stopNode()`, `${value.message}`);
      return new Result(false, `${value.message}`);
    });

    return Promise.resolve(result);
  };

  /**
   * Kill a node given a name
   *
   * @param {string} name - Node to be killed
   * @return {Promise<Result>} Returns a result
   */
  public screenKillNode: (name: string) => Promise<Result> = async (name) => {
    const result = await this.makeCall(URI.ROS_SCREEN_KILL_NODE, [name], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as Result;
        if (!parsed.result) {
          this.logger?.error(`Fail to kill the node [${name}]: ${parsed.message}`, parsed.message, false);
        }
        return parsed;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at screenKillNode()`, `${value.message}`);
      return new Result(false, `Provider [${this.name()}]: Error at screenKillNode(): ${value.message}`);
    });
    return Promise.resolve(result);
  };

  /**
   * Get list of available screens 'ros.screen.get_list'
   *
   * @return {Promise<ScreensMapping[]>} Returns a list of screens
   */
  private getScreenList: () => Promise<ScreensMapping[] | null> = async () => {
    const result = await this.makeCall(URI.ROS_SCREEN_GET_LIST, [], true).then((value: TResultData) => {
      if (value.result) {
        const screenMappings = value.data as ScreensMapping[];
        const screenList: ScreensMapping[] = [];
        screenMappings?.forEach((p: ScreensMapping) => {
          screenList.push(new ScreensMapping(p.name, p.screens));
        });
        return screenList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getScreenList()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Get list of diagnostics
   *
   * @return {Promise<DiagnosticArray>}
   */
  private getDiagnostics: () => Promise<DiagnosticArray | null> = async () => {
    const result = await this.makeCall(URI.ROS_PROVIDER_GET_DIAGNOSTICS, [], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as DiagnosticArray;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getDiagnostics()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Get list of warnings
   *
   * @return {Promise<SystemWarningGroup[]>}
   */
  private updateSystemWarnings: () => Promise<SystemWarningGroup[] | null> = async () => {
    const result = await this.makeCall(URI.ROS_PROVIDER_GET_WARNINGS, [], true).then((value: TResultData) => {
      if (value.result) {
        const warnings = value.data as SystemWarningGroup[];
        this.warnings = warnings;
        return warnings;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at updateSystemWarnings()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Return a list with loggers for given node
   *
   * @param {string} node - Node name
   * @return {Promise<LoggerConfig[]>} Returns a result
   */
  public getNodeLoggers: (node: string) => Promise<LoggerConfig[]> = async (node) => {
    const result = await this.makeCall(URI.ROS_NODES_GET_LOGGERS, [node], true).then((value: TResultData) => {
      if (value.result) {
        // handle the result of type: {result: bool, message: str}
        if (!Array.isArray(value.data) && !value.result) {
          this.logger?.error(`Provider [${this.name()}]: Error at getNodeLoggers(): ${value.message}`, ``);
          return [];
        }
        return value.data as LoggerConfig[];
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at getNodeLoggers()`, `${value.message}`);
      return [];
    });
    return Promise.resolve(result);
  };

  /**
   * Set new logger levels for a ros node
   *
   * @param {string} node - Node name
   * @param {LoggerConfig[]} loggers - Node name
   * @return {Promise<Result>} Returns a result
   */
  public setNodeLoggers: (node: string, loggers: LoggerConfig[]) => Promise<Result> = async (node, loggers) => {
    const result = await this.makeCall(URI.ROS_NODES_SET_LOGGER_LEVEL, [node, loggers], true).then(
      (value: TResultData) => {
        if (value.result) {
          return value.data as Result;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at getNodeLoggers()`, `${value.message}`);
        return new Result(false, `Provider [${this.name()}]: Error at getNodeLoggers(): ${value.message}`);
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Unregister a node given a name
   *
   * @param {string} name - Node to be stopped
   * @return {Promise<Result>} Returns a result
   */
  public unregisterNode: (name: string) => Promise<Result> = async (name) => {
    const result = await this.makeCall(URI.ROS_NODES_UNREGISTER, [name], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as Result;
        if (parsed.result) {
          return parsed;
        }
        // use debug because of spamming messages when nodes does not run
        this.logger?.debug(`${parsed.message}`, parsed.message);
        return parsed;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at unregisterNode()`, `${value.message}`);
      return new Result(false, `Provider [${this.name()}]: Error at unregisterNode(): ${value.message}`);
    });
    return Promise.resolve(result);
  };

  /* Parameters */

  /**
   * Return a list with all registered parameters values and types
   *
   * @return {Promise<Result>} Returns a result
   */
  public getParameterList: () => Promise<RosParameter[]> = async () => {
    const result = await this.makeCall("ros.parameters.get_list", [], true).then((value: TResultData) => {
      if (value.result) {
        const paramList: RosParameter[] = [];
        const parsed = value.data as RosParameter[];
        parsed.forEach((item: RosParameter) => {
          paramList.push(new RosParameter(item.name, item.value, item.type, this.id));
        });
        return paramList;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at getParameterList()`, `${value.message}`);
      return [];
    });
    return Promise.resolve(result);
  };

  /**
   * Return a list with all registered parameters values and types for a list of nodes
   *
   * @param {string[]} nodes - List of node names
   * @return {Promise<Result>} Returns a result
   */
  public getNodeParameters: (nodes: string[]) => Promise<RosParameter[]> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PARAMETERS_GET_NODE_PARAMETERS, [nodes], false).then(
      (value: TResultData) => {
        if (value.result) {
          const paramList: RosParameter[] = [];
          const parsed = value.data as RosParameter[];
          parsed.forEach((item: RosParameter) => {
            paramList.push(new RosParameter(item.name, item.value, item.type, this.id));
          });
          return paramList;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at stopNode()`, `${value.message}`);
        return [];
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Set the value of a parameter
   *
   * @param {RosParameter} parameter
   * @return {Promise<Boolean>} Returns true if success
   */
  public setParameter: (parameter: RosParameter) => Promise<boolean> = async (parameter) => {
    const result = await this.makeCall(URI.ROS_PARAMETERS_SET_PARAMETER, [parameter], true).then(
      (value: TResultData) => {
        if (value.result) {
          const parsed = value.data as boolean;
          return parsed;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at setParameter()`, `${value.message}`);
        return false;
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Check if a parameter exists
   *
   * @param {string} parameterName
   * @return {Promise<Boolean>} Returns true if success
   */
  public hasParameter: (parameterName: string) => Promise<boolean> = async (parameterName) => {
    const result = await this.makeCall(URI.ROS_PARAMETERS_HAS_PARAMETER, [parameterName], true).then(
      (value: TResultData) => {
        if (value.result) {
          return value.data as boolean;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at hasParameter()`, `${value.message}`);
        return false;
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Delete a list of parameters
   *
   * @param {string[]} parameters list of parameters to be deleted
   * @return {Promise<Boolean>} Returns true if success
   */
  public deleteParameters: (parameters: string[]) => Promise<boolean> = async (parameters) => {
    const result = await this.makeCall(URI.ROS_PARAMETERS_DELETE_PARAMETERS, [parameters], true).then(
      (value: TResultData) => {
        if (value.result) {
          return value.data as boolean;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at setParameter()`, `${value.message}`);
        return false;
      }
    );
    return Promise.resolve(result);
  };

  /* Publisher handler */

  /**
   * Callback of master daemon ready status (true/false)
   */
  private callbackDaemonReady: (msg: JSONObject) => void = (msg) => {
    // this.logger?.debugInterface(URI.ROS_DAEMON_READY, msg, '', this.id);
    const msgObj = msg as unknown as TProviderDaemonReady;
    if (msgObj.status && !this.daemon) {
      this.updateDaemonInit();
    } else if (msgObj.timestamp) {
      // update diff state
      this.currentDelay = (Date.now() - msgObj.timestamp + this.timeDiff) / 1000.0;
      emitCustomEvent(EVENT_PROVIDER_DELAY, new EventProviderDelay(this, this.currentDelay));
    }
    this.daemon = msgObj.status;
  };

  /**
   * Callback of master discovery ready status (true/false)
   */
  private callbackDiscoveryReady: (msg: JSONObject) => void = (msg) => {
    this.logger?.debugInterface(URI.ROS_DISCOVERY_READY, msg, "", this.id);
    const msgObj = msg as unknown as TProviderDiscoveryReady;
    this.discovery = msgObj.status;
  };

  /**
   * Callback when any launch file or ROS nodes changes  in provider
   */
  public updateRosNodes: (msg: JSONObject) => void = async (msg) => {
    this.logger?.debug(`Trigger update ros nodes for ${this.id}`, "");
    const msgObj = msg as unknown as { path: string; action: string };
    if (msgObj?.path) {
      emitCustomEvent(EVENT_PROVIDER_LAUNCH_LOADED, new EventProviderLaunchLoaded(this, msgObj.path));
    }
    if (await this.lockRequest("updateRosNodes")) {
      return;
    }

    // get nodes from remote provider
    const nlUnfiltered = await this.getNodeList();
    const nl = nlUnfiltered.filter((n) => {
      let ignored = false;

      // exclude nodes belonging to a different provider
      if (!this.showRemoteNodes) {
        if (
          (this.rosState.masteruri && n.masteruri !== this.rosState.masteruri) ||
          (n.location instanceof String && n.location === "remote") ||
          (n.location instanceof Array &&
            !n.location.some((loc) => loc.startsWith("SHM") || loc.startsWith("UDPv4:[127.0.0.1]")))
        ) {
          ignored = true;
        }
      }

      // exclude ignored nodes
      this.IGNORED_NODES.forEach((ignoredNode) => {
        if (n.name.indexOf(ignoredNode) !== -1) ignored = true;
      });
      return !ignored;
    });

    const dynamicReconfigureNodes = new Set<string>();

    // check if nodes are not available or run in other host (not monitoring)
    nl.forEach((n) => {
      // idGlobal should be the same for life of the node on remote host
      n.idGlobal = `${this.id}${n.id.replaceAll("/", ".")}`;
      n.providerName = this.name();
      n.providerId = this.id;

      // copy (selected) old state
      const oldNode = this.rosNodes.find((item) => item.idGlobal === n.idGlobal);
      if (oldNode) {
        n.diagnosticStatus = oldNode.diagnosticStatus;
        n.diagnosticLevel = oldNode.diagnosticLevel;
        n.diagnosticMessage = oldNode.diagnosticMessage;
        n.launchPaths = oldNode.launchPaths;
        n.launchPath = oldNode.launchPath;
        n.group = oldNode.group;
        n.launchInfo = oldNode.launchInfo;
        n.rosLoggers = oldNode.rosLoggers;
        if (oldNode.pid !== n.pid) {
          emitCustomEvent(EVENT_PROVIDER_NODE_STARTED, new EventProviderNodeStarted(this, n));
        }
      } else if (n.system_node) {
        n.group = this.settings.get("namespaceSystemNodes") as string;
      }

      if (!n.node_API_URI || n.node_API_URI.length === 0) return;
      if (!n.masteruri || n.masteruri.length === 0) return;

      // update infos available only for ROS1 nodes
      if (!n.pid || n.pid <= 0) {
        const hostApiUir = n.node_API_URI.split(":")[0];
        const hostMasterUri = n.masteruri.split(":")[0];

        if (hostApiUir === hostMasterUri) {
          // node runs on the same host, but it is not available
          // probably the node is dead
          n.status = RosNodeStatus.DEAD;
        } else {
          // node runs on a different host, we can not monitor its state
          // will be shown as XXXX
          n.status = RosNodeStatus.NOT_MONITORED;
        }
      }
      // check if the node has dynamic reconfigure service
      Array.from(n.services.keys()).forEach((service) => {
        if (service.endsWith("/set_parameters")) {
          const serviceNs = service.slice(0, -15);
          n.dynamicReconfigureServices.push(serviceNs);
          if (serviceNs !== n.name) {
            dynamicReconfigureNodes.add(serviceNs);
          }
        }
      });
    });
    nl.forEach((n) => {
      if (dynamicReconfigureNodes.has(n.name)) {
        n.dynamicReconfigureServices.push(n.name);
      }
    });
    this.rosNodes = nl;
    // emitCustomEvent(
    //   EVENT_PROVIDER_ROS_NODES,
    //   new EventProviderRosNodes(this, nl),
    // );
    this.updateLaunchContent();
    this.unlockRequest("updateRosNodes");
  };

  public callbackChangedFile: (msg: JSONObject) => void = async (msg) => {
    this.logger?.debugInterface(URI.ROS_PATH_CHANGED, msg, "", this.id);
    if (!msg || (await this.lockRequest("callbackChangedFile"))) {
      return;
    }

    emitCustomEvent(EVENT_PROVIDER_PATH_EVENT, new EventProviderPathEvent(this, msg as unknown as PathEvent));
    this.unlockRequest("callbackChangedFile");
  };

  /**
   * Update the screen state of each node reported in the list of ScreensMapping.
   */
  public callbackScreensUpdate: (msg: JSONObject) => void = async (msg) => {
    this.logger?.debugInterface(URI.ROS_SCREEN_LIST, msg, "", this.id);
    if (!msg) {
      return;
    }

    this.updateScreens(msg as unknown as ScreensMapping[]);
  };

  /**
   * Update diagnostics of each node reported in the list of DiagnosticsArray.
   */
  private callbackDiagnosticsUpdate: (msg: JSONObject) => void = async (msg) => {
    this.logger?.debugInterface(URI.ROS_PROVIDER_DIAGNOSTICS, msg, "", this.id);
    if (!msg) {
      return;
    }
    this.updateDiagnostics(msg as unknown as DiagnosticArray);
  };

  /**
   * Update the provider warnings reported in the list of SystemWarningGroup.
   */
  private callbackProviderWarnings: (msg: JSONObject) => void = async (msg) => {
    this.logger?.debugInterface(URI.ROS_PROVIDER_WARNINGS, msg, "", this.id);
    if (!msg) {
      return;
    }

    const msgParsed: SystemWarningGroup[] = msg as unknown as SystemWarningGroup[];
    this.warnings = msgParsed;
    emitCustomEvent(EVENT_PROVIDER_WARNINGS, new EventProviderWarnings(this, msgParsed));
  };

  private registerCallback: (uri: string, callback: (msg: JSONObject) => void) => void = async (uri, callback) => {
    this.logger?.debug(`Provider: (${this.name()}) Subscribing to: [${uri}]`, "", false);
    const result = await this.connection.subscribe(uri, callback);
    if (!result.result) {
      this.logger?.error(`Provider: (${this.name()}) Subscribing to: [${uri}] failed: ${result.message}`, "", false);
    }
  };

  private registerCallbacks: () => Promise<boolean> = async () => {
    if (!this.isAvailable()) {
      return false;
    }
    await this.connection
      .closeSubscriptions()
      .catch((error) => {
        this.setConnectionState(ConnectionState.STATES.ERRORED, error);
        return false;
      })
      .then(() => {
        // await this.connection.connection.closeRegistrations();
        this.logger?.info(`register callbacks for ${this.id}`, "", false);
        this.getCallbacks().forEach((item: TConCallback) => {
          this.logger?.info(`  register callback for ${item.uri}`, "", false);
          this.registerCallback(item.uri, item.callback);
        });
        return true;
      });
    return false;
  };

  /* Generic functions */

  /**
   * Execute a call considering [callAttempts] and [delayCallAttempts]
   *
   * @param {string} uri URI to be called
   * @param {unknown[]} args call arguments
   * @return {Promise<JSONObject>} Return promise of the call
   */
  private makeCall: (uri: string, args: unknown[], lockRequest: boolean) => Promise<TResultData> = async (
    uri,
    args,
    lockRequest = true
  ) => {
    if (!this.isAvailable()) {
      return {
        result: false,
        message: `[${this.name()}]: Ignoring request to: [${uri}], provider not yet connected!`,
        data: null,
      };
    }

    if (lockRequest && (await this.lockRequest(uri))) {
      return {
        result: false,
        message: `[${this.name()}]: Ignoring request to: [${uri}] with ${JSON.stringify(args)}, request already running!`,
        data: null,
      };
    }
    const callRequest: (_uri: string, _args: unknown[], currentAttempt?: number) => Promise<TResultData> = async (
      _uri,
      _args,
      currentAttempt = 0
    ) => {
      try {
        const r = await this.connection.call(_uri, _args).catch((err) => {
          // TODO
          // this.logger?.warn(`failed call ${_uri}@${this.name()}: ${JSON.stringify(err)}`, '', false);
          if (Object.keys(err).includes("result")) {
            err.data = null;
            return err;
          }
          return { result: false, message: err, data: null };
        });
        return r;
      } catch (err) {
        if (`${err}`.includes("{")) {
          const errorObj = JSON.parse(`${err}`);
          if (errorObj.error?.includes("wamp.error.runtime_error")) {
            // do not re-try on runtime errors
            this.unlockRequest(uri);
            return { result: false, message: `[${this.name()}]: request to [${uri}] failed: ${err}`, data: null };
          }
        } else if (err === "Connection is closed") {
          this.unlockRequest(uri);
          return { result: false, message: `[${this.name()}]: request to [${uri}] failed: ${err}`, data: null };
        }
        if (currentAttempt >= this.callAttempts) {
          this.unlockRequest(uri);
          return {
            result: false,
            message: `[URI] (${uri}) [${this.name()}]: Max call attempts (${this.callAttempts}) reached!`,
            data: null,
          };
        }

        // didn't work, wait and repeat
        this.logger?.info(
          `[URI] (${uri}) [${this.name()}]`,
          `Waiting (${this.delayCallAttempts} ms) Attempt: [${currentAttempt}/${this.callAttempts}]`,
          false
        );
        await delay(this.delayCallAttempts);
        return callRequest(_uri, _args, currentAttempt + 1);
      }
    };

    const result = await callRequest(uri, args);
    // const result = await this.connection.call(uri, args);
    this.unlockRequest(uri);
    this.logger?.debugInterface(uri, result, "", this.name());
    return result;
  };

  private onCloseConnection: (state: ConnectionState, details: string) => void = (state, details) => {
    this.setConnectionState(state, details);
    // clear all activities
    this.currentRequestList.clear();
    emitCustomEvent(EVENT_PROVIDER_ACTIVITY, new EventProviderActivity(this, false, ""));
  };

  private onOpenConnection = () => {
    // connected state is set after all remote connections are registered
    this.daemon = false;
    this.discovery = false;
    this.setConnectionState(ConnectionState.STATES.SERVER_CONNECTED, "");
  };

  /*
  Prevents multiple calls to the same [request]
  Return true when [request has been already locked]
  */
  private lockRequest: (request: string) => Promise<boolean> = async (request) => {
    if (this.currentRequestList.has(request)) {
      this.logger?.debug(`[${this.name()}]: Wait 1 sec for release request to: [${request}]`, "", false);
      // TODO: wait until release?
      await delay(1000);
      if (this.currentRequestList.has(request)) {
        this.logger?.debug(`[${this.name()}]: Ignoring request to: [${request}]`, "", false);
        return Promise.resolve(true);
      }
    }
    emitCustomEvent(EVENT_PROVIDER_ACTIVITY, new EventProviderActivity(this, true, request));
    this.currentRequestList.add(request);
    return Promise.resolve(false);
  };

  /*
  Remove lock for a given [request]
  */
  private unlockRequest = (request: string) => {
    this.currentRequestList.delete(request);
    if (this.currentRequestList.size === 0) {
      emitCustomEvent(EVENT_PROVIDER_ACTIVITY, new EventProviderActivity(this, false, ""));
    }
  };
}
