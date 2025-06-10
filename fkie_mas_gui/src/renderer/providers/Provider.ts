import { JSONObject, TResult, TResultData, TResultProcess, TSystemInfo, TTag } from "@/types";
import { TResultParam } from "@/types/TResultParam";
import FingerprintIcon from "@mui/icons-material/Fingerprint";
import { emitCustomEvent } from "react-custom-events";
import { TagColors, colorFromHostname } from "../components/UI/Colors";
import { DEFAULT_BUG_TEXT, ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import {
  DaemonVersion,
  DiagnosticArray,
  FileItem,
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
  RosParameterValue,
  RosService,
  RosTopic,
  RosTopicId,
  ScreensMapping,
  SubscriberEvent,
  SubscriberFilter,
  SubscriberNode,
  SystemWarningGroup,
  URI,
} from "../models";
import { EVENT_FILTER_NODES, eventFilterNodes } from "../pages/NodeManager/layout/events";
import { delay, generateUniqueId } from "../utils";
import CmdTerminal from "./CmdTerminal";
import CmdType from "./CmdType";
import ConnectionState from "./ConnectionState";
import ProviderConnection, { TProviderTimestamp, TResultClearPath, TResultStartNode } from "./ProviderConnection";
import RosProviderState from "./RosProviderState";
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
  EVENT_PROVIDER_ROS_SERVICES,
  EVENT_PROVIDER_ROS_TOPICS,
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
  EventProviderRosServices,
  EventProviderRosTopics,
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

export type TParamListResult = {
  params: RosParameter[];
  errors: string[];
};

export type TConCallback = {
  uri: string;
  callback: (msg: JSONObject) => void;
};

export interface IProvider {
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
  rosServices: RosService[] = [];
  rosTopics: RosTopic[] = [];

  /** List of nodes with same GUID */
  sameIdDict: { [guid: string]: string[] } = {};

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

  systemEnv: JSONObject = {};

  /** All known hostnames for this provides, e.g. IPv4 IPv6 or names */
  hostnames: string[] = [];

  /** Time difference in milliseconds to local host. */
  timeDiff: number = 0;

  /** Time difference in seconds calculated if daemon.ready received. */
  currentDelay: number = 0;

  /** remote timestamp. */
  timestamp: number = 0;

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

  // started echo topics to the received echo events
  private echoTopics: string[] = [];

  /**
   * Keep tracks of running async request, to prevent multiple executions
   */
  private currentRequestList = new Set();

  /**
   * constructor that initializes a new instance of a provider object.
   *
   * @param settings - External settings
   * @param host - IP address or hostname of a remote server on remote host.
   * @param rosVersion - ROS version as string of {'1', '2'}
   * @param port - Port of a remote server on remote host. If zero, it depends on the ros version.
   * @param logger - External logger
   */
  constructor(
    settings: ISettingsContext,
    host: string,
    rosVersion: string,
    port: number = 0,
    networkId: number = 0,
    useSSL: boolean = false,
    logger: ILoggingContext | null = null
  ) {
    this.logger = logger;
    this.settings = settings;
    this.rosVersion = rosVersion;
    this.hostnames = [host];
    this.id = `${host}-${generateUniqueId()}`;
    if (this.type === WebsocketConnection.type) {
      this.connection = new WebsocketConnection(
        host,
        rosVersion,
        port,
        networkId,
        useSSL,
        this.onCloseConnection,
        this.onOpenConnection,
        logger
      );
    }
  }

  public getCallbacks: () => TConCallback[] = () => {
    return [
      { uri: URI.ROS_PROVIDER_LIST, callback: this.providerUpdated },
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
          // this.updateScreens();  <. this request is performed while update nodes
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

  public setSettingsCtx(settings: ISettingsContext): void {
    this.settings = settings;
  }

  public setLoggerCtx(logger: ILoggingContext): void {
    this.logger = logger;
  }

  /** Creates a command string to open screen or log in a terminal */
  public cmdForType: (
    type: CmdType,
    nodeName: string,
    topicName: string,
    screenName: string,
    cmd: string
  ) => Promise<CmdTerminal> = async (type, nodeName = "", topicName = "", screenName = "", cmd = "") => {
    const result = new CmdTerminal();
    let cmdType = type;
    if (cmdType === CmdType.SCREEN && screenName === "") {
      cmdType = CmdType.LOG;
    }
    switch (cmdType) {
      case CmdType.CMD: {
        const prefix = this.rosState?.ros_domain_id ? `export ROS_DOMAIN_ID=${this.rosState?.ros_domain_id}; ` : "";
        result.cmd = cmd ? `${prefix}${cmd}` : `${prefix}/bin/bash`;
        break;
      }
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
      case CmdType.LOG: {
        // eslint-disable-next-line no-case-declarations
        const logPaths = await this.getLogPaths([nodeName]);
        if (logPaths.length > 0) {
          // `tail -f ${logPaths[0].screen_log} \r`,
          result.cmd = `${this.settings.get("logCommand")} ${logPaths[0].screen_log}`;
          result.log = logPaths[0].screen_log;
        }
        break;
      }
      case CmdType.ECHO: {
        if (this.rosState.ros_version === "1") {
          result.cmd = `rostopic echo ${topicName}`;
        } else if (this.rosState.ros_version === "2") {
          const prefix = this.rosState?.ros_domain_id ? `export ROS_DOMAIN_ID=${this.rosState?.ros_domain_id}; ` : "";
          result.cmd = `${prefix}ros2 topic echo ${topicName}`;
        }
        break;
      }
      case CmdType.TERMINAL: {
        const prefix = this.rosState?.ros_domain_id ? `export ROS_DOMAIN_ID=${this.rosState?.ros_domain_id}; ` : "";
        result.cmd = cmd ? `${prefix}${cmd}` : `${prefix}/bin/bash`;
        break;
      }
      case CmdType.SET_TIME:
        result.cmd = cmd;
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
      //
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
   */
  public init: () => Promise<boolean> = () => {
    if (this.connection.connected()) return Promise.resolve(true);

    this.setConnectionState(ConnectionState.STATES.CONNECTING, "");

    const result = this.connection.open();
    return Promise.resolve(result);
  };

  /**
   * Close the connection to the websocket
   */
  public close: () => void = async () => {
    let closeError = "";
    await this.connection.close().catch((error) => {
      console.error("Provider: error when close:", error);
      closeError = error;
    });
    this.setConnectionState(
      ConnectionState.STATES.CLOSED,
      closeError ? `closed with error: ${JSON.stringify(closeError)}` : "closed"
    );
  };

  /**
   * Check if the provider is available
   */
  public isAvailable: () => boolean = () => {
    return this.connection.connected();
  };

  /**
   * Check if the provider is ready
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

  public providerUpdated: () => void = () => {
    this.updateProviderList();
  };

  /**
   * Request a list of available providers using uri URI.ROS_PROVIDER_GET_LIST
   */
  public updateProviderList: (ignoreNewHosts?: boolean | undefined) => Promise<boolean> = async (
    ignoreNewHosts = false
  ) => {
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
      this.logger?.debug(`Providers updated for [${this.name()}]`, "");
      if (this.rosVersion === "1") {
        this.discovery = true;
      }

      // Update list of providers
      let oldRemoteProviders = this.remoteProviders;
      this.remoteProviders = [];
      for (const p of rosProviders) {
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
        } else if (!ignoreNewHosts) {
          // add provider discovered by the contacted provider
          // add discovered provider
          const np = new Provider(
            this.settings,
            p.host,
            p.ros_version ? p.ros_version : "2",
            p.port,
            0,
            false, // TODO get useSSL from settings
            this.logger
          );
          oldRemoteProviders = oldRemoteProviders.filter((orp) => orp.url() !== np.url());
          np.rosState = p;
          np.discovered = [this.id];
          np.hostnames = p.hostnames;
          this.remoteProviders.push(np);
        }
      }
      for (const np of this.remoteProviders) {
        emitCustomEvent(EVENT_PROVIDER_DISCOVERED, new EventProviderDiscovered(np, this));
      }
      for (const p of oldRemoteProviders) {
        emitCustomEvent(EVENT_PROVIDER_REMOVED, new EventProviderRemoved(p, this));
      }
      this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  /**
   * Get content of the file from provider.
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
    if (bitesWritten > 0) {
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
   * Get system info from provider using the websocket uri 'ros.provider.get_system_info'
   */
  public getProviderSystemInfo: () => Promise<object> = async () => {
    const systemInfo = await this.makeCall(URI.ROS_PROVIDER_GET_SYSTEM_INFO, [], true).then((value: TResultData) => {
      if (value.result) {
        this.systemInfo = (value.data as { system_info: object }).system_info;
        return this.systemInfo;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getProviderSystemInfo()`, `${value.message}`);
      return {};
    });
    return Promise.resolve(systemInfo);
  };

  /**
   * Get system environment from provider.
   */
  public getProviderSystemEnv: () => Promise<object> = async () => {
    const systemEnv = await this.makeCall(URI.ROS_PROVIDER_GET_SYSTEM_ENV, [], true).then((value: TResultData) => {
      if (value.result) {
        this.systemEnv = (value.data as { environment: JSONObject }).environment;
        return this.systemEnv;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getProviderSystemEnv()`, `${value.message}`);
      return {};
    });
    return Promise.resolve(systemEnv);
  };

  private calcTimeDiff: (startTs: number, endTs: number, remoteTs: number) => number = (startTs, endTs, remoteTs) => {
    // try to remove JavaScript andNetwork delay
    const diffStartEnd = endTs - startTs;
    const diffToStart = remoteTs - startTs;
    const diffToEnd = endTs - remoteTs;
    let result = Math.abs(Math.abs(diffToStart) + Math.abs(diffToEnd) - diffStartEnd) / 2.0;
    if (diffToStart < 0) {
      result *= -1.0;
    }
    return result;
  };

  /**
   * Updates the time difference in milliseconds to the provider using the websocket uri URI.ROS_PROVIDER_GET_TIMESTAMP
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
      this.timestamp = providerResponse.timestamp;
      const endTs = Date.now();
      this.timeDiff = this.calcTimeDiff(startTs, endTs, providerResponse.timestamp);
      this.logger?.debug(
        `Time difference to [${this.name()}]: approx. ${this.timeDiff}, returned from daemon: ${providerResponse.diff}`,
        ""
      );
      emitCustomEvent(EVENT_PROVIDER_TIME_DIFF, new EventProviderTimeDiff(this, this.timeDiff));
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  /**
   * Get list of available nodes using the uri URI.ROS_NODES_GET_LIST
   */
  public getNodeList: (forceRefresh: boolean) => Promise<RosNode[]> = async (forceRefresh = false) => {
    interface IRosNode {
      id: string;
      name: string;
      status: string;
      namespace: string;
      node_API_URI: string;
      pid: number;
      process_ids: number[];
      masteruri: string;
      location: string;
      is_local: boolean;
      publishers: RosTopicId[];
      subscribers: RosTopicId[];
      services: RosTopicId[];
      screens: string[];
      system_node: boolean;
      guid: string | null;
      is_container: boolean | null;
      container_name: string | null;
      lifecycle_state: string | null;
      lifecycle_available_transitions: [string, number][] | null;
    }
    const rawNodeList = await this.makeCall(URI.ROS_NODES_GET_LIST, [forceRefresh], true).then((value: TResultData) => {
      if (value.result) {
        return (value.data as unknown as IRosNode[]) || [];
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
        for (const n of rawNodeList) {
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
          if (n.process_ids) {
            rn.processIds = n.process_ids;
          }

          // determine GUID of ROS2 nodes. It is the UUID after last '-' in id
          const idSplitted = n.id.split("-");
          if (idSplitted.length > 1) {
            rn.guid = idSplitted[idSplitted.length - 1];
          }
          if (n.is_container) {
            rn.is_container = n.is_container;
          }
          if (n.container_name) {
            rn.container_name = n.container_name;
          }
          if (n.lifecycle_state) {
            rn.lifecycle_state = n.lifecycle_state;
          }
          if (n.lifecycle_available_transitions) {
            rn.lifecycle_available_transitions = n.lifecycle_available_transitions.map((item) => {
              return { label: item[0], id: item[1] };
            });
          }
          rn.publishers = n.publishers;
          rn.subscribers = n.subscribers;
          rn.services = n.services;
          // add screens
          // TODO: Filter screens that belongs to the same master URI
          rn.screens = n.screens;
          rn.isLocal = n.is_local;
          nodeList.set(n.id, rn);
        }

        this.logger?.debug(`Nodes updated for [${this.name()}]`, "");
        if (this.rosVersion === "1") {
          this.discovery = rawNodeList.length > 0;
        }
        return Array.from(nodeList.values());
      } catch (error) {
        this.logger?.error(`Provider [${this.name()}]: Error at getNodeList()`, `${error}`);
        return Promise.resolve([]);
      }
    }

    return Promise.resolve([]);
  };

  public getTopic(id: RosTopicId): RosTopic | undefined {
    return this.rosTopics.find((item) => item.name === id.name && item.msg_type === id.msg_type);
  }

  /**
   * Get list of available service using the uri URI.ROS_NODES_GET_SERVICES
   */
  public getServiceList: (filter: RosTopicId[]) => Promise<RosService[]> = async (filter = []) => {
    const rawSrvList = await this.makeCall(URI.ROS_NODES_GET_SERVICES, [filter], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as unknown as RosService[];
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getServiceList()`, `${value.message}`);
      return [];
    });

    return Promise.resolve(rawSrvList);
  };

  /**
   * Get list of available topics using the uri URI.ROS_NODES_GET_TOPICS
   */
  public getTopicList: (filter: RosTopicId[]) => Promise<RosTopic[]> = async (filter = []) => {
    const rawTopicsList = await this.makeCall(URI.ROS_NODES_GET_TOPICS, [filter], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as unknown as RosTopic[];
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getTopicList()`, `${value.message}`);
      return [];
    });

    return Promise.resolve(rawTopicsList);
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
   */
  public getPackageList: (force: boolean) => Promise<RosPackage[]> = async (force) => {
    const comparePackages: (a: RosPackage, b: RosPackage) => number = (a, b) => {
      if (a.path < b.path) {
        return -1;
      }
      if (a.path > b.path) {
        return 1;
      }
      return 0;
    };
    this.packages = [];
    const result = await this.makeCall(URI.ROS_PACKAGES_GET_LIST, [force], true).then((value: TResultData) => {
      if (value.result) {
        const rosPackageList: RosPackage[] = (value.data as RosPackage[]) || [];
        return rosPackageList.sort(comparePackages);
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
   * @param path - Folder path to retrieve content from.
   * @return Returns a list of [PathItem] elements
   */
  public getPathList: (path: string) => Promise<PathItem[]> = async (path) => {
    const result = await this.makeCall(URI.ROS_PATH_GET_LIST_RECURSIVE, [path], true).then((value: TResultData) => {
      if (value.result) {
        const fileList: PathItem[] = [];
        const uniquePaths: string[] = [];
        const pathItems = (value.data as PathItem[]) || [];
        for (const p of pathItems) {
          if (!uniquePaths.includes(p.path)) {
            fileList.push(new PathItem(p.path, p.mtime, p.size, p.type, this.connection.host));
            uniquePaths.push(p.path);
          }
        }
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
   * @param nodes - List of node names to get log files
   * @return Returns a list of [PathItem] elements
   */
  public getLogPaths: (nodes: string[]) => Promise<LogPathItem[]> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PATH_GET_LOG_PATHS, [nodes], true).then((value: TResultData) => {
      if (value.result) {
        const logPathList: LogPathItem[] = [];
        const logPathItems = (value.data as LogPathItem[]) || [];
        for (const p of logPathItems) {
          logPathList.push(new LogPathItem(p.node, p.screen_log, p.screen_log_exists, p.ros_log, p.ros_log_exists));
        }
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
   * @param nodes - List of node names to clear log files
   * @return Returns a list of [PathItem] elements
   */
  public clearLogPaths: (nodes: string[]) => Promise<TResultClearPath[]> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PATH_CLEAR_LOG_PATHS, [nodes], true).then((value: TResultData) => {
      if (value.result) {
        const logPathList: TResultClearPath[] = [];
        const clearPathResults = (value.data as TResultClearPath[]) || [];
        for (const p of clearPathResults) {
          logPathList.push({
            node: p.node,
            result: p.result,
            message: p.message,
          });
        }
        return logPathList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at clearLogPaths()`, `${value.message}`);
      return [];
    });
    return Promise.resolve(result);
  };

  /**
   * Clear the path of log files for given nodes
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
   */
  public launchLoadFile: (request: LaunchLoadRequest, reload: boolean) => Promise<LaunchLoadReply | null> = async (
    request,
    reload
  ) => {
    let result: LaunchLoadReply | null = null;

    if (reload)
      result = await this.makeCall(URI.ROS_LAUNCH_RELOAD, [request], true).then((value: TResultData) => {
        if (value.result) {
          return value.data as LaunchLoadReply;
        }
        this.logger?.error(`Provider [${this.name()}]: Error at launchLoadFile()`, `${value.message}`);
        return null;
      });
    else {
      result = await this.makeCall(URI.ROS_LAUNCH_LOAD, [request], true).then((value: TResultData) => {
        if (value.result) {
          return value.data as LaunchLoadReply;
        }
        this.logger?.error(`Provider [${this.name()}]: Error at launchLoadFile()`, `${value.message}`);
        return null;
      });
    }
    return Promise.resolve(result);
  };

  /**
   * Unload a launch file from the launch servicer
   */
  public launchUnloadFile: (request: LaunchFile) => Promise<LaunchLoadReply | null> = async (request) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_UNLOAD, [request], true).then((value: TResultData) => {
      if (value.result) {
        return value.data as LaunchLoadReply;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at launchUnloadFile()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /** Update the screens of the node and create a new node it not exists */
  public applyScreens: (screens: ScreensMapping[] | null) => boolean = (screens = null) => {
    this.screens = screens ? screens : [];
    let nodesUpdated: boolean = false;
    // update nodes
    // if node exist (it is running), only update the associated launch file
    this.screens.map((screen) => {
      const idxNode = this.rosNodes.findIndex((n) => {
        return n.name === screen.name;
      });
      if (idxNode >= 0) {
        let oScr = this.rosNodes[idxNode].screens;
        if (oScr === undefined) oScr = [];
        const nScr = screen.screens ? screen.screens : [];
        if (JSON.stringify(oScr.sort()) !== JSON.stringify(nScr.sort())) {
          nodesUpdated = true;
          this.rosNodes[idxNode].screens = nScr;
        }
        if (nScr.length > 0 && this.rosNodes[idxNode].status === RosNodeStatus.INACTIVE) {
          this.rosNodes[idxNode].status = RosNodeStatus.ONLY_SCREEN;
        }
      } else {
        // create a new node for screen
        nodesUpdated = true;
        const n = new RosNode(screen.name, screen.name, "/", "", RosNodeStatus.INACTIVE);
        // idGlobal should be the same for life of the node on remote host
        n.idGlobal = `${this.id}${n.id.replaceAll("/", "#")}`;
        n.providerName = this.name();
        n.providerId = this.id;
        n.status = RosNodeStatus.ONLY_SCREEN;
        // set if system node, e.g. ttyd
        if (screen.name.startsWith("/ttyd-") || screen.name.startsWith("/roscore-")) {
          n.system_node = true;
        }
        n.screens = screen.screens;
        n.isLocal = true;
        n.processIds.push(Number.parseInt(screen.name.split(".")[0]));
        if (screen.name === "/mas-gui") {
          n.system_node = true;
        }
        this.rosNodes.push(n);
      }
    });
    const nodesToRemove: string[] = [];
    this.rosNodes.forEach((node: RosNode, idx: number) => {
      if (node.status !== RosNodeStatus.RUNNING) {
        if (node.screens && node.screens.length > 0) {
          const screenMapping = this.screens.find((s) => node.id === s.name);
          if (!screenMapping) {
            nodesUpdated = true;
            this.rosNodes[idx].screens = [];
          }
        }
        if (node.screens?.length === 0 && this.rosNodes[idx].launchInfo.size === 0) {
          // remove node if no launchInfo?
          nodesToRemove.push(node.idGlobal);
          nodesUpdated = true;
        }
      }
    });
    this.rosNodes = this.rosNodes.filter((node) => !nodesToRemove.includes(node.idGlobal));
    emitCustomEvent(EVENT_PROVIDER_SCREENS, new EventProviderScreens(this, this.screens));
    if (nodesUpdated) {
      emitCustomEvent(EVENT_PROVIDER_ROS_NODES, new EventProviderRosNodes(this, this.rosNodes));
    }
    return nodesUpdated;
  };

  /**
   * Updates the screens. This is called on message received by subscribed topic and also by request.
   * On request the msgs list should be null. In this case the list is requested by this method.
   */
  public updateScreens: () => Promise<boolean> = async () => {
    const result = await this.getScreenList();
    if (result) {
      this.applyScreens(result);
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
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
    const diagStatus = diags.status || [];
    for (const status of diagStatus) {
      const matchingNode = this.rosNodes.find((node) => node.id === status.name);
      if (matchingNode) {
        matchingNode.diagnosticLevel = status.level;
        matchingNode.diagnosticMessage = status.message;
        if (matchingNode.diagnosticStatus && matchingNode.diagnosticStatus.length === 0) {
          matchingNode.diagnosticStatus.push(status);
        } else if (matchingNode.diagnosticStatus) {
          // do not add the same value
          const lastStatus = matchingNode.diagnosticStatus[matchingNode.diagnosticStatus.length - 1];
          if (
            lastStatus.level !== status.level ||
            lastStatus.message !== status.message ||
            (lastStatus.values || []).length !== (status.values || []).length
          ) {
            matchingNode.diagnosticStatus.push(status);
          }
        }
      }
    }
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
   */
  public updateLaunchContent: () => Promise<boolean> = async () => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_LIST, [], true).then((value: TResultData) => {
      if (value.result) {
        const parsedList = value.data as LaunchContent[];
        const launchList: LaunchContent[] = [];

        if (parsedList) {
          for (const parsed of parsedList) {
            // filter only the launch files associated to the provider
            if (this.rosState.masteruri && parsed.masteruri !== this.rosState.masteruri) {
              return false;
            }
            launchList.push(
              new LaunchContent(
                parsed.path,
                parsed.args || [],
                parsed.masteruri || "",
                parsed.host,
                parsed.nodes || [],
                parsed.parameters?.map((p) => {
                  if (this.rosVersion === "1") {
                    return new RosParameter("", p.name, p.value, p.type, this.id);
                  }
                  return new RosParameter("", p[0], p[1], "", this.id);
                }) || [],
                parsed.associations || [],
                parsed.warnings || []
              )
            );
          }
        }
        this.launchFiles = launchList;
        emitCustomEvent(EVENT_PROVIDER_LAUNCH_LIST, new EventProviderLaunchList(this, this.launchFiles));

        const capabilityGroupParamName = `/${this.settings.get("capabilityGroupParameter")}`;
        const nodeGroups: { [nodeName: string]: { namespace?: string; name?: string } } = {};
        // update nodes
        // Add nodes from launch files to the list of nodes
        for (const launchFile of this.launchFiles) {
          const nodes: LaunchNodeInfo[] = launchFile.nodes || [];
          for (const [idxLn, launchNode] of nodes.entries()) {
            // get parameter of a node and determine the capability group parameter
            const nodeParameters: RosParameter[] = [];
            let nodeGroup: { namespace?: string; name?: string } = {};
            let groupParameterFound = false;
            let nodesParametersFound = false;
            // const uniqueNodeName = launchNode.unique_name ? launchNode.unique_name : "";
            const uniqueNodeName = launchNode.node_name ? launchNode.node_name : launchNode.unique_name;
            if (uniqueNodeName) {
              const capabilityGroupOfNode = `${uniqueNodeName}${capabilityGroupParamName}`;
              // update parameters
              if (this.rosVersion === "1") {
                const parameters: RosParameter[] = launchNode.parameters || [];
                for (const p of parameters) {
                  if (nodesParametersFound) {
                    // skip parse further parameter if we found one and next was not in node namespace
                    // assumption: parameters are sorted
                    // break;
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
                }
              } else {
                const env_capability_group = launchNode.env?.MAS_CAPABILITY_GROUP;
                if (env_capability_group) {
                  nodeGroup = { namespace: "", name: `{${env_capability_group}}` };
                }
                const parameters: RosParameter[] = launchNode.parameters || [];
                // in ros2 we have a lot of temporary files with one parameter.
                // We join the content of these files to one dictionary
                let allJoinedParams = {};
                for (const p of parameters) {
                  let joinedParam = false;
                  if (p.type.indexOf("yaml") >= 0 || p.type  === 'dict') {
                    let allNodes = p.value["/**"];
                    if (!allNodes && launchNode.node_name) {
                      allNodes = p.value[launchNode.node_name];
                    }
                    if (!allNodes && launchNode.node_name) {
                      // TODO: split node name and get the path step by step from: {ns: {node: {ros__parameters}}}
                    }
                    if (allNodes) {
                      const rosParameters = allNodes["ros__parameters"];
                      if (rosParameters) {
                        const capabilityGroup = rosParameters["capability_group"];
                        if (capabilityGroup) {
                          const ns = "";
                          nodeGroup = { namespace: ns, name: `{${capabilityGroup}}` };
                        }
                        allJoinedParams = { ...allJoinedParams, ...rosParameters };
                        joinedParam = true;
                      }
                    }
                  } else if (p.name === "capability_group") {
                    nodeGroup = { namespace: "", name: `{${p.value}}` };
                  }
                  if (!joinedParam) {
                    nodeParameters.push(new RosParameter(launchNode.node_name || "", p.name, p.value, "", this.id));
                  }
                }
                if (Object.keys(allJoinedParams).length > 0) {
                  nodeParameters.push(
                    new RosParameter(
                      launchNode.node_name || "",
                      "/tmp/launch_params_*/**/ros__parameters",
                      allJoinedParams,
                      "yaml",
                      this.id
                    )
                  );
                }
              }
              if (launchNode.node_name && nodeGroup.name) {
                nodeGroups[launchNode.node_name] = nodeGroup;
              }
              let associations: string[] = [];
              const lAssociations = launchFile.associations || [];
              for (const item of lAssociations) {
                if (item.node === uniqueNodeName) {
                  associations = item.nodes || [];
                }
              }
              nodes[idxLn].associations = associations;
              nodes[idxLn].parameters = nodeParameters;

              // determine sigkill_timeout
              if (!launchNode.sigkill_timeout || launchNode.sigkill_timeout <= 0) {
                let killTime: RosParameterValue | undefined = undefined;
                killTime = LaunchNodeInfo.getParam(
                  nodeParameters || [],
                  launchNode.node_name || "",
                  "mas/kill_on_stop"
                );
                if (killTime === undefined) {
                  killTime = LaunchNodeInfo.getParam(
                    nodeParameters || [],
                    launchNode.node_name || "",
                    "nm/kill_on_stop"
                  );
                }
                if (killTime === undefined) {
                  killTime = LaunchNodeInfo.getEnvParam(launchNode.env, "MAS_KILL_ON_STOP");
                }
                if (killTime !== undefined) {
                  nodes[idxLn].sigkill_timeout = Number.parseInt(killTime as string);
                }
              }

              //launchPaths
              //parameters
              // if node exist (it is running), only update the associated launch file
              let nodeIsRunning = false;
              const iNode = this.rosNodes.findIndex((n) => {
                return n.name === uniqueNodeName;
              });
              if (iNode >= 0) {
                this.rosNodes[iNode].launchInfo.set(launchFile.path, launchNode);
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
                  launchNode.node_namespace ? launchNode.node_namespace : "/",
                  "",
                  RosNodeStatus.INACTIVE
                );
                n.launchInfo.set(launchFile.path, launchNode);
                // idGlobal should be the same for life of the node on remote host
                n.idGlobal = `${this.id}${n.id.replaceAll("/", "#")}`;
                n.providerName = this.name();
                n.providerId = this.id;
                n.isLocal = true;
                if (nodeGroup.name) {
                  n.capabilityGroup = nodeGroup;
                }
                const screens = this.screens.filter((screen) => {
                  return screen.name === n.name;
                });
                n.screens = screens.length > 0 ? screens[0].screens : [];
                this.rosNodes.push(n);
              }
            }
          }
        }

        // set tags for nodelets/composable and other tags)
        const composableManagers: string[] = [];
        for (const [idx, n] of this.rosNodes.entries()) {
          const nodeGroup = nodeGroups[n.name];
          if (nodeGroup) {
            this.rosNodes[idx].capabilityGroup = nodeGroup;
          }
          // Check if this is a nodelet/composable and assign tags accordingly.
          const composableParents = n.getAllContainers();
          const tags: TTag[] = [];
          for (const item of composableParents) {
            const composableParent = item.split("|").slice(-1).at(0);
            if (composableParent) {
              if (!composableManagers.includes(composableParent)) {
                composableManagers.push(composableParent);
              }
              tags.push({
                id: `nodelet-${composableParent}`,
                data: "Nodelet",
                color: TagColors[composableManagers.indexOf(composableParent) % TagColors.length],
                tooltip: `Composable node, container: ${composableParent}`,
              });
            }
          }
          // mark nodes with same GUID
          if (n.guid && this.sameIdDict[n.guid]?.length > 1) {
            tags.push({
              id: n.guid,
              data: FingerprintIcon,
              color: colorFromHostname(n.guid),
              tooltip: `Nodes with same id ${n.guid}`,
              onClick: (event: React.MouseEvent) => {
                // if (n.guid) navigator.clipboard.writeText(n.guid);
                // this.logger?.success(`${n.guid} copied!`, "", true);
                emitCustomEvent(EVENT_FILTER_NODES, eventFilterNodes(n.guid as string));
                event?.stopPropagation();
              },
            });
          }

          if (tags.length > 0) {
            this.rosNodes[idx].tags = tags;
          }
        }
        // Assign tags to the found nodelet/composable managers.
        for (const managerId of composableManagers) {
          const node = this.rosNodes.find((n) => n.id === managerId || n.name === managerId);
          if (node) {
            const tag: TTag = {
              id: "Manager",
              data: "Manager",
              color: TagColors[composableManagers.indexOf(node.id) % TagColors.length],
              tooltip: "Manager of a composable nodes",
            };
            if (!node.tags) {
              node.tags = [tag];
            } else {
              node.tags.unshift(tag);
            }
          }
        }

        this.daemon = true;
        const changed = this.applyScreens(this.screens);
        if (!changed) {
          emitCustomEvent(EVENT_PROVIDER_ROS_NODES, new EventProviderRosNodes(this, this.rosNodes));
        }
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
   */
  public launchGetIncludedFiles: (request: LaunchIncludedFilesRequest) => Promise<LaunchIncludedFile[] | null> = async (
    request
  ) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_INCLUDED_FILES, [request], true).then(
      (value: TResultData) => {
        if (value.result) {
          const parsedList = (value.data as LaunchIncludedFile[]) || [];
          const launchList: LaunchIncludedFile[] = [];

          for (const lf of parsedList) {
            launchList.push(
              new LaunchIncludedFile(
                this.connection.host,
                lf.path,
                lf.line_number,
                lf.inc_path,
                lf.exists,
                lf.raw_inc_path,
                lf.rec_depth,
                lf.args || [],
                lf.default_inc_args || [],
                lf.size,
                lf.conditional_excluded
              )
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
   */
  public getMessageStruct: (request: string) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_MSG_STRUCT, [request], true).then((value: TResultData) => {
      if (value.result) {
        const response = value.data as LaunchMessageStruct;
        if (response.valid) {
          return response;
        }
        this.logger?.error(`Provider [${this.name()}]: Can't parse message: ${request}`, response.error_msg);
        return null;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getMessageStruct()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Returns a messages struct for given service type.
   */
  public getServiceStruct: (request: string) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_SRV_STRUCT, [request], true).then((value: TResultData) => {
      if (value.result) {
        const response = value.data as LaunchMessageStruct;
        if (response.valid) {
          return response;
        }
        this.logger?.error(`Provider [${this.name()}]: Can't parse service: ${request}`, response.error_msg);
        return null;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getServiceStruct()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Start Publisher
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
   */
  public callService: (request: LaunchCallService) => Promise<LaunchMessageStruct | null> = async (request) => {
    const result = await this.makeCall(URI.ROS_LAUNCH_CALL_SERVICE, [request], true).then((value: TResultData) => {
      if (value.result) {
        if (!value.data) {
          return null;
        }
        const response = value.data as LaunchMessageStruct;
        return response;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at callService()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Requests the list with known message types
   */
  public getRosMessageMessageTypes: () => Promise<string[]> = async () => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_MESSAGE_TYPES, [], true).then((value: TResultData) => {
      if (value.result) {
        if (!value.data) {
          return [];
        }
        const response = value.data as string[];
        return response;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getRosMessageMessageTypes()`, `${value.message}`);
      return [];
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
      this.logger?.error(
        `Provider [${this.name()}]: [updateSubscribedMessage] Could not parse message ${msg}`,
        `Error: ${error}`
      );
    }
  };

  /**
   * Start subscriber node for given topic
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
  public startNode: (node: RosNode) => Promise<TResultStartNode> = async (node) => {
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
      "", // cmd
      node.ignore_timer || false
    );

    const result = await this.makeCall(URI.ROS_LAUNCH_START_NODE, [request], true).then((value: TResultData) => {
      emitCustomEvent(EVENT_PROVIDER_NODE_STARTED, new EventProviderNodeStarted(this, node));
      if (value.result) {
        const response = value.data as LaunchNodeReply;

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
            details: response.status.msg || "",
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
          message: "Start Node: Invalid response",
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
   */
  public screenKillNode: (name: string, signal?: string) => Promise<Result> = async (name, signal) => {
    const result = await this.makeCall(URI.ROS_SCREEN_KILL_NODE, [name, signal], true).then((value: TResultData) => {
      if (value.result) {
        const parsed = value.data as Result;
        if (!parsed.result) {
          this.logger?.error(
            `Provider [${this.name()}]: Fail to kill the node [${name}]: ${parsed.message}`,
            parsed.message,
            false
          );
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
   */
  private getScreenList: () => Promise<ScreensMapping[] | null> = async () => {
    const result = await this.makeCall(URI.ROS_SCREEN_GET_LIST, [], true).then((value: TResultData) => {
      if (value.result) {
        const screenMappings = (value.data as ScreensMapping[]) || [];
        const screenList: ScreensMapping[] = [];
        for (const p of screenMappings) {
          screenList.push(new ScreensMapping(p.name, p.screens || []));
        }
        return screenList;
      }
      this.logger?.error(`Provider [${this.name()}]: Error at getScreenList()`, `${value.message}`);
      return null;
    });
    return Promise.resolve(result);
  };

  /**
   * Get list of diagnostics
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
   */
  private updateSystemWarnings: () => Promise<SystemWarningGroup[] | null> = async () => {
    const result = await this.makeCall(URI.ROS_PROVIDER_GET_WARNINGS, [], true).then((value: TResultData) => {
      if (value.result) {
        const warnings = value.data as SystemWarningGroup[];
        if (Array.isArray(warnings)) {
          this.warnings = warnings;
          return warnings;
        }
        this.logger?.error(
          `Provider [${this.name()}]: Error at updateSystemWarnings()`,
          `reported message is not an array ${JSON.stringify(warnings)}`
        );
      } else {
        this.logger?.error(`Provider [${this.name()}]: Error at updateSystemWarnings()`, `${value.message}`);
      }
      return null;
    });
    if (this.warnings.length > 0) {
      emitCustomEvent(EVENT_PROVIDER_WARNINGS, new EventProviderWarnings(this, this.warnings));
    }
    return Promise.resolve(result);
  };

  /**
   * Return a list with loggers for given node
   */
  public getNodeLoggers: (node: string, loggerNames: string[]) => Promise<LoggerConfig[]> = async (
    node,
    loggerNames = []
  ) => {
    const result = await this.makeCall(URI.ROS_NODES_GET_LOGGERS, [node, loggerNames], true).then(
      (value: TResultData) => {
        if (value.result) {
          // handle the result of type: {result: bool, message: str}
          if (!Array.isArray(value.data) && !value.result) {
            this.logger?.error(`Provider [${this.name()}]: Error at getNodeLoggers(): ${value.message}`, "");
            return [];
          }
          return value.data as LoggerConfig[];
        }
        this.logger?.warn(`Provider [${this.name()}]: Error at getNodeLoggers()`, `${value.message}`, true);
        return [];
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Set new logger levels for a ros node
   */
  public setNodeLoggers: (node: string, loggers: LoggerConfig[]) => Promise<Result> = async (node, loggers) => {
    const result = await this.makeCall(URI.ROS_NODES_SET_LOGGER_LEVEL, [node, loggers], true).then(
      (value: TResultData) => {
        if (value.result) {
          return value.data as Result;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at setNodeLoggers()`, `${value.message}`);
        return new Result(false, `Provider [${this.name()}]: Error at setNodeLoggers(): ${value.message}`);
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Unregister a node given a name
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
   */
  public getParameterList: () => Promise<TParamListResult> = async () => {
    const result = await this.makeCall("ros.parameters.get_list", [], true, 10000).then((value: TResultData) => {
      if (value.result) {
        // update with id
        if ((value.data as TParamListResult)?.params) {
          const paramList = [
            ...(((value.data as TParamListResult)?.params as RosParameter[])?.map((item) => {
              item.id = `${item.node}#${item.name}`;
              return item;
            }) || []),
          ];
          return { params: paramList, errors: (value.data as TParamListResult)?.errors || [] };
        }
        // old style
        const paramList = [
          ...((value.data as RosParameter[])?.map((item) => {
            item.id = `${item.node}#${item.name}`;
            return item;
          }) || []),
        ];
        return { params: paramList, errors: [] };
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at getParameterList()`, `${value.message}`);
      return { params: [], errors: [value.message] };
    });
    return Promise.resolve(result);
  };

  /**
   * Return a list with all registered parameters values and types for a list of nodes
   */
  public getNodeParameters: (nodes: string[]) => Promise<TParamListResult> = async (nodes) => {
    const result = await this.makeCall(URI.ROS_PARAMETERS_GET_NODE_PARAMETERS, [nodes], false, 60000).then(
      (value: TResultData) => {
        if (value.result) {
          // update with id
          if ((value.data as TParamListResult)?.params) {
            const paramList = [
              ...(((value.data as TParamListResult)?.params as RosParameter[])?.map((item) => {
                item.id = `${item.node}#${item.name}`;
                return item;
              }) || []),
            ];
            return { params: paramList, errors: (value.data as TParamListResult)?.errors || [] };
          }
          // old style
          const paramList = [
            ...((value.data as RosParameter[])?.map((item) => {
              item.id = `${item.node}#${item.name}`;
              return item;
            }) || []),
          ];
          return { params: paramList, errors: [] };
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at stopNode()`, `${value.message}`);
        return { params: [], errors: [value.message] };
      }
    );
    return Promise.resolve(result);
  };

  /**
   * Set the value of a parameter
   */
  public setParameter: (
    paramName: string,
    paramType: string,
    paramValue: string,
    nodeName: string
  ) => Promise<TResultParam> = async (paramName, paramType, paramValue, nodeName) => {
    const setResult = await this.makeCall(
      URI.ROS_PARAMETERS_SET_PARAMETER,
      [paramName, paramType, paramValue, nodeName],
      true
    ).then((value: TResultData) => {
      if (value.result) {
        const setResponse = value.data as TResultParam;
        return setResponse;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at setParameter()`, `${value.message}`);
      return {
        result: false,
        message: value.message,
      };
    });
    return Promise.resolve(setResult);
  };

  /**
   * Delete a list of parameters
   */
  public deleteParameters: (parameters: string[], nodeName: string) => Promise<TResult> = async (
    parameters,
    nodeName
  ) => {
    const delResult = await this.makeCall(URI.ROS_PARAMETERS_DELETE_PARAMETERS, [parameters, nodeName], true).then(
      (value: TResultData) => {
        if (value.result) {
          const setResponse = value.data as TResult;
          return setResponse;
        }
        this.logger?.debug(`Provider [${this.name()}]: Error at deleteParameters()`, `${value.message}`);
        return {
          result: false,
          message: value.message,
        };
      }
    );
    return Promise.resolve(delResult);
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
  public updateRosNodes: (msg: JSONObject, forceRefresh?: boolean) => void = async (msg, forceRefresh = false) => {
    this.logger?.debug(`Trigger update ros nodes for ${this.id}`, "");
    const msgObj = msg as unknown as { path: string; action: string };
    if (msgObj?.path) {
      emitCustomEvent(EVENT_PROVIDER_LAUNCH_LOADED, new EventProviderLaunchLoaded(this, msgObj.path));
    }
    if (await this.lockRequest("updateRosNodes")) {
      return;
    }

    // get nodes from remote provider
    const nlUnfiltered = await this.getNodeList(forceRefresh);
    const sameIdDict = {};
    const nl = nlUnfiltered.filter((n) => {
      let ignored = false;

      // ignore nodes, which belongs to a discovered remote provider.
      // Otherwise, the node is displayed under Not connected host.
      if (
        (this.rosState.masteruri && n.masteruri !== this.rosState.masteruri) ||
        (n.location instanceof String && n.location === "remote") ||
        !n.isLocal
      ) {
        ignored = true;
        ignored =
          this.remoteProviders.filter((prov) => {
            for (const nodeLocation of n.location) {
              for (const provName of prov.hostnames) {
                if (nodeLocation.includes(provName)) {
                  return true;
                }
              }
            }
            return false;
          }).length > 0;
      }

      // update the list with same GUIDs
      if (n.guid) {
        const sameIdEntry = sameIdDict[n.guid];
        if (sameIdEntry) {
          sameIdEntry.push(n.id);
        } else {
          sameIdDict[n.guid] = [n.id];
        }
      }
      // exclude ignored nodes
      for (const ignoredNode of this.IGNORED_NODES) {
        if (n.name.indexOf(ignoredNode) !== -1) ignored = true;
      }
      return !ignored;
    });

    const dynamicReconfigureNodes = new Set<string>();

    // check if nodes are not available or run in other host (not monitoring)
    // biome-ignore lint/complexity/noForEach: <explanation>
    nl.forEach((n) => {
      // idGlobal should be the same for life of the node on remote host
      n.idGlobal = `${this.id}${n.id.replaceAll("/", "#")}`;
      n.providerName = this.name();
      n.providerId = this.id;

      // copy (selected) old state
      const oldNode = this.rosNodes.find((item) => item.idGlobal === n.idGlobal);
      if (oldNode) {
        n.diagnosticStatus = oldNode.diagnosticStatus;
        n.diagnosticLevel = oldNode.diagnosticLevel;
        n.diagnosticMessage = oldNode.diagnosticMessage;
        n.launchPath = oldNode.launchPath;
        n.group = oldNode.group;
        n.launchInfo = oldNode.launchInfo;
        n.rosLoggers = oldNode.rosLoggers;
        n.is_container = oldNode.is_container;
        n.container_name = oldNode.container_name;
        n.screens = oldNode.screens;
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
      const services: RosTopicId[] = n.services || [];
      for (const service of services) {
        if (service.name.endsWith("/set_parameters")) {
          const serviceNs = service.name.slice(0, -15);
          n.dynamicReconfigureServices.push(serviceNs);
          if (serviceNs !== n.name) {
            dynamicReconfigureNodes.add(serviceNs);
          }
        }
      }
    });
    // biome-ignore lint/complexity/noForEach: <explanation>
    nl.forEach((n) => {
      if (dynamicReconfigureNodes.has(n.name)) {
        n.dynamicReconfigureServices.push(n.name);
      }
    });
    this.rosNodes = nl;
    this.sameIdDict = sameIdDict;
    await this.updateLaunchContent();
    await this.updateScreens();
    this.updateRosServices();
    this.updateRosTopics();
    this.unlockRequest("updateRosNodes");
  };

  public updateRosServices: () => void = async () => {
    this.logger?.debug(`Trigger update ros services for ${this.id}`, "");
    if (await this.lockRequest("updateRosServices")) {
      return;
    }

    // get service from remote provider
    this.rosServices = await this.getServiceList([]);
    emitCustomEvent(EVENT_PROVIDER_ROS_SERVICES, new EventProviderRosServices(this));
    this.unlockRequest("updateRosServices");
  };

  /**
   * Try to search for a process which can be the node. If not found, possible process will be returned.
   */
  public findNodeProcess: (node: string) => Promise<TResultProcess> = async (node) => {
    const result = await this.makeCall(URI.ROS_PROCESS_FIND_NODE, [node], false, 60000).then((value: TResultData) => {
      if (value.result) {
        return value.data as TResultProcess;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at findNodeProcess()`, `${value.message}`);
      return { result: result.result, message: result.message };
    });
    return Promise.resolve(result);
  };

  public killProcess: (pid: number) => Promise<TResult> = async (pid) => {
    const result = await this.makeCall(URI.ROS_PROCESS_KILL, [pid], false, 60000).then((value: TResultData) => {
      if (value.result) {
        return value.data as TResult;
      }
      this.logger?.debug(`Provider [${this.name()}]: Error at killProcess()`, `${value.message}`);
      return { result: result.result, message: result.message };
    });
    return Promise.resolve(result);
  };

  public updateRosTopics: () => void = async () => {
    this.logger?.debug(`Trigger update ros topics for ${this.id}`, "");
    if (await this.lockRequest("updateRosTopics")) {
      return;
    }

    // get publishers from remote provider
    this.rosTopics = await this.getTopicList([]);
    emitCustomEvent(EVENT_PROVIDER_ROS_TOPICS, new EventProviderRosTopics(this));
    this.unlockRequest("updateRosTopics");
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

    this.applyScreens(msg.screens as unknown as ScreensMapping[]);
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
        for (const item of this.getCallbacks()) {
          this.logger?.info(`  register callback for ${item.uri}`, "", false);
          this.registerCallback(item.uri, item.callback);
        }
        return true;
      });
    return false;
  };

  /* Generic functions */

  /**
   * Execute a call considering [callAttempts] and [delayCallAttempts]
   */
  private makeCall: (uri: string, args: unknown[], lockRequest: boolean, timeout?: number) => Promise<TResultData> =
    async (uri, args, lockRequest = true, timeout = undefined) => {
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
      const callRequest: (
        _uri: string,
        _args: unknown[],
        currentAttempt?: number,
        _timeout?: number
      ) => Promise<TResultData> = async (_uri, _args, currentAttempt = 0, _timeout = undefined) => {
        try {
          const r = await this.connection.call(_uri, _args, _timeout).catch((err) => {
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
          return callRequest(_uri, _args, currentAttempt + 1, timeout);
        }
      };

      const result = await callRequest(uri, args, undefined, timeout);
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

  private onOpenConnection: () => void = () => {
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
  private unlockRequest: (request: string) => void = (request) => {
    this.currentRequestList.delete(request);
    if (this.currentRequestList.size === 0) {
      emitCustomEvent(EVENT_PROVIDER_ACTIVITY, new EventProviderActivity(this, false, ""));
    }
  };
}
