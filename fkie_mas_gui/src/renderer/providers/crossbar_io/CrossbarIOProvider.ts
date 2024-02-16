import { emitCustomEvent } from 'react-custom-events';
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
  LaunchNodeReply,
  LaunchPublishMessage,
  LogPathItem,
  PathItem,
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
} from '../../models';
import CrossbarIO from './crossbar_io';

import { TagColors } from '../../components/UI/Colors';

import {
  DEFAULT_BUG_TEXT,
  ILoggingContext,
} from '../../context/LoggingContext';
import {
  ISettingsContext,
  getCrossbarPortFromRos,
} from '../../context/SettingsContext';
import { delay, generateUniqueId } from '../../utils';
import { ConnectionState } from '../ConnectionState';
import { RosProviderState } from '../RosProviderState';
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_DISCOVERED,
  EVENT_PROVIDER_LAUNCH_LIST,
  EVENT_PROVIDER_PATH_EVENT,
  EVENT_PROVIDER_ROS_NODES,
  EVENT_PROVIDER_SCREENS,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX,
  EVENT_PROVIDER_TIME_DIFF,
  EVENT_PROVIDER_WARNINGS,
  EventProviderActivity,
  EventProviderDiscovered,
  EventProviderLaunchList,
  EventProviderPathEvent,
  EventProviderRosNodes,
  EventProviderScreens,
  EventProviderState,
  EventProviderSubscriberEvent,
  EventProviderTimeDiff,
  EventProviderWarnings,
} from '../events';

import CmdTerminal from '../CmdTerminal';
import CmdType from '../CmdType';

/**
 * CrossbarIOProvider class to connect with a running WAMP Router
 */
class CrossbarIOProvider {
  IGNORED_NODES = [];

  /**
   * Unique Identifier
   */
  id: string;

  type: string = 'crossbar-wamp';

  userName: string = '';

  rosVersion: string;

  /** Default connection settings */
  connectionTimeout: number = 5000;

  crossbar: CrossbarIO;

  isLocalHost: boolean = false;

  discovered: boolean = false;

  /** State of the connection to remote provider. */
  connectionState: ConnectionState = ConnectionState.STATES.UNKNOWN;

  /** State provided from ROS provider */
  rosState: RosProviderState = new RosProviderState();

  daemonVersion: DaemonVersion = new DaemonVersion('', '');

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

  /** Providers discovered be this provider.
   * For each provider in this list an event (EVENT_PROVIDER_DISCOVERED) will be emitted. */
  remoteProviders: CrossbarIOProvider[] = [];

  screens: ScreensMapping[] = [];

  systemInfo: {} = {};

  systemEnv: {} = {};

  /** All known hostnames for this provides, e.g. IPv4 IPv6 or names */
  hostnames: string[] = [];

  /** Time difference in milliseconds to local host. */
  timeDiff: number = 0;

  /** Warnings reported by the provider. */
  warnings: SystemWarningGroup[] = [];

  /**
   * External logger
   */
  logger: ILoggingContext | null;

  settings: ISettingsContext;

  callAttempts: number = 10;

  delayCallAttempts: number = 1000;

  errorDetails: string = '';

  // started echo topics to the received echo events
  private echoTopics: string[] = [];

  /**
   * Keep tracks of running async request, to prevent multiple executions
   */
  private currentRequestList = new Set();

  /**
   * constructor that initializes a new instance of a CrossbarIO object.
   *
   * @param {ISettingsContext} settings - External settings
   * @param {string} host - IP address or hostname of a CROSSBAR server on remote host.
   * @param {string} rosVersion - ROS version as string of {'1', '2'}
   * @param {number} port - Port of a CROSSBAR server on remote host. If zero, it depends on the ros version.
   * @param {ILoggingContext | null} logger - External logger
   */
  constructor(
    settings: ISettingsContext,
    host: string,
    rosVersion: string,
    port: number = 0,
    useSSL: boolean = false,
    logger: ILoggingContext | null = null,
  ) {
    this.logger = logger;
    this.settings = settings;
    this.rosVersion = rosVersion;
    const providerPort = port !== 0 ? port : getCrossbarPortFromRos(rosVersion);
    this.crossbar = new CrossbarIO(
      host,
      providerPort,
      this.onCloseCrossbar,
      this.onOpenCrossbar,
      useSSL,
    );

    this.hostnames = [host];
    this.id = `${host}-${providerPort}-${generateUniqueId()}`;
    if (this.logger)
      this.logger.debug(
        `Create new connection to ${this.crossbar.wsURI}`,
        `${JSON.stringify(this)}`,
      );
  }

  public url: () => string = () => {
    return this.crossbar.wsURI;
  };

  public name: () => string = () => {
    let name = this.userName;
    if (!name) {
      name = this.rosState.name ? this.rosState.name : this.crossbar.host;
    }
    return name;
  };

  public host: () => string = () => {
    return this.crossbar.host;
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
    cmd: string,
  ) => Promise<CmdTerminal> = async (
    type,
    nodeName = '',
    topicName = '',
    screenName = '',
    cmd = '',
  ) => {
    const result = new CmdTerminal();
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
          let createdScreenName = '';
          if (this.rosState.ros_version === '1') {
            createdScreenName = nodeName
              .substring(1)
              .replaceAll('_', '__')
              .replaceAll('/', '_');
          } else if (this.rosState.ros_version === '2') {
            createdScreenName = nodeName.substring(1).replaceAll('/', '.');
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
          result.cmd = `/usr/bin/less -fKLnQrSU ${logPaths[0].screen_log}`;
          result.log = logPaths[0].screen_log;
        }
        break;
      case CmdType.ECHO:
        if (this.rosState.ros_version === '1') {
          result.cmd = `rostopic echo ${topicName}`;
        } else if (this.rosState.ros_version === '2') {
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

  public setConnectionState: (state: ConnectionState, details: string) => void =
    async (state: ConnectionState, details: string = '') => {
      // ignore call with no state changes
      if (this.connectionState === state) return;

      // update to new state
      const oldState = this.connectionState;
      this.connectionState = state;
      this.errorDetails = details;
      // send an event to inform all listener about new state
      emitCustomEvent(
        EVENT_PROVIDER_STATE,
        new EventProviderState(this, state, oldState, details),
      );
      // get provider details depending on new state
      if (
        state === ConnectionState.STATES.CONNECTING &&
        oldState === ConnectionState.STATES.STARTING
      ) {
        delay(3000);
      } else if (state === ConnectionState.STATES.CROSSBAR_CONNECTED) {
        this.registerCallbacks()
          .then(() => {
            this.setConnectionState(
              ConnectionState.STATES.CROSSBAR_REGISTERED,
              '',
            );
            return true;
          })
          .catch((error) => {
            this.setConnectionState(ConnectionState.STATES.ERRORED, error);
          });
      } else if (state === ConnectionState.STATES.CROSSBAR_REGISTERED) {
        this.getDaemonVersion()
          .then((dv) => {
            this.daemonVersion = dv;
            this.updateProviderList();
            return true;
          })
          .catch((error) => {
            this.setConnectionState(ConnectionState.STATES.ERRORED, error);
            return false;
          });
      } else if (state === ConnectionState.STATES.CONNECTED) {
        this.updateTimeDiff(3);
        this.getProviderSystemInfo();
        this.getProviderSystemEnv();
        this.updateRosNodes();
        this.updateDiagnostics(null);
        this.getPackageList();
        // this.launchGetList();
      }
    };

  /**
   * Initializes the ROS provider
   *
   * @return {Promise} True is a connection to a WAMP Router succeeded
   */
  public init: () => Promise<boolean> = () => {
    if (this.crossbar.connectionIsOpen) return Promise.resolve(true);

    this.setConnectionState(ConnectionState.STATES.CONNECTING, '');

    const result = this.crossbar.open(this.connectionTimeout);
    return Promise.resolve(result);
  };

  /**
   * Close the connection to the WAMP Router
   */
  public close: () => void = async () => {
    let closeError = '';
    await this.crossbar.close().catch((error) => {
      console.error(`CrossbarIOProvider: error when close:`, error);
      closeError = error;
    });
    this.setConnectionState(
      ConnectionState.STATES.CLOSED,
      closeError
        ? `closed with error: ${JSON.stringify(closeError)}`
        : 'closed',
    );
  };

  /**
   * Reconnect the connection to the WAMP Router
   */
  public reconnect: () => void = async () => {
    if (this.crossbar.connectionIsOpen) return Promise.resolve(true);

    this.setConnectionState(ConnectionState.STATES.CONNECTING, 'reconnect');

    return this.crossbar.open(this.connectionTimeout);
  };

  /**
   * Check if the provider is available
   *
   * @return {boolean} true is available
   */
  public isAvailable: () => boolean = () => {
    return this.crossbar.connectionIsOpen;
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

  /**
   * Request a list of available providers using the WAMP uri URI.ROS_PROVIDER_GET_LIST
   *
   * @return {CrossbarIOProvider} Empty
   */
  public updateProviderList: () => Promise<boolean> = async () => {
    const result = await this.makeCall(
      URI.ROS_PROVIDER_GET_LIST,
      [],
      true,
      true,
    );
    if (result[0]) {
      if (this.logger) {
        this.logger.debug(`Providers updated for [${this.name()}]`, ``);
      }
      this.discovery = true;

      // Update list of providers
      const rosProviders: RosProviderState[] = JSON.parse(result[1]);
      this.remoteProviders = [];
      rosProviders.forEach((p: RosProviderState) => {
        // the remote provider list should contain at least the details to itself (origin === true)
        if (p.origin) {
          // apply remote attributes to new current provider
          this.rosState = p;
          // TODO: visualize warning if hosts are not equal
          if (
            this.crossbar.host !== 'localhost' &&
            this.crossbar.host !== p.host &&
            !p.hostnames.includes(this.crossbar.host)
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
          const np = new CrossbarIOProvider(
            this.settings,
            p.host,
            p.ros_version ? p.ros_version : '2',
            p.port,
            false, // TODO get useSSL from settings
            this.logger,
          );
          this.remoteProviders.push(np);
          np.rosState = p;
          np.discovered = true;
          emitCustomEvent(
            EVENT_PROVIDER_DISCOVERED,
            new EventProviderDiscovered(np, this),
          );
        }
      });
      this.setConnectionState(ConnectionState.STATES.CONNECTED, '');
      return Promise.resolve(true);
    }
    if (result[2].includes('wamp.error.no_such_procedure')) {
      this.discovery = false;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at updateProviderList()`,
        `${result[2]}`,
      );
    }
    this.setConnectionState(
      ConnectionState.STATES.ERRORED,
      `Provider [${this.name()}]: updateProviderList() result: ${result[2]}`,
    );
    return Promise.resolve(false);
    // throw Error(
    //   `Provider [${this.name()}]: getProviderList() result: ${result[2]}`,
    // );
  };

  /**
   * Get content of the file from provider.
   *
   * @return {Promise<{ file: FileItem; error: string }>}
   */
  public getFileContent: (
    path: string,
  ) => Promise<{ file: FileItem; error: string }> = async (path) => {
    const result = await this.makeCall(URI.ROS_FILE_GET, [path], false, true);
    if (result[0]) {
      const fileItem: FileItem = JSON.parse(result[1]) as FileItem;
      fileItem.host = this.crossbar.host;
      return Promise.resolve({ file: fileItem, error: '' });
    }
    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: can not get content for ${path}: `,
        `${result[2]}`,
        false,
      );
    }
    return Promise.resolve({
      file: new FileItem(this.crossbar.host, path),
      error: result[2],
    });
  };

  /**
   * Save content of the file in providers file system.
   *
   * @return {Promise<{ bytesWritten: number; error: string }>}
   */
  public saveFileContent: (
    file: FileItem,
  ) => Promise<{ bytesWritten: number; error: string }> = async (file) => {
    const result = await this.makeCall(URI.ROS_FILE_SAVE, [file], false, true);
    if (result[0]) {
      return Promise.resolve({
        bytesWritten: JSON.parse(result[1]) as number,
        error: '',
      });
    }
    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: can not save content to ${file.path}: `,
        `${result[2]}`,
        false,
      );
    }
    return Promise.resolve({
      bytesWritten: -1,
      error: result[2],
    });
  };

  /**
   * Get daemon version using the WAMP uri 'ros.daemon.get_version'
   *
   * @return {Promise<DaemonVersion>}
   */
  public getDaemonVersion: () => Promise<DaemonVersion> = async () => {
    const result = await this.makeCall(URI.ROS_DAEMON_VERSION, [], true, false);
    if (result[0]) {
      this.daemonVersion = JSON.parse(result[1]);
      this.daemon = true;
      return Promise.resolve(this.daemonVersion);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getDaemonVersion()`,
        `${result[2]}`,
      );
    }
    this.daemon = false;
    throw Error(
      `Provider [${this.name()}]: getDaemonVersion() result: ${result[2]}`,
    );
  };

  /**
   * Get system info from provider using the WAMP uri 'ros.provider.get_system_info'
   *
   * @return {Promise<any>}
   */
  public getProviderSystemInfo: () => Promise<any> = async () => {
    const result = await this.makeCall(
      URI.ROS_PROVIDER_GET_SYSTEM_INFO,
      [],
      true,
      true,
    );
    if (result[0]) {
      this.systemInfo = JSON.parse(result[1]);
      return Promise.resolve(this.systemInfo);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getProviderSystemInfo()`,
        `${result[2]}`,
      );
    }
    return Promise.resolve({});
  };

  /**
   * Get system environment from provider using the WAMP uri 'ros.provider.get_system_env'
   *
   * @return {Promise<any>}
   */
  public getProviderSystemEnv: () => Promise<any> = async () => {
    const result = await this.makeCall(
      URI.ROS_PROVIDER_GET_SYSTEM_ENV,
      [],
      true,
      true,
    );
    if (result[0]) {
      this.systemEnv = JSON.parse(result[1]);
      return Promise.resolve(this.systemEnv);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getProviderSystemEnv()`,
        `${result[2]}`,
      );
    }
    return Promise.resolve({});
  };

  /**
   * Updates the time difference in milliseconds to the provider using the WAMP uri URI.ROS_PROVIDER_GET_TIMESTAMP
   * Emit event on successful update.
   */
  public updateTimeDiff: (count: number | undefined) => Promise<boolean> =
    async (count = undefined) => {
      const startTs = Date.now();
      const result = await this.makeCall(
        URI.ROS_PROVIDER_GET_TIMESTAMP,
        [startTs],
        true,
        true,
      );
      if (result[0]) {
        const providerResponse = JSON.parse(result[1]);
        const endTs = Date.now();
        const delayTs = (endTs - startTs) / 2.0;
        let diffTime = providerResponse.timestamp - startTs;
        // time difference smaller then half delay results in 0
        if (diffTime > delayTs) {
          diffTime -= delayTs;
        } else if (diffTime >= 0) {
          diffTime = 0;
        }
        if (this.logger) {
          this.logger.debug(
            `Time difference to [${this.name()}]: approx. ${diffTime}, returned from daemon: ${
              providerResponse.diff
            }`,
            ``,
          );
        }
        if (diffTime > 0) {
          // check multiple times for time difference. E.g. on startup when the delay is to long.
          const newCount = count === undefined ? 2 : count;
          if (newCount > 0) {
            return this.updateTimeDiff(newCount - 1);
          }
        }
        this.timeDiff = diffTime;
        emitCustomEvent(
          EVENT_PROVIDER_TIME_DIFF,
          new EventProviderTimeDiff(this, diffTime),
        );
        return true;
      }

      if (this.logger) {
        this.logger.error(
          `Provider [${this.name()}]: Error at updateTimeDiff()`,
          `${result[2]}`,
        );
      }
      return false;
    };

  /**
   * Get list of available nodes using the WAMP uri URI.ROS_NODES_GET_LIST
   *
   * @return {Promise} Returns a list of ROS nodes
   */
  public getNodeList: () => Promise<RosNode[]> = async () => {
    const result = await this.makeCall(URI.ROS_NODES_GET_LIST, [], true, true);

    if (result[0]) {
      // cast incoming object into a proper representation
      const nodeList = new Map<string, RosNode>();
      try {
        const rawNodeList = JSON.parse(result[1]);
        if (rawNodeList) {
          rawNodeList.forEach(
            (n: {
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
            }) => {
              let status = RosNodeStatus.UNKNOWN;
              if (n.status && n.status === 'running')
                status = RosNodeStatus.RUNNING;
              else if (n.status && n.status === 'inactive')
                status = RosNodeStatus.INACTIVE;
              else if (n.status && n.status === 'not_monitored')
                status = RosNodeStatus.NOT_MONITORED;
              else if (n.status && n.status === 'dead')
                status = RosNodeStatus.DEAD;
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
                n.system_node,
              );

              if (n.parent_id) {
                rn.parent_id = n.parent_id;
              }

              // Add Array elements
              n.publishers.forEach((s: RosTopic) => {
                rn.publishers.set(
                  s.name,
                  new RosTopic(s.name, s.msgtype, s.publisher, s.subscriber),
                );
              });

              n.subscribers.forEach((s: RosTopic) => {
                rn.subscribers.set(
                  s.name,
                  new RosTopic(s.name, s.msgtype, s.publisher, s.subscriber),
                );
              });

              n.services.forEach((s: RosService) => {
                rn.services.set(
                  s.name,
                  new RosService(
                    s.name,
                    s.srvtype,
                    s.masteruri,
                    s.service_API_URI,
                    s.provider,
                    s.location,
                  ),
                );
              });

              // add screens
              // TODO: Filter screens that belongs to the same master URI
              rn.screens = n.screens;
              nodeList.set(n.id, rn);
            },
          );

          if (this.logger) {
            this.logger.debug(`Nodes updated for [${this.name()}]`, ``);
          }
          this.discovery = true;
          return Array.from(nodeList.values());
        }
      } catch (error) {
        if (this.logger) {
          this.logger.error(
            `Provider [${this.name()}]: Error at getNodeList()`,
            `${error}`,
          );
        }
        return Promise.resolve([]);
      }
    }

    if (result[2].includes('wamp.error.no_such_procedure')) {
      this.discovery = false;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getNodeList()`,
        `${result[2]}`,
      );
    }
    return Promise.resolve([]);
  };

  /**
   * Return the current host, using the WAMP URI URI.ROS_SYSTEM_GET_URI
   *
   * @return {Promise} Returns a list of ROS nodes
   */
  public getSystemUri = async () => {
    const result = await this.makeCall(URI.ROS_SYSTEM_GET_URI, [], true, true);
    if (result[0]) {
      return result[1];
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getSystemUri()`,
        `${result[2]}`,
      );
    }

    return '';
  };

  /* File manager */

  /**
   * Get list of available packages using the WAMP uri 'ros.packages.get_list'
   *
   * @return {Promise<RosPackage[]>} Returns a list of ROS packages
   */
  public getPackageList: () => Promise<RosPackage[]> = async () => {
    this.packages = [];
    const result = await this.makeCall('ros.packages.get_list', [], true, true);

    const comparePackages = (a: RosPackage, b: RosPackage) => {
      if (a.path < b.path) {
        return -1;
      }
      if (a.path > b.path) {
        return 1;
      }
      return 0;
    };

    if (result[0]) {
      const packageList: RosPackage[] = [];
      JSON.parse(result[1]).forEach((p: RosPackage) => {
        packageList.push(new RosPackage(p.name, p.path));
      });

      this.packages = packageList;
      return packageList.sort(comparePackages);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getPackageList()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve([]);
  };

  /**
   * Get list of files available for a given path
   *
   * @param {string} path - Folder path to retrieve content from.
   * @return {Promise<PathItem[]>} Returns a list of [PathItem] elements
   */
  public getPathList: (path: string) => Promise<PathItem[]> = async (path) => {
    const result = await this.makeCall(
      URI.ROS_PATH_GET_LIST_RECURSIVE,
      [path],
      true,
      true,
    );

    if (result[0]) {
      const fileList: PathItem[] = [];
      const uniquePaths: string[] = [];
      JSON.parse(result[1]).forEach((p: PathItem) => {
        if (!uniquePaths.includes(p.path)) {
          fileList.push(
            new PathItem(p.path, p.mtime, p.size, p.type, this.crossbar.host),
          );
          uniquePaths.push(p.path);
        }
      });

      return fileList;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getPackageList()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve([]);
  };

  /**
   * Get the path of log files for all [nodes]
   *
   * @param {string[]} nodes - List of node names to get log files
   * @return {Promise<LogPathItem>} Returns a list of [PathItem] elements
   */
  public getLogPaths: (nodes: string[]) => Promise<LogPathItem[]> = async (
    nodes,
  ) => {
    const result = await this.makeCall(
      URI.ROS_PATH_GET_LOG_PATHS,
      [nodes],
      true,
      true,
    );

    if (result[0]) {
      const logPathList: LogPathItem[] = [];
      JSON.parse(result[1]).forEach((p: LogPathItem) => {
        logPathList.push(
          new LogPathItem(
            p.node,
            p.screen_log,
            p.screen_log_exists,
            p.ros_log,
            p.ros_log_exists,
          ),
        );
      });

      return logPathList;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getLogPaths()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve([]);
  };

  /**
   * Clear the path of log files for given nodes
   *
   * @param {string[]} nodes - List of node names to clear log files
   * @return {Promise<{node: str, result: bool, message: str}[]>} Returns a list of [PathItem] elements
   */
  public clearLogPaths: (
    nodes: string[],
  ) => Promise<{ node: string; result: boolean; message: string }[]> = async (
    nodes,
  ) => {
    const result = await this.makeCall(
      URI.ROS_PATH_CLEAR_LOG_PATHS,
      [nodes],
      true,
      true,
    );

    if (result[0]) {
      const logPathList: { node: string; result: boolean; message: string }[] =
        [];
      JSON.parse(result[1]).forEach(
        (p: { node: string; result: boolean; message: string }) => {
          logPathList.push({
            node: p.node,
            result: p.result,
            message: p.message,
          });
        },
      );

      return logPathList;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at clearLogPaths()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve([]);
  };

  /**
   * Clear the path of log files for given nodes
   *
   * @return {Promise<{result: bool, message: str}>} Returns a list of [PathItem] elements
   */
  public rosCleanPurge: () => Promise<{ result: boolean; message: string }> =
    async () => {
      const result = await this.makeCall(
        URI.ROS_PROVIDER_ROS_CLEAN_PURGE,
        [],
        true,
        true,
      );

      if (result[0]) {
        const returnResult: {
          result: boolean;
          message: string;
        } = JSON.parse(result[1]);

        return returnResult;
      }

      if (this.logger) {
        this.logger.error(
          `Provider [${this.name()}]: Error at rosCleanPurge()`,
          `${result[2]}`,
        );
      }

      return Promise.resolve({
        result: false,
        message: `Provider [${this.name()}]: Error at rosCleanPurge(): ${
          result[2]
        }`,
      });
    };

  /**
   * Terminate all running subprocesses (ROS, Crossbar, TTYD) of the provider
   *
   * @return {Promise<{result: bool, message: str}>}
   */
  public shutdown: () => Promise<{ result: boolean; message: string }> =
    async () => {
      const result = await this.makeCall(
        URI.ROS_PROVIDER_SHUTDOWN,
        [],
        true,
        true,
      );

      if (result[0]) {
        const returnResult: {
          result: boolean;
          message: string;
        } = JSON.parse(result[1]);

        return returnResult;
      }

      if (this.logger) {
        this.logger.error(
          `Provider [${this.name()}]: Error at shutdown()`,
          `${result[2]}`,
        );
      }

      return Promise.resolve({
        result: false,
        message: `Provider [${this.name()}]: Error at shutdown(): ${result[2]}`,
      });
    };

  /* Launch servicer */

  /**
   * Load a new launch file into launch servicer
   *
   * @param {LaunchLoadRequest} request - Launch request
   * @return {Promise<LaunchLoadReply>} Returns a LaunchLoadReply
   */
  public launchLoadFile: (
    request: LaunchLoadRequest,
    reload: boolean,
  ) => Promise<LaunchLoadReply | null> = async (request, reload) => {
    let result = null;

    if (reload)
      result = await this.makeCall(
        URI.ROS_LAUNCH_RELOAD,
        [request],
        true,
        true,
      );
    else
      result = await this.makeCall(URI.ROS_LAUNCH_LOAD, [request], true, true);

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      const loadReply = new LaunchLoadReply(
        parsed.status,
        parsed.paths,
        parsed.args,
        parsed.changed_nodes,
      );
      return loadReply;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at launchLoadFile()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  /**
   * Unload a launch file from the launch servicer
   *
   * @param {LaunchFile} request - Launch file to be removed
   * @return {Promise<LaunchLoadReply>} Returns a LaunchLoadReply
   */
  public launchUnloadFile: (
    request: LaunchFile,
  ) => Promise<LaunchLoadReply | null> = async (request) => {
    const result = await this.makeCall(
      URI.ROS_LAUNCH_UNLOAD,
      [request],
      true,
      true,
    );

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      const loadReply = new LaunchLoadReply(
        parsed.status,
        parsed.paths,
        parsed.args,
        parsed.changed_nodes,
      );
      return loadReply;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at launchUnloadFile()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  /**
   * Updates the screens. This is called on message received by subscribed topic and also by request.
   * On request the msgs list should be null. In this case the list is requested by this method.
   */
  public updateScreens: (msgs: ScreensMapping[] | null) => Promise<boolean> =
    async (msgs = null) => {
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
      emitCustomEvent(
        EVENT_PROVIDER_SCREENS,
        new EventProviderScreens(this, screens),
      );
      return Promise.resolve(true);
    };

  /**
   * Updates the diagnostics. This is called on message received by subscribed topic and also by request.
   * On request the msgs list should be null. In this case the list is requested by this method.
   */
  public updateDiagnostics: (msg: DiagnosticArray | null) => Promise<boolean> =
    async (msg = null) => {
      let diags = msg;
      if (diags === null) {
        diags = await this.getDiagnostics();
        if (diags === null) {
          return Promise.resolve(false);
        }
      }
      // update the screens
      diags.status.forEach((status) => {
        const matchingNode = this.rosNodes.find(
          (node) => node.id === status.name,
        );
        if (matchingNode) {
          matchingNode.diagnosticLevel = status.level;
          matchingNode.diagnosticMessage = status.message;
          if (matchingNode.diagnosticStatus.length === 0) {
            matchingNode.diagnosticStatus.push(status);
          } else {
            // do not add the same value
            const lastStatus =
              matchingNode.diagnosticStatus[
                matchingNode.diagnosticStatus.length - 1
              ];
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

  /**
   * Get the list of nodes loaded in launch files. update launch files into provider object
   *
   * @return {Promise<LaunchContent>} Returns a list of LaunchContent elements
   */
  public updateLaunchContent: () => Promise<boolean> = async () => {
    const result = await this.makeCall(URI.ROS_LAUNCH_GET_LIST, [], true, true);
    if (result[0]) {
      const parsedList = JSON.parse(result[1]);
      const launchList: LaunchContent[] = [];

      if (parsedList) {
        parsedList.forEach((parsed: LaunchContent) => {
          // filter only the launch files associated to the provider
          if (
            this.rosState.masteruri &&
            parsed.masteruri !== this.rosState.masteruri
          ) {
            return;
          }
          launchList.push(
            new LaunchContent(
              parsed.path,
              parsed.args,
              parsed.masteruri,
              parsed.host,
              parsed.nodes,
              parsed.parameters.map(
                (p) => new RosParameter(p.name, p.value, p.type, this.id),
              ),
              parsed.associations,
            ),
          );
        });
      }
      this.launchFiles = launchList;
      emitCustomEvent(
        EVENT_PROVIDER_LAUNCH_LIST,
        new EventProviderLaunchList(this, this.launchFiles),
      );

      // update nodes
      // Add nodes from launch files to the list of nodes
      this.launchFiles.forEach((launchFile) => {
        launchFile.nodes.forEach((launchNode) => {
          const nodeParameters = new Map();

          const uniqueNodeName = launchNode.unique_name
            ? launchNode.unique_name
            : '';
          if (uniqueNodeName) {
            // update parameters
            launchFile.parameters.forEach((p) => {
              if (p.name.indexOf(uniqueNodeName) !== -1) {
                nodeParameters.set(p.name.replace(uniqueNodeName, ''), p.value);
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
              this.rosNodes[iNode].parameters = nodeParameters;
              this.rosNodes[iNode].launchInfo = launchNode;
              this.rosNodes[iNode].associations = associations;
              nodeIsRunning = true;
            }

            if (!nodeIsRunning) {
              // if node is not running add it as inactive node
              const n = new RosNode(
                uniqueNodeName,
                uniqueNodeName,
                launchNode.node_namespace ? launchNode.node_namespace : '',
                '',
                RosNodeStatus.INACTIVE,
              );
              n.launchPaths.add(launchFile.path);
              n.parameters = nodeParameters;
              n.launchInfo = launchNode;
              // idGlobal should be the same for life of the node on remote host
              n.idGlobal = `${this.id}${n.id.replaceAll('/', '.')}`;
              n.providerName = this.name();
              n.providerId = this.id;
              n.associations = associations;
              this.rosNodes.push(n);
            }
          }
        });
        // set tags for nodelets/composable and other tags)
        const composableManagers: string[] = [];

        this.rosNodes.forEach((n) => {
          // Check if this is a nodelet/composable and assign tags accordingly.
          let composableParent =
            n?.parent_id || n.launchInfo?.composable_container;
          if (composableParent) {
            composableParent = composableParent.split('|').slice(-1).at(0);
            if (composableParent) {
              if (!composableManagers.includes(composableParent)) {
                composableManagers.push(composableParent);
              }
              n.tags = [
                {
                  text: 'Nodelet',
                  color:
                    TagColors[
                      composableManagers.indexOf(composableParent) %
                        TagColors.length
                    ],
                },
              ];
            }
          }
        });

        // Assign tags to the found nodelet/composable managers.
        composableManagers.forEach((managerId) => {
          const node = this.rosNodes.find(
            (n) => n.id === managerId || n.name === managerId,
          );
          if (node) {
            node.tags = [
              {
                text: 'Manager',
                color:
                  TagColors[
                    composableManagers.indexOf(node.id) % TagColors.length
                  ],
              },
            ];
          }
        });
      });
      // do not sort nodes -> start order defined by capability_group
      // this.rosNodes.sort(compareRosNodes);
      // await this.updateScreens(null);
      this.daemon = true;
      emitCustomEvent(
        EVENT_PROVIDER_ROS_NODES,
        new EventProviderRosNodes(this, this.rosNodes),
      );

      return Promise.resolve(true);
    }
    this.launchFiles = [];

    if (result[2].includes('wamp.error.no_such_procedure')) {
      this.daemon = false;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at updateLaunchContent()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(false);
  };

  /**
   * Returns all included files in given launch file.
   *
   * @param {LaunchIncludedFilesRequest} request - Launch file to be requested
   * @return {Promise<LogPathItem>} Returns a list of [PathItem] elements
   */
  public launchGetIncludedFiles: (
    request: LaunchIncludedFilesRequest,
  ) => Promise<LaunchIncludedFile[] | null> = async (
    request: LaunchIncludedFilesRequest,
  ) => {
    const result = await this.makeCall(
      URI.ROS_LAUNCH_GET_INCLUDED_FILES,
      [request],
      true,
      true,
    );

    if (result[0]) {
      const parsedList = JSON.parse(result[1]);
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
                this.crossbar.host,
                lf.path,
                lf.line_number,
                lf.inc_path,
                lf.exists,
                lf.raw_inc_path,
                lf.rec_depth,
                lf.args,
                lf.default_inc_args,
                lf.size,
              ),
            );
          },
        );
      }

      return launchList;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at launchGetIncludedFiles()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  /**
   * Returns a messags struct for given message type.
   *
   * @param {string} request - Topic type to be requested
   * @return {Promise<LaunchMessageStruct>} Returns a message struct
   */
  public getMessageStruct: (
    request: string,
  ) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(
      URI.ROS_LAUNCH_GET_MSG_STRUCT,
      [request],
      true,
      true,
    );

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      const response = new LaunchMessageStruct(
        parsed.msg_type,
        parsed.data,
        parsed.valid,
        parsed.error_msg,
      );
      if (response.valid) {
        return response.data;
      }
      if (this.logger) {
        this.logger.error(`Can't parse message: ${request}`, parsed.error_msg);
      }
      return Promise.resolve(null);
    }
    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getMessageStruct()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  /**
   * Returns a messags struct for given service type.
   *
   * @param {string} request - Service type to be requested
   * @return {Promise<LaunchMessageStruct>} Returns a message struct
   */
  public getServiceStruct: (
    request: string,
  ) => Promise<LaunchMessageStruct | null> = async (request: string) => {
    const result = await this.makeCall(
      URI.ROS_LAUNCH_GET_SRV_STRUCT,
      [request],
      true,
      true,
    );

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      const response = new LaunchMessageStruct(
        parsed.msg_type,
        parsed.data,
        parsed.valid,
        parsed.error_msg,
      );
      if (response.valid) {
        return response.data;
      }
      if (this.logger) {
        this.logger.error(`Can't parse service: ${request}`, parsed.error_msg);
      }
      return Promise.resolve(null);
    }
    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getServiceStruct()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  /**
   * Start Publisher
   *
   * @param {LaunchPublishMessage} request - Launch request
   * @return {Promise<Boolean>} Returns true if success
   */
  public publishMessage: (request: LaunchPublishMessage) => Promise<boolean> =
    async (request) => {
      const result = await this.makeCall(
        URI.ROS_LAUNCH_PUBLISH_MESSAGE,
        [request],
        true,
        true,
      );

      if (result[0]) {
        const parsed = JSON.parse(result[1]);
        return parsed;
      }

      if (this.logger) {
        this.logger.error(
          `Provider [${this.name()}]: Error at publishMessage()`,
          `${result[2]}`,
        );
      }

      return false;
    };

  /**
   * Call Service
   *
   * @param {LaunchCallService} request - Launch request
   * @return {Promise<LaunchMessageStruct | null>} Returns true if success
   */
  public callService: (
    request: LaunchCallService,
  ) => Promise<LaunchMessageStruct | null> = async (request) => {
    const result = await this.makeCall(
      URI.ROS_LAUNCH_CALL_SERVICE,
      [request],
      true,
      true,
    );

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      const response = new LaunchMessageStruct(
        parsed.msg_type,
        parsed.data,
        parsed.valid,
        parsed.error_msg,
      );
      return response;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at callService()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve(null);
  };

  private generateCrossbarTopic: (topic: string) => string = (topic) => {
    return `${URI.ROS_SUBSCRIBER_EVENT_PREFIX}.${topic.replaceAll('/', '_')}`;
  };

  /**
   * Initializes new providers using message callback from provider server
   * First closes all existing providers and then create new ones.
   */
  private callbackNewSubscribedMessage: (msgs: string[]) => void = (msgs) => {
    if (this.logger)
      this.logger.debugCrossbar(
        URI.ROS_SUBSCRIBER_EVENT_PREFIX,
        msgs[0],
        '',
        this.id,
      );
    const msg = msgs[0];

    // ignore empty and duplicated messages
    if (msg.length === 0) {
      return;
    }

    try {
      const msgParsed: SubscriberEvent = JSON.parse(msg);
      emitCustomEvent(
        `${EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX}_${msgParsed.topic}`,
        new EventProviderSubscriberEvent(this, msgParsed),
      );
    } catch (error) {
      if (this.logger)
        this.logger.error(
          `[updateSubscribedMessage] Could not parse message ${msg}`,
          `Error: ${error}`,
        );
    }
  };

  /**
   * Start subscriber node for given topic
   *
   * @param {SubscriberNode} subscriber - Launch request
   * @return {Promise<boolean | null>} Returns true if success
   */
  public startSubscriber: (request: SubscriberNode) => Promise<boolean | null> =
    async (request) => {
      const hasEcho = this.echoTopics.includes(request.topic);
      if (hasEcho) {
        return Promise.resolve(null);
      }
      this.echoTopics.push(request.topic);
      const result = await this.makeCall(
        URI.ROS_SUBSCRIBER_START,
        [request],
        true,
        true,
      );

      if (result[0]) {
        const parsed = JSON.parse(result[1]);
        const cbTopic = this.generateCrossbarTopic(request.topic);
        if (this.logger)
          this.logger.debug(
            `CROSSBAR_IO: (${this.name()}) Subscribing to: [${cbTopic}]`,
            '',
          );
        await this.crossbar.subscribe(
          cbTopic,
          this.callbackNewSubscribedMessage,
        );
        if (this.logger)
          this.logger.debug(
            `Started subscriber node for '${request.topic} [${
              request.message_type
            }]' on '${this.name()}'`,
            '',
          );
        return Promise.resolve(parsed);
      }

      if (this.logger) {
        this.logger.error(
          `Provider [${this.name()}]: Error at startSubscriber()`,
          `${result[2]}`,
        );
      }

      return Promise.resolve(null);
    };

  /**
   * Stop subscriber node for given topic
   *
   * @param {string} topic - topic name
   * @return {Promise<{result: boolean, message: string}>} Returns true if success
   */
  public stopSubscriber: (
    topic: string,
  ) => Promise<{ result: boolean; message: string }> = async (topic) => {
    const hasTopic = this.echoTopics.includes(topic);
    if (hasTopic) {
      this.echoTopics = this.echoTopics.filter((e) => e !== topic);
      const cbTopic = this.generateCrossbarTopic(topic);
      await this.crossbar.closeSubscription(cbTopic);
    }
    let result = null;
    result = await this.makeCall(URI.ROS_SUBSCRIBER_STOP, [topic], true, true);

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      return parsed;
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at stopSubscriber()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve({ result: false, message: result[2] });
  };

  /**
   * Update the filter for given topic
   */
  public updateFilterRosTopic: (
    topicName: string,
    msg: SubscriberFilter,
  ) => Promise<boolean | null> = async (topicName, msg) => {
    const cbTopic = `${URI.ROS_SUBSCRIBER_FILTER_PREFIX}.${topicName.replaceAll(
      '/',
      '_',
    )}`;
    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Publish to: [${cbTopic}]`,
        '',
      );
    const result = await this.crossbar.publish(cbTopic, [msg]);
    return result;
  };

  /**
   * Start a ROS node using a given provider and node object
   */
  public startNode = async (node: RosNode) => {
    if (node.providerId !== this.id) {
      return {
        success: false,
        message: `Inconsistent provider (${
          node.providerName
        } vs. ${this.name()})`,
        details: DEFAULT_BUG_TEXT,
        response: null,
      };
    }

    // TODO: Add log level and format?
    const request = new LaunchNode(
      node.name, // name
      '', // opt_binary
      `${node.launchPath}`, // opt_launch
      '', // log level
      '', // log format
      node.masteruri?.length > 0
        ? node.masteruri
        : `${this.rosState.masteruri}`, // masteruri
      true, // reload global parameters
      '', // cmd
    );

    const result = await this.makeCall(
      URI.ROS_LAUNCH_START_NODE,
      [request],
      true,
      true,
    );

    if (!result[0]) {
      return {
        success: false,
        message: 'Invalid message from [ros.launch.start_node]',
        details: result[2],
        response: null,
      };
    }

    const parsed = JSON.parse(result[1]);
    const response = new LaunchNodeReply(
      parsed.name,
      parsed.status,
      parsed.paths,
      parsed.launch_files,
    );

    if (!response) {
      return {
        success: false,
        message: `Invalid return. Node: ${node.id}`,
        details: DEFAULT_BUG_TEXT,
        response: null,
      };
    }

    if (response.status.code === 'OK') {
      return {
        success: true,
        message: `Node started: [${node.id}]`,
        details: '',
        response,
      };
    }

    if (response.status.code === 'NODE_NOT_FOUND') {
      return {
        success: false,
        message: `Could not start node [${node.name}]: Node not found`,
        details: `Node: ${node.id} Details: ${response.status.msg}`,
        response,
      };
    }

    if (response.status.code === 'MULTIPLE_LAUNCHES') {
      return {
        success: false,
        message: `Could not start node [${node.name}]: Multiple launches found`,
        details: `Node: ${node.id} Details: ${response.status.msg}`,
        response,
      };
    }

    if (response.status.code === 'MULTIPLE_BINARIES') {
      return {
        success: false,
        message: `Could not start node [${node.name}]: Multiple binaries found`,
        details: `Node: ${node.id} Details: ${response.status.msg}`,
        response,
      };
    }

    if (response.status.code === 'CONNECTION_ERROR') {
      return {
        success: false,
        message: `Could not start node [${node.name}]: Connection error`,
        details: `Node: ${node.id} Details: ${response.status.msg}`,
        response,
      };
    }

    if (response.status.code === 'ERROR') {
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
  };

  /**
   * Stop a node given a name
   *
   * @param {string} id - Unique ID of the node to be stopped
   * @return {Promise<Result>} Returns a result
   */
  public stopNode: (id: string) => Promise<Result> = async (id) => {
    const result = await this.makeCall(
      URI.ROS_NODES_STOP_NODE,
      [id],
      true,
      true,
    );
    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      if (parsed.result) {
        return parsed;
      }
      if (this.logger) {
        // use debug because of spamming messages when nodes does not run
        this.logger.debug(`${parsed.message}`, parsed.message);
      }
      return parsed;
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at stopNode()`,
        `${result[2]}`,
      );
    }

    return new Result(false, `${result[2]}`);
  };

  /**
   * Kill a node given a name
   *
   * @param {string} name - Node to be killed
   * @return {Promise<Result>} Returns a result
   */
  public screenKillNode: (name: string) => Promise<Result> = async (name) => {
    const result = await this.makeCall(
      URI.ROS_SCREEN_KILL_NODE,
      [name],
      true,
      true,
    );
    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      if (parsed.result) {
        return parsed;
      }
      if (this.logger) {
        this.logger.error(
          `Fail to kill the node [${name}]: ${parsed.message}`,
          parsed.message,
          false,
        );
        return parsed;
      }
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at screenKillNode()`,
        `${result[2]}`,
      );
    }

    return new Result(
      false,
      `Provider [${this.name()}]: Error at screenKillNode(): ${result[2]}`,
    );
  };

  /**
   * Get list of available screens 'ros.screen.get_list'
   *
   * @return {Promise<ScreensMapping[]>} Returns a list of screens
   */
  private getScreenList: () => Promise<ScreensMapping[] | null> = async () => {
    const result = await this.makeCall(URI.ROS_SCREEN_GET_LIST, [], true, true);
    if (result[0]) {
      const screenList: ScreensMapping[] = [];
      JSON.parse(result[1]).forEach((p: ScreensMapping) => {
        screenList.push(new ScreensMapping(p.name, p.screens));
      });
      return Promise.resolve(screenList);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getScreenList()`,
        `${result[2]}`,
      );
    }
    return Promise.resolve(null);
  };

  /**
   * Get list of diagnostics
   *
   * @return {Promise<DiagnosticArray>}
   */
  private getDiagnostics: () => Promise<DiagnosticArray | null> = async () => {
    const result = await this.makeCall(
      URI.ROS_PROVIDER_GET_DIAGNOSTICS,
      [],
      true,
      true,
    );
    if (result[0]) {
      const diagnostics: DiagnosticArray = JSON.parse(result[1]);
      return Promise.resolve(diagnostics);
    }

    if (this.logger) {
      this.logger.error(
        `Provider [${this.name()}]: Error at getDiagnostics()`,
        `${result[2]}`,
      );
    }
    return Promise.resolve(null);
  };

  /**
   * Unregister a node given a name
   *
   * @param {string} name - Node to be stopped
   * @return {Promise<Result>} Returns a result
   */
  public unregisterNode: (name: string) => Promise<Result> = async (name) => {
    const result = await this.makeCall(
      URI.ROS_NODES_UNREGISTER,
      [name],
      true,
      true,
    );
    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      if (parsed.result) {
        return parsed;
      }
      if (this.logger) {
        // use debug because of spamming messages when nodes does not run
        this.logger.debug(`${parsed.message}`, parsed.message);
      }
      return parsed;
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at unregisterNode()`,
        `${result[2]}`,
      );
    }

    return new Result(
      false,
      `Provider [${this.name()}]: Error at unregisterNode(): ${result[2]}`,
    );
  };

  /* Parameters */

  /**
   * Return a list with all registered parameters values and types
   *
   * @return {Promise<Result>} Returns a result
   */
  public getParameterList: () => Promise<RosParameter[]> = async () => {
    const result = await this.makeCall(
      'ros.parameters.get_list',
      [],
      true,
      true,
    );
    const paramList: RosParameter[] = [];

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      parsed.forEach((item: { name: string; value: string; type: string }) => {
        paramList.push(
          new RosParameter(item.name, item.value, item.type, this.id),
        );
      });
      return paramList;
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at getParameterList()`,
        `${result[2]}`,
      );
    }

    return Promise.resolve([]);
  };

  /**
   * Return a list with all registered parameters values and types for a list of nodes
   *
   * @param {string[]} nodes - List of node names
   * @return {Promise<Result>} Returns a result
   */
  public getNodeParameters: (nodes: string[]) => Promise<RosParameter[]> =
    async (nodes) => {
      const result = await this.makeCall(
        URI.ROS_PARAMETERS_GET_NODE_PARAMETERS,
        [nodes],
        true,
        true,
      );

      const paramList: RosParameter[] = [];

      if (result[0]) {
        const parsed = JSON.parse(result[1]);
        parsed.forEach(
          (item: { name: string; value: string; type: string }) => {
            paramList.push(
              new RosParameter(item.name, item.value, item.type, this.id),
            );
          },
        );
        return paramList;
      }

      if (this.logger) {
        this.logger.debug(
          `Provider [${this.name()}]: Error at stopNode()`,
          `${result[2]}`,
        );
      }

      return Promise.resolve([]);
    };

  /**
   * Set the value of a parameter
   *
   * @param {RosParameter} parameter
   * @return {Promise<Boolean>} Returns true if success
   */
  public setParameter: (parameter: RosParameter) => Promise<boolean> = async (
    parameter,
  ) => {
    const result = await this.makeCall(
      URI.ROS_PARAMETERS_SET_PARAMETER,
      [parameter],
      true,
      true,
    );

    if (result[0]) {
      const parsed = JSON.parse(result[1]);
      return parsed;
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at setParameter()`,
        `${result[2]}`,
      );
    }

    return false;
  };

  /**
   * Check if a parameter exists
   *
   * @param {string} parameterName
   * @return {Promise<Boolean>} Returns true if success
   */
  public hasParameter: (parameterName: string) => Promise<boolean> = async (
    parameterName,
  ) => {
    const result = await this.makeCall(
      URI.ROS_PARAMETERS_HAS_PARAMETER,
      [parameterName],
      true,
      true,
    );

    if (result[0]) {
      return JSON.parse(result[1]);
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at hasParameter()`,
        `${result[2]}`,
      );
    }

    return false;
  };

  /**
   * Delete a list of parameters
   *
   * @param {string[]} parameters list of parameters to be deleted
   * @return {Promise<Boolean>} Returns true if success
   */
  public deleteParameters: (parameters: string[]) => Promise<boolean> = async (
    parameters,
  ) => {
    const result = await this.makeCall(
      URI.ROS_PARAMETERS_DELETE_PARAMETERS,
      [parameters],
      true,
      true,
    );

    if (result[0]) {
      return JSON.parse(result[1]);
    }

    if (this.logger) {
      this.logger.debug(
        `Provider [${this.name()}]: Error at setParameter()`,
        `${result[2]}`,
      );
    }

    return false;
  };

  /* Publisher handler */

  /**
   * Callback of master daemon ready status (true/false)
   */
  private callbackDaemonReady: (msgs: string[]) => void = (msgs) => {
    const msg = msgs[0];
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_DAEMON_READY, msg, '', this.id);

    if (typeof msg === 'string') {
      try {
        this.daemon = JSON.parse(msg).status;
      } catch (error) {
        if (this.logger)
          this.logger.error(
            `[callbackDaemonReady] Could not parse message ${msg}`,
            `Error: ${error}`,
          );
      }
    }
  };

  /**
   * Callback of master discovery ready status (true/false)
   */
  private callbackDiscoveryReady: (msgs: string[]) => void = (msgs) => {
    const msg = msgs[0];
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_DISCOVERY_READY, msg, '', this.id);

    if (typeof msg === 'string') {
      try {
        this.discovery = JSON.parse(msg).status;
      } catch (error) {
        if (this.logger)
          this.logger.error(
            `[callbackDiscoveryReady] Could not parse message ${msg}`,
            `Error: ${error}`,
          );
      }
    }
  };

  /**
   * Callback when any launch file or ROS nodes changes  in provider
   */
  public updateRosNodes: () => void = async () => {
    if (this.logger)
      this.logger.debug(`Trigger update ros nodes for ${this.id}`, '');
    if (await this.lockRequest('updateRosNodes')) {
      return;
    }

    // get nodes from remote provider
    const nlUnfiltered = await this.getNodeList();
    const nl = nlUnfiltered.filter((n) => {
      let ignored = false;

      // exclude nodes belonging to a different provider
      if (
        (this.rosState.masteruri && n.masteruri !== this.rosState.masteruri) ||
        (n.location instanceof String && n.location === 'remote') ||
        (n.location instanceof Array &&
          !n.location.some(
            (loc) =>
              loc.startsWith('SHM') || loc.startsWith('UDPv4:[127.0.0.1]'),
          ))
      ) {
        ignored = true;
      }

      // exclude ignored nodes
      this.IGNORED_NODES.forEach((ignoredNode) => {
        if (n.name.indexOf(ignoredNode) !== -1) ignored = true;
      });
      return !ignored;
    });

    // check if nodes are not available or run in other host (not monitoring)
    nl.forEach((n) => {
      // idGlobal should be the same for life of the node on remote host
      n.idGlobal = `${this.id}${n.id.replaceAll('/', '.')}`;
      n.providerName = this.name();
      n.providerId = this.id;

      if (!n.node_API_URI || n.node_API_URI.length === 0) return;
      if (!n.masteruri || n.masteruri.length === 0) return;

      if (!n.pid || n.pid <= 0) {
        const hostApiUir = n.node_API_URI.split(':')[0];
        const hostMasterUri = n.masteruri.split(':')[0];

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
      // copy (selected) old state
      const oldNode = this.rosNodes.find(
        (item) => item.idGlobal === n.idGlobal,
      );
      if (oldNode) {
        n.diagnosticStatus = oldNode.diagnosticStatus;
        n.diagnosticLevel = oldNode.diagnosticLevel;
        n.diagnosticMessage = oldNode.diagnosticMessage;
        n.launchPaths = oldNode.launchPaths;
        n.launchPath = oldNode.launchPath;
        n.group = oldNode.group;
        n.launchInfo = oldNode.launchInfo;
      }
    });
    // do not sort nodes -> start order defined by capability_group
    // nl.sort(compareRosNodes);

    this.rosNodes = nl;
    // emitCustomEvent(
    //   EVENT_PROVIDER_ROS_NODES,
    //   new EventProviderRosNodes(this, nl),
    // );
    this.updateLaunchContent();
    this.unlockRequest('updateRosNodes');
  };

  private callbackChangedFile: (msg: string) => void = async (msg) => {
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_PATH_CHANGED, msg, '', this.id);
    if (!msg || (await this.lockRequest('callbackChangedFile'))) {
      return;
    }

    const msgParsed = JSON.parse(msg);
    emitCustomEvent(
      EVENT_PROVIDER_PATH_EVENT,
      new EventProviderPathEvent(this, msgParsed),
    );
    this.unlockRequest('callbackChangedFile');
  };

  /**
   * Update the screen state of each node reported in the list of ScreensMapping.
   */
  private callbackScreensUpdate: (msg: string) => void = async (msg) => {
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_SCREEN_LIST, msg, '', this.id);
    if (!msg) {
      return;
    }

    const msgParsed: ScreensMapping[] = JSON.parse(msg);
    this.updateScreens(msgParsed);
  };

  /**
   * Update diagnostics of each node reported in the list of DiagnosticsArray.
   */
  private callbackDiagnosticsUpdate: (msg: string) => void = async (msg) => {
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_PROVIDER_DIAGNOSTICS, msg, '', this.id);
    if (!msg) {
      return;
    }

    const msgParsed: DiagnosticArray = JSON.parse(msg);
    this.updateDiagnostics(msgParsed);
  };

  /**
   * Update the screen state of each node reported in the list of ScreensMapping.
   */
  private callbackProviderWarnings: (msg: string) => void = async (msg) => {
    if (this.logger)
      this.logger.debugCrossbar(URI.ROS_PROVIDER_WARNINGS, msg, '', this.id);
    if (!msg || (await this.lockRequest('callbackProviderWarnings'))) {
      return;
    }

    const msgParsed: SystemWarningGroup[] = JSON.parse(msg);
    this.warnings = msgParsed;
    emitCustomEvent(
      EVENT_PROVIDER_WARNINGS,
      new EventProviderWarnings(this, msgParsed),
    );
    this.unlockRequest('callbackProviderWarnings');
  };

  private registerCallbacks: () => Promise<any> = async () => {
    if (!this.isAvailable()) {
      return;
    }
    await this.crossbar.closeSubscriptions();
    await this.crossbar.closeRegistrations();

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_PROVIDER_LIST
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(
      URI.ROS_PROVIDER_LIST,
      this.updateProviderList,
    );

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_DAEMON_READY
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(
      URI.ROS_DAEMON_READY,
      this.callbackDaemonReady,
    );

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_DISCOVERY_READY
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(
      URI.ROS_DISCOVERY_READY,
      this.callbackDiscoveryReady,
    );

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_LAUNCH_CHANGED
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(URI.ROS_LAUNCH_CHANGED, this.updateRosNodes);

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_NODES_CHANGED
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(URI.ROS_NODES_CHANGED, this.updateRosNodes);

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_PATH_CHANGED
        }]`,
        '',
        false,
      );
    this.crossbar.subscribe(URI.ROS_PATH_CHANGED, this.callbackChangedFile);

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_SCREEN_LIST
        }]`,
        '',
        false,
      );
    await this.crossbar.subscribe(
      URI.ROS_SCREEN_LIST,
      this.callbackScreensUpdate,
    );

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_PROVIDER_WARNINGS
        }]`,
        '',
        false,
      );

    await this.crossbar.subscribe(
      URI.ROS_PROVIDER_WARNINGS,
      this.callbackProviderWarnings,
    );

    if (this.logger)
      this.logger.debug(
        `CROSSBAR_IO: (${this.name()}) Subscribing to: [${
          URI.ROS_PROVIDER_DIAGNOSTICS
        }]`,
        '',
        false,
      );

    await this.crossbar.subscribe(
      URI.ROS_PROVIDER_DIAGNOSTICS,
      this.callbackDiagnosticsUpdate,
    );
  };

  /* Generic functions */

  /**
   * Execute a call considering [callAttempts] and [delayCallAttempts]
   *
   * @param {string} uri Crossbar URI to be called
   * @param {any} args call arguments
   * @return {Promise<any>} Return promise of the call
   */
  private makeCall: (
    uri: string,
    args: any,
    lockRequest: boolean,
    reportError: boolean,
  ) => Promise<any> = async (
    uri,
    args,
    lockRequest = true,
    reportError = true,
  ) => {
    if (!this.isAvailable()) {
      return [
        false,
        '',
        `[${this.name()}]: Ignoring request to: [${uri}], provider not yet connected!`,
      ];
    }

    if (lockRequest && (await this.lockRequest(uri))) {
      return [
        false,
        '',
        `[${this.name()}]: Ignoring request to: [${uri}] with ${JSON.stringify(
          args,
        )}, request already running!`,
      ];
    }

    const callRequest: (
      _uri: string,
      _args: any,
      _reportError: boolean,
      currentAttempt?: number,
    ) => any = async (
      _uri: string,
      _args: any,
      _reportError = true,
      currentAttempt = 0,
    ) => {
      try {
        const r = await this.crossbar.call(_uri, _args);
        if (r[2]) {
          const errorObj = JSON.parse(r[2]);
          if (errorObj.error.includes('wamp.error.runtime_error')) {
            // do not re-try on runtime errors
            this.unlockRequest(uri);
            return [r[0], r[1], errorObj.args];
          }
        }
        if (r[0]) {
          // it works, stop loop
          this.unlockRequest(uri);
          return r;
        }

        if (currentAttempt >= this.callAttempts) {
          this.unlockRequest(uri);
          return [
            false,
            '',
            `[CROSSBAR] (${uri}) [${this.name()}]: Max call attempts (${
              this.callAttempts
            }) reached!`,
          ];
        }

        // didn't work, wait and repeat
        if (this.logger) {
          if (_reportError) {
            this.logger.error(
              `[CROSSBAR] (${uri}) [${this.name()}]`,
              r[2] as string,
              false,
            );
          }

          this.logger.info(
            `[CROSSBAR] (${uri}) [${this.name()}]`,
            `Waiting (${this.delayCallAttempts} ms) Attempt: [${currentAttempt}/${this.callAttempts}]`,
            false,
          );
        }

        await delay(this.delayCallAttempts);
        return callRequest(_uri, _args, _reportError, currentAttempt + 1);
      } catch (err) {
        return callRequest(_uri, _args, _reportError, currentAttempt + 1);
      }
    };

    const result = await callRequest(uri, args, reportError);
    // const result = await this.crossbar.call(uri, args);

    if (this.logger) {
      this.logger.debugCrossbar(uri, result[1], result[2], this.name());
    }

    return result;
  };

  private onCloseCrossbar = (reason: string, details: string) => {
    if (this.crossbar.wsUnreachable)
      this.setConnectionState(ConnectionState.STATES.UNREACHABLE, details);
    else this.setConnectionState(reason, details);
  };

  private onOpenCrossbar = () => {
    // connected state is set after all crossbar connections are registered
    this.daemon = false;
    this.discovery = false;
    this.setConnectionState(ConnectionState.STATES.CROSSBAR_CONNECTED, '');
  };

  /*
  Prevents multiple calls to the same [request]
  Return true when [request has been already locked]
  */
  private lockRequest: (request: string) => Promise<boolean> = async (
    request,
  ) => {
    if (this.currentRequestList.has(request)) {
      if (this.logger) {
        this.logger.debug(
          `[${this.name()}]: Wait 1 sec for release request to: [${request}]`,
          '',
          false,
        );
      }
      // TODO: wait until release?
      await delay(1000);
      if (this.currentRequestList.has(request)) {
        if (this.logger) {
          this.logger.debug(
            `[${this.name()}]: Ignoring request to: [${request}]`,
            '',
            false,
          );
        }
        return Promise.resolve(true);
      }
    }
    emitCustomEvent(
      EVENT_PROVIDER_ACTIVITY,
      new EventProviderActivity(this, true, request),
    );
    this.currentRequestList.add(request);
    return Promise.resolve(false);
  };

  /*
  Remove lock for a given [request]
  */
  private unlockRequest = (request: string) => {
    this.currentRequestList.delete(request);
    if (this.currentRequestList.size === 0) {
      emitCustomEvent(
        EVENT_PROVIDER_ACTIVITY,
        new EventProviderActivity(this, false, ''),
      );
    }
  };
}

export default CrossbarIOProvider;
