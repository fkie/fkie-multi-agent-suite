import { useDebounceCallback } from '@react-hook/debounce';
import { SnackbarKey, useSnackbar } from 'notistack';
import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
} from 'react';

import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';
import {
  EVENT_PROVIDER_DISCOVERED,
  EVENT_PROVIDER_PATH_EVENT,
  EVENT_PROVIDER_ROS_NODES,
  EVENT_PROVIDER_SCREENS,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
  EventProviderDiscovered,
  EventProviderPathEvent,
  EventProviderRosNodes,
  EventProviderScreens,
  EventProviderState,
  EventProviderWarnings,
} from '../providers/events';

import { ConnectionState, CrossbarIOProvider } from '../providers';

import MultimasterManager from '../../main/IPC/MultimasterManager';
import { IROSInfo, ROSInfo } from '../../main/IPC/ROSInfo';
import { ISystemInfo, SystemInfo } from '../../main/IPC/SystemInfo';
import {
  ReloadFileAlertComponent,
  RestartNodesAlertComponent,
} from '../components/UI';
import useLocalStorage from '../hooks/useLocalStorage';
import {
  LaunchArgument,
  LaunchLoadRequest,
  PATH_EVENT_TYPE,
  ProviderLaunchConfiguration,
  RosNode,
  SubscriberFilter,
  SubscriberNode,
  getFileName,
} from '../models';
import { delay } from '../utils';
import { LoggingContext } from './LoggingContext';
import { SSHContext } from './SSHContext';
import { LAUNCH_FILE_EXTENSIONS, SettingsContext } from './SettingsContext';

declare global {
  interface Window {
    ROSInfo?: ROSInfo;
    SystemInfo?: SystemInfo;
    MultimasterManager?: MultimasterManager;
  }
}

export interface IRosProviderContext {
  initialized: boolean;
  rosInfo: IROSInfo | null;
  systemInfo: ISystemInfo | null;
  multimasterManager: MultimasterManager | null;
  providers: CrossbarIOProvider[];
  providersConnected: CrossbarIOProvider[];
  providersConnectedPast: CrossbarIOProvider[];
  mapProviderRosNodes: Map<string, RosNode[]>;
  connectToProvider: (provider: CrossbarIOProvider) => Promise<boolean>;
  startProvider: (
    provider: CrossbarIOProvider,
    forceStartWithDefault: boolean,
  ) => Promise<boolean>;
  startConfig: (config: ProviderLaunchConfiguration) => Promise<boolean>;

  getProviderName: (providerId: string) => string;
  getProviderLaunchConfig: (
    providerId: string,
  ) => ProviderLaunchConfiguration | undefined;
  refreshProviderList: () => void;
  closeProviders: () => void;
  updateNodeList: (providerId: string) => void;
  updateLaunchList: (providerId: string) => void;
  reloadLaunchFile: (providerId: string, modifiedFile: string) => Promise<void>;
  getProviderById: (
    providerId: string,
    includeNotAvailable: boolean,
  ) => CrossbarIOProvider | undefined;
  getProviderByHost: (hostName: string) => CrossbarIOProvider | null;
  registerSubscriber: (
    providerId: string,
    topic: string,
    messageType: string,
    filter: SubscriberFilter,
  ) => void;
  unregisterSubscriber: (providerId: string, topic: string) => void;
  updateFilterRosTopic: (
    provider: CrossbarIOProvider,
    topicName: string,
    msg: SubscriberFilter,
  ) => void;
  isLocalHost: (host: string) => void;
  saveProviderConfig: (cfg: ProviderLaunchConfiguration) => void;
  deleteProviderConfig: (providerId: string) => void;
}

export const DEFAULT = {
  initialized: false,
  rosInfo: null,
  systemInfo: null,
  multimasterManager: null,
  providers: [],
  providersConnected: [],
  providersConnectedPast: [],
  mapProviderRosNodes: new Map(), // Map<providerId: string, nodes: RosNode[]>
  connectToProvider: () => new Promise<false>(() => {}),
  startProvider: () => new Promise<boolean>(() => {}),
  startConfig: () => new Promise<boolean>(() => {}),

  getProviderName: () => '',
  getProviderLaunchConfig: () => undefined,
  refreshProviderList: () => {},
  closeProviders: () => {},
  updateNodeList: () => {},
  reloadLaunchFile: () => new Promise<void>(() => {}),
  updateLaunchList: () => {},
  getProviderById: () => undefined,
  getProviderByHost: () => null,
  registerSubscriber: () => null,
  unregisterSubscriber: () => null,
  updateFilterRosTopic: () => null,
  isLocalHost: () => null,
  saveProviderConfig: () => null,
  deleteProviderConfig: () => null,
};

interface IRosProviderComponent {
  children: React.ReactNode;
}

// (ms) time to debounce callbacks
// useful to prevent unnecessary updates
const debounceTimeout = 100;

export const RosContext = createContext<IRosProviderContext>(DEFAULT);

export function RosProviderReact(
  props: IRosProviderComponent,
): ReturnType<React.FC<IRosProviderComponent>> {
  const { children } = props;
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const SSHCtx = useContext(SSHContext);
  const { enqueueSnackbar } = useSnackbar();

  const [initialized, setInitialized] = useState(DEFAULT.initialized);
  const [initializedSystem, setInitializedSystem] = useState(false);
  const [initializedProviders, setInitializedProviders] = useState(false);
  const [rosInfo, setRosInfo] = useState<IROSInfo | null>(null);
  const [systemInfo, setSystemInfo] = useState<ISystemInfo | null>(null);
  const [multimasterManager, setMultimasterManager] =
    useState<MultimasterManager | null>(null);
  const [providersAddQueue, setProvidersAddQueue] = useState<
    CrossbarIOProvider[]
  >([]);
  const [providers, setProviders] = useState<CrossbarIOProvider[]>([]);
  const [providersConnected, setProvidersConnected] = useState<
    CrossbarIOProvider[]
  >([]);
  const [providersConnectedPast, setProvidersConnectedPast] = useState<
    CrossbarIOProvider[]
  >([]);
  const [providerLaunches, setProviderLaunches] = useLocalStorage<
    ProviderLaunchConfiguration[]
  >('Providers:providerLaunches', []);
  const [providerLaunchesMap, setProviderLaunchesMap] = useState<
    Map<string, ProviderLaunchConfiguration>
  >(new Map());

  const [mapProviderRosNodes, setMapProviderRosNodes] = useState(
    DEFAULT.mapProviderRosNodes,
  );

  /** Save configuration which are loaded into provider panel. */
  const saveProviderConfig = useCallback(
    (cfg: ProviderLaunchConfiguration) => {
      setProviderLaunches([
        ...providerLaunches.filter((prov) => {
          return prov.host !== cfg.host || prov.port !== cfg.port;
        }),
        cfg,
      ]);
    },
    [providerLaunches, setProviderLaunches],
  );

  /** Delete start configuration and remove provider for given provider ID */
  const deleteProviderConfig = useCallback(
    (providerId: string) => {
      // get associated configuration with given provider ID
      const provLaunch = providerLaunchesMap.get(providerId);
      if (provLaunch) {
        // remove configuration
        setProviderLaunches(
          providerLaunches.filter((item) => {
            return !(
              item.host === provLaunch.host && item.port === provLaunch.port
            );
          }),
        );
        providerLaunchesMap.delete(providerId);
      }
      // delete provider
      setProviders((currProv) => [
        ...currProv.filter((item) => {
          if (item.id === providerId) {
            item.close();
            return false;
          }
          return true;
        }),
      ]);
      // remove connected provider
      setProvidersConnected((currProv) => [
        ...currProv.filter((item) => item.id !== providerId),
      ]);
      // remove provider from past connected list
      setProvidersConnectedPast((currProv) => [
        ...currProv.filter((item) => item.id !== providerId),
      ]);
    },
    [
      providerLaunches,
      providerLaunchesMap,
      setProviders,
      setProviderLaunches,
      setProvidersConnected,
      setProvidersConnectedPast,
    ],
  );

  /** Returns true if given host is one of local IPv4 addresses and local host names.  */
  function isLocalHost(host: string) {
    if (host === 'localhost' || host === systemInfo?.osInfo?.hostname) {
      return true;
    }
    const localIps = [];
    systemInfo?.networkInterfaces?.forEach((ni) => {
      if (host === ni.ip4) {
        localIps.push(ni.ip4);
      }
    });
    return localIps.length > 0;
  }

  /** Search and return a provider using its id */
  const getProviderById = useCallback(
    (providerId: string, includeNotAvailable: boolean = true) => {
      return providers.find((provider) => {
        return (
          (provider.isAvailable() || includeNotAvailable) &&
          provider.id === providerId
        );
      });
    },
    [providers],
  );

  const getProviderLaunchConfig = (providerId: string) => {
    return providerLaunchesMap.get(providerId);
  };

  /**
   * Search and return a provider using its host name
   */
  const getProviderByHost = useCallback(
    (hostName: string) => {
      if (initialized) {
        const p = providers.find((provider) => {
          return (
            provider.isAvailable() &&
            (provider.crossbar.host === hostName ||
              provider.hostnames.includes(hostName))
          );
        });

        if (p) return p;
      }

      return null;
    },
    [initialized, providers],
  );

  /**
   * Search and return a provider using known hosts and port
   */
  const getProviderByHosts = useCallback(
    (
      hosts: string[],
      port: number = 0,
      defaultValue: CrossbarIOProvider | null = null,
    ) => {
      const p = providers.find((provider) => {
        let result = true;
        if (port !== 0 && provider.crossbar.port !== 0) {
          result = provider.crossbar.port === port;
        }
        if (!result) return false;
        // join hostnames without duplicates
        result =
          hosts.filter((value) => {
            return provider.hostnames?.includes(value);
          }).length > 0;
        return result;
      });
      if (p) {
        return p;
      }
      if (defaultValue !== null) {
        // add provider
        setProvidersAddQueue((oldValues) => [...oldValues, defaultValue]);
      }
      return defaultValue;
    },
    [providers],
  );

  const getProviderLaunchConfigByHost = useCallback(
    (host: string, port: number) => {
      const result: ProviderLaunchConfiguration[] = providerLaunches.filter(
        (cfg: ProviderLaunchConfiguration) => {
          return (
            cfg.host === host &&
            ((port === cfg.port || port) === 0 || cfg.port === 0)
          );
        },
      );
      if (result.length === 1) {
        return result[0];
      }
      return undefined;
    },
    [providerLaunches],
  );

  /**
   * Trigger updateRosNodes() of the provider.
   */
  const updateNodeList = useCallback(
    async (providerId: string) => {
      logCtx.debug(
        `Triggering update of ROS nodes from ${providerId}`,
        '',
        false,
      );
      const provider = getProviderById(providerId);
      await provider?.updateRosNodes();
    },
    [getProviderById, logCtx],
  );

  /**
   * Trigger updateLaunchContent() of the provider.
   */
  const updateLaunchList = useCallback(
    async (providerId: string) => {
      logCtx.debug(
        `Triggering update of ROS launch files from ${providerId}`,
        '',
      );
      const provider = getProviderById(providerId);
      await provider?.updateLaunchContent();
    },
    [getProviderById, logCtx],
  );

  /**
   * Close the connection to all registered providers
   * Useful when new providers wants to be registered
   * as it prevents re-registration of URIs
   */
  const closeProviders = useCallback(() => {
    providers.forEach((prov) => {
      console.log(`closeProviders: Closing provider [${prov.name()}]`);
      prov.close();
    });
  }, [providers]);

  /**
   * Launch a file [filePath] on a given [provider], using arguments [args]
   */
  const launchFile = useCallback(
    async (
      provider: CrossbarIOProvider,
      filePath: string,
      args: LaunchArgument[],
      reload = false,
    ) => {
      if (!filePath) return { success: false, error: `Invalid file path` };
      if (!provider || !provider.isAvailable())
        return { success: false, error: `Invalid/unavailable provider` };

      /*
       * ros_package -
       * launch - Launch file in the package path.
       * path - if set, this will be used instead of package/launch
       * @param {LaunchArgument[]} args - Arguments to load the launch file.
       * @param {boolean} force_first_file - If True, use first file if more than one was found in the package.
       * @param {boolean} request_args - If True, the launch file will not be loaded, only launch arguments are requested.
       * masteruri - Starts nodes of this file with specified ROS_MASTER_URI.
       * host - Start nodes of this file on specified host.
       * */
      const rosPackage = ''; // ROS package name.
      const launch = ''; // Launch file in the package path.
      const path = filePath;
      const forceFirstFile = true;
      const requestArgs = false;

      const request = new LaunchLoadRequest(
        rosPackage,
        launch,
        path,
        args,
        forceFirstFile,
        requestArgs,
        `${provider.rosState.masteruri ? provider.rosState.masteruri : ''}`,
        provider.crossbar.host,
      );

      console.log(`LOAD LAUNCH: ${JSON.stringify(request)}`);
      const resultLaunchLoadFile = await provider.launchLoadFile(
        request,
        reload,
      );

      if (!resultLaunchLoadFile) {
        return {
          success: false,
          error: `Invalid response for [launchLoadFile], check DAEMON screen output`,
          reply: null,
        };
      }

      if (resultLaunchLoadFile.status.code === 'OK') {
        // trigger node's update (will force a reload using useEffect hook)
        updateNodeList(provider.id);
        // updateLaunchList(provider.name());
        return { success: true, error: '', reply: resultLaunchLoadFile };
      }

      if (resultLaunchLoadFile.status.code === 'PARAMS_REQUIRED') {
        return {
          success: false,
          error: 'Please fill all arguments',
          reply: resultLaunchLoadFile,
        };
      }

      if (resultLaunchLoadFile.status.code === 'ERROR') {
        return {
          success: false,
          error: `Reported error: ${resultLaunchLoadFile.status.msg}`,
          reply: resultLaunchLoadFile,
        };
      }

      return {
        success: false,
        error: `Could not load file: ${resultLaunchLoadFile.status.msg}`,
        reply: resultLaunchLoadFile,
      };
    },
    [
      updateNodeList,
      // updateLaunchList
    ],
  );

  function createRestartNodesAlertComponent(
    key: SnackbarKey | undefined,
    message: string,
    provider: CrossbarIOProvider,
    changedNodes: string[],
    modifiedFile: string,
  ) {
    return (
      <RestartNodesAlertComponent
        id={key}
        message={message}
        provider={provider}
        nodeList={changedNodes}
        onReload={(p, nodeList) => {
          Promise.all(
            nodeList.map(async (nodeName: string) => {
              const node = new RosNode();
              node.providerId = p;
              node.name = nodeName;
              node.id = nodeName;
              node.launchPath = modifiedFile;

              await provider.stopNode(node.id);
              const resultStartNode = await provider.startNode(node);

              if (resultStartNode.success) {
                logCtx.success(
                  resultStartNode.message,
                  resultStartNode.details,
                );
              } else {
                logCtx.error(resultStartNode.message, resultStartNode.details);
              }

              return resultStartNode;
            }),
          ).catch((error) => {
            logCtx.debug(`Error restarting nodes: ${error}`, '');
          });
        }}
      />
    );
  }

  /**
   * Reload launch files: First unload file and then load it again with last arguments provided
   */
  const reloadLaunchFile = useCallback(
    async (providerId: string, modifiedFile: string) => {
      // If valid launch file extension
      if (
        LAUNCH_FILE_EXTENSIONS.find((fe) => modifiedFile.indexOf(fe) !== -1)
      ) {
        const provider = getProviderById(providerId) as CrossbarIOProvider;
        if (!provider) return;

        // Launch file again using last arguments

        // get args from launch file saved on provider object
        let args: LaunchArgument[] = [];
        if (provider.launchFiles) {
          const filteredLaunchFile = provider.launchFiles.find(
            (lf) => lf.path === modifiedFile,
          );
          if (filteredLaunchFile) {
            args = filteredLaunchFile.args;
          }
        }

        const result = await launchFile(provider, modifiedFile, args, true);
        if (result && result.success) {
          logCtx.success(
            `Launch file [${getFileName(modifiedFile)}] reloaded`,
            `File: ${modifiedFile}`,
          );

          // check if nodes have to be restarted
          if (
            result.reply &&
            result.reply.changed_nodes &&
            result.reply.changed_nodes.length > 0
          ) {
            // ask use if nodes should be restarted
            enqueueSnackbar(`Do you want to restart nodes on: `, {
              persist: true,
              anchorOrigin: {
                vertical: 'top',
                horizontal: 'right',
              },
              preventDuplicate: true,
              content: (key, message) =>
                createRestartNodesAlertComponent(
                  key,
                  `${message}`,
                  provider,
                  result.reply.changed_nodes,
                  modifiedFile,
                ),
            });
          }
        }
        if (result && !result.success) {
          logCtx.error(
            `Launch file error: ${result.error}`,
            `Provider: ${provider.name()} File: ${modifiedFile}`,
          );
        }
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [getProviderById, launchFile],
  );

  /**
   * Forces an update on the provider list for all connected provider.
   */
  const refreshProviderList = useDebounceCallback(() => {
    providersConnected.forEach((provider) => {
      provider.updateProviderList();
    });
  }, debounceTimeout);

  const startConfig: (config: ProviderLaunchConfiguration) => Promise<boolean> =
    useCallback(
      async (config) => {
        if (!multimasterManager) return false;

        let allStarted = true;
        try {
          const isLocal = isLocalHost(config.host);
          const credential = isLocal
            ? null
            : SSHCtx.getCredentialHost(config.host);

          // check and add provider if new
          let provider = getProviderByHosts(
            [config.host],
            config.port,
            null,
          ) as CrossbarIOProvider;
          if (!provider) {
            provider = new CrossbarIOProvider(
              settingsCtx,
              config.host,
              config.rosVersion,
              config.port,
              config.useSSL,
              logCtx,
            );
            provider.isLocalHost = isLocal;
            // add provider using add queue
            setProvidersAddQueue((oldValues) => [...oldValues, provider]);
            return false;
          }

          if (!isLocal && !credential) {
            // it is possible to start the nodes in parallel (using Promises.all) but it is not advisable.
            // A problem arises when the ROS master node is not running, and a race condition between
            // daemon and discovery appears trying to spawn the master node.
            // instead, just start each process in a sequence

            // check for credentials and throw an exception
            provider.setConnectionState(
              ConnectionState.STATES.NO_SSH_CREDENTIALS,
              '',
            );
            allStarted = false;
            throw new Error('Missing SSH credentials');
          }

          if (
            !(
              config.daemon.enable ||
              config.discovery.enable ||
              config.terminal.enable
            )
          ) {
            // use default configuration if no one is configured to start
            config.daemon.enable = true;
            config.discovery.enable = true;
            config.terminal.enable = true;
          }

          // Find running system nodes
          const systemNodes = provider.rosNodes.filter((n) => n.system_node);

          // Restart Daemon
          if (config.daemon.enable) {
            if (config.forceRestart) {
              const daemonNode = systemNodes.find(
                (n) =>
                  n.id.includes('mas_daemon') ||
                  n.id.includes('node_manager_daemon'),
              );
              if (daemonNode && provider) {
                logCtx.debug(
                  `Stopping running NodeManager-Daemon on host ${config.host}`,
                  '',
                );
                await provider.stopNode(daemonNode.id);
                // wait a little bit until the ros node is unregistered
                await delay(2000);
              }
            }
            logCtx.debug(
              `Starting NodeManager-Daemon on host '${config.host}'`,
              '',
            );
            const resultStartDaemon =
              await multimasterManager.startMultimasterDaemon(
                config.rosVersion,
                credential,
              );
            if (!resultStartDaemon.result) {
              logCtx.error(
                `Failed to start daemon on host '${config.host}'`,
                resultStartDaemon.message,
              );
              provider.setConnectionState(
                ConnectionState.STATES.UNREACHABLE,
                resultStartDaemon.message,
              );
              allStarted = false;
              return false;
            }
          }
          // Restart Discovery
          if (config.discovery.enable) {
            if (config.forceRestart) {
              const discoveryNode = systemNodes.find(
                (n) =>
                  n.id.includes('mas_discovery') ||
                  n.id.includes('master_discovery'),
              );
              if (discoveryNode && provider) {
                logCtx.debug(
                  `Stopping running Multimaster-Discovery on host '${config.host}'`,
                  '',
                );
                await provider.stopNode(discoveryNode.id);
                // wait a little bit until the ros node is unregistered
                await delay(2000);
              }
            }
            logCtx.debug(
              `Starting Multimaster-Discovery on host '${config.host}'`,
              '',
            );
            const resultStartDiscovery =
              await multimasterManager.startMasterDiscovery(
                config.rosVersion,
                credential,
                undefined,
                config.discovery.networkId,
                undefined,
                undefined,
                config.discovery.robotHosts,
              );
            if (!resultStartDiscovery.result) {
              logCtx.error(
                `Failed to start discovery on host '${config.host}'`,
                resultStartDiscovery.message,
              );
              provider.setConnectionState(
                ConnectionState.STATES.UNREACHABLE,
                resultStartDiscovery.message,
              );
              allStarted = false;
              return false;
            }
          }

          // Restart Sync
          if (config.sync.enable) {
            if (config.forceRestart) {
              const syncNode = systemNodes.find(
                (n) =>
                  n.id.includes('mas_sync') || n.id.includes('master_sync'),
              );
              if (syncNode && provider) {
                logCtx.debug(
                  `Stopping running Multimaster-Sync on host '${config.host}'`,
                  '',
                );
                await provider.stopNode(syncNode.id);
                // wait a little bit until the ros node is unregistered
                await delay(2000);
              }
            }
            logCtx.debug(
              `Starting Multimaster-Sync on host '${config.host}'`,
              '',
            );
            const resultStartSync = await multimasterManager.startMasterSync(
              config.rosVersion,
              credential,
              undefined,
              config.sync.doNotSync,
              config.sync.syncTopics,
            );
            if (!resultStartSync.result) {
              logCtx.warn(
                `Failed to start sync on host '${config.host}'`,
                resultStartSync.message,
              );
              allStarted = false;
              return false;
            }
          }

          // Start terminal manager
          if (config.terminal.enable) {
            logCtx.debug(
              `Starting Terminal-Manager on host '${config.host}'`,
              '',
            );
            const resultStartTerminal =
              await multimasterManager.startTerminalManager(
                config.rosVersion,
                credential,
                config.terminal.port,
              );
            if (!resultStartTerminal.result) {
              logCtx.warn(
                `Failed to start terminal manager on host '${config.host}'`,
                resultStartTerminal.message,
              );
              allStarted = false;
              return false;
            }
          }

          // wait a little longer to make sure the processes are fully started
          setTimeout(() => {
            logCtx.success(
              `Provider on host '${config.host}' started successfully`,
              '',
            );
            connectToProvider(provider);
          }, 3000);
        } catch (error) {
          logCtx.error(`Error starting host: ${config.host}`, `${error}`);
          allStarted = false;
          const provider = getProviderByHosts([config.host], config.port, null);
          if (provider !== null) {
            if (`${error}`.includes('Missing SSH credentials')) {
              // add state (no SSH credentials) to provider
              provider.setConnectionState(
                ConnectionState.STATES.NO_SSH_CREDENTIALS,
                'start aborted',
              );
            } else {
              provider.setConnectionState(
                ConnectionState.STATES.UNREACHABLE,
                `Error starting host: ${config.host}: ${error}`,
              );
            }
          } else {
            console.warn(
              `provider for host ${config.host}:${config.port} not found`,
            );
          }
        }
        return allStarted;
      },
      // eslint-disable-next-line react-hooks/exhaustive-deps
      [
        multimasterManager,
        isLocalHost,
        SSHCtx,
        getProviderByHosts,
        settingsCtx,
        logCtx,
      ],
    );

  const startProvider: (
    provider: CrossbarIOProvider,
    forceStartWithDefault: boolean,
  ) => Promise<boolean> = useCallback(
    async (
      provider: CrossbarIOProvider,
      forceStartWithDefault: boolean = true,
    ) => {
      provider.setConnectionState(ConnectionState.STATES.STARTING, '');
      const cfg = providerLaunchesMap.get(provider.id);
      if (cfg) {
        const result = await startConfig(cfg);
        return result;
      }
      if (forceStartWithDefault) {
        // use default configuration to start the provider
        const defaultCfg = new ProviderLaunchConfiguration(
          provider.host(),
          provider.rosVersion,
        );
        defaultCfg.daemon.enable = true;
        defaultCfg.discovery.enable = true;
        defaultCfg.terminal.enable = true;
        defaultCfg.autoConnect = true;
        defaultCfg.autostart = false;
        const result = await startConfig(defaultCfg);
        return result;
      }
      return false;
    },
    [providerLaunchesMap, startConfig],
  );

  /** Connects to the given provider and add it to the list. */
  const connectToProvider: (prov: CrossbarIOProvider) => Promise<boolean> =
    useCallback(
      async (prov) => {
        // check / add the provider
        const provider = getProviderByHosts(
          prov.hostnames,
          prov.crossbar.port,
          prov,
        ) as CrossbarIOProvider;

        if (provider.isAvailable()) {
          // already connected
          return true;
        }

        provider.isLocalHost = isLocalHost(provider.crossbar.host);
        try {
          if (await provider.init()) {
            return true;
          }
          const launchCfg = providerLaunchesMap.get(provider.id);
          if (launchCfg?.autostart && launchCfg?.daemon.enable) {
            // start system nodes, if auto start is enabled
            logCtx.info(
              `Start system nodes for provider [${provider.name()}]`,
              `URI: ${provider.url()}`,
            );
            const startResult = await startProvider(provider, false);
            // if (startResult) {
            //   logCtx.success(
            //     `Provider on host '${provider.host()}' started successfully`,
            //     '',
            //   );
            //   // wait a little longer to make sure the processes are fully started
            //   setTimeout(() => {
            //     connectToProvider(provider);
            //   }, 3000);
            //   // const connectResult = await connectToProvider(provider);
            //   // return connectResult;
            // }
            return startResult;
          }
          const error = `Could not initialize provider [${provider.name()}] (${
            provider.type
          } ) in [ws://${provider.crossbar.host}:${provider.crossbar.port}]`;
          const details = `Initialization failed, please check your provider configuration; autostart: ${launchCfg?.autostart}`;
          logCtx.error(error, details);
          provider.errorDetails = `${error} ${details}`;
        } catch (error: any) {
          logCtx.debug(
            `Could not initialize provider [${provider.name()}] (${
              provider.type
            } )in [ws://${provider.crossbar.host}:${provider.crossbar.port}]`,
            `Error: ${JSON.stringify(error)}`,
          );
        }
        return false;
      },
      // eslint-disable-next-line react-hooks/exhaustive-deps
      [
        providerLaunchesMap,
        logCtx,
        startProvider,
        getProviderByHosts,
        setProvidersAddQueue,
      ],
    );

  const getProviderName = (providerId: string) => {
    const name = providers.filter((item) => item.id === providerId)[0]?.name();
    return name || '';
  };

  /**
   * Register subscriptions for crossbar URI callbacks for a given provider
   */
  const registerSubscriber = useCallback(
    async (
      providerId: string,
      topic: string,
      messageType: string,
      filter: SubscriberFilter,
    ) => {
      const provider = getProviderById(providerId) as CrossbarIOProvider;
      if (!provider) {
        logCtx.error(
          `Can not start subscriber for: ${topic}`,
          `Provider not found: ${providerId}`,
        );
        return;
      }

      // start ros node with given subscriber
      const sNode = new SubscriberNode(topic, messageType);
      if (filter !== undefined) {
        sNode.filter = filter;
      }
      await provider.startSubscriber(sNode);
    },
    [getProviderById, logCtx],
  );

  const unregisterSubscriber = useCallback(
    async (providerId: string, topic: string) => {
      const provider = getProviderById(providerId) as CrossbarIOProvider;
      if (!provider) {
        logCtx.error(
          `Can not stop subscriber for: ${topic}`,
          `Provider not found: ${providerId}`,
        );
        return;
      }

      // stop ros node for given topic
      const result = await provider.stopSubscriber(topic);
      if (result) {
        logCtx.debug(
          `Stopped subscriber node for '${topic} on '${provider.name()}'`,
          '',
        );
      } else {
        logCtx.error(
          `Can not stop subscriber node for: ${topic} on '${provider.name()}`,
          `${result}`,
        );
      }
    },
    [getProviderById, logCtx],
  );

  /**
   * Update the filter for given topic
   */
  const updateFilterRosTopic = useCallback(
    async (
      provider: CrossbarIOProvider,
      topicName: string,
      msg: SubscriberFilter,
    ) => {
      if (!provider.isAvailable()) {
        return;
      }
      await provider.updateFilterRosTopic(topicName, msg);
    },
    [],
  );

  function createReloadFileAlertComponent(
    key: SnackbarKey | undefined,
    message: string,
    provider: CrossbarIOProvider,
    modifiedFile: string,
    modification: PATH_EVENT_TYPE,
    launchFilePath: string,
    onReload: (providerId: string, modifiedFile: string) => Promise<void>,
  ) {
    return (
      <ReloadFileAlertComponent
        id={key}
        message={message}
        provider={provider}
        modifiedFile={modifiedFile}
        modification={modification}
        launchFile={launchFilePath}
        onReload={onReload}
      />
    );
  }

  // initialize ROS and system Info
  const init = async () => {
    setInitialized(() => false);
    setInitializedSystem(false);
    setInitializedProviders(false);

    if (window.MultimasterManager) {
      setMultimasterManager(window.MultimasterManager);
    }
    // get local ROS Info
    if (window.ROSInfo) {
      const rinfo = await window.ROSInfo.getInfo();
      setRosInfo(rinfo);
    }
    // get local System Info
    if (window.SystemInfo) {
      setSystemInfo(await window.SystemInfo.getInfo());
    }
    setInitializedSystem(true);
  };

  useCustomEventListener(
    EVENT_PROVIDER_DISCOVERED,
    (data: EventProviderDiscovered) => {
      // trigger add new provider
      logCtx.debug(
        `trigger add new provider: ${data.provider.rosState.name}`,
        `RosState details: ${JSON.stringify(data.provider.rosState)}`,
      );
      setProvidersAddQueue((oldValue) => [...oldValue, data.provider]);
    },
    [setProvidersAddQueue],
  );

  /** Handle events caused by changed files. */
  useCustomEventListener(
    EVENT_PROVIDER_PATH_EVENT,
    (data: EventProviderPathEvent) => {
      data.path.affected.forEach((arg: string) => {
        enqueueSnackbar(
          `Do you want to reload file [${getFileName(arg)}] on `,
          {
            persist: true,
            anchorOrigin: {
              vertical: 'top',
              horizontal: 'right',
            },
            preventDuplicate: true,
            content: (key, message) => {
              return createReloadFileAlertComponent(
                key,
                `${message}`,
                data.provider,
                data.path.srcPath,
                data.path.eventType,
                arg,
                reloadLaunchFile,
              );
            },
          },
        );
      });
    },
  );

  useCustomEventListener(
    EVENT_PROVIDER_ROS_NODES,
    (data: EventProviderRosNodes) => {
      // add nodes to map
      const newMap = new Map();
      // remove provider not in the providers list anymore
      const availableProviderKeys = providers.map((prov) => {
        return prov.id;
      });
      // remove not listed provider
      mapProviderRosNodes.forEach((nodes, provId) => {
        if (availableProviderKeys.includes(provId)) {
          newMap.set(provId, nodes);
        }
      });
      newMap.set(data.provider.id, data.nodes);
      logCtx.debug(
        `ros nodes updated for ${data.provider.id}: ${data.nodes.length} nodes`,
        '',
      );

      setMapProviderRosNodes(newMap);
    },
    [providers, mapProviderRosNodes, setMapProviderRosNodes],
  );

  useCustomEventListener(
    EVENT_PROVIDER_SCREENS,
    (data: EventProviderScreens) => {
      let updated = false;
      // update screens for affected node
      data.screens.forEach((nodeScreen) => {
        const matchingNode = data.provider.rosNodes.find(
          (node) =>
            node.id === nodeScreen.name || node.name === nodeScreen.name,
        );
        if (matchingNode) {
          matchingNode.screens = nodeScreen.screens;
          updated = true;
        }
      });
      if (updated) {
        // inform listener about updated nodes
        emitCustomEvent(
          EVENT_PROVIDER_ROS_NODES,
          new EventProviderRosNodes(data.provider, data.provider.rosNodes),
        );
      }
    },
  );

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    const { provider, newState, oldState, details } = data;
    console.log(
      `trigger connection state ${
        provider.id
      }: new: ${newState}, old: ${oldState}, ${JSON.stringify(details)}`,
    );
    switch (newState) {
      case ConnectionState.STATES.CONNECTING:
        // this state is set by provider itself while connect
        break;
      case ConnectionState.STATES.STARTING:
        // this state is set by this context while it starts nodes
        break;
      case ConnectionState.STATES.CROSSBAR_CONNECTED:
        break;
      case ConnectionState.STATES.CROSSBAR_REGISTERED:
        break;
      case ConnectionState.STATES.CONNECTED:
        // trigger updates on state change
        setProvidersConnectedPast([
          ...providersConnectedPast.filter((p) => p.id !== provider.id),
          provider,
        ]);
        setProvidersConnected(
          providers.filter((prov) => {
            return prov.connectionState === ConnectionState.STATES.CONNECTED;
          }),
        );
        break;
      case ConnectionState.STATES.CLOSED:
        mapProviderRosNodes.set(provider.id, []);
        setProvidersConnected(
          providers.filter((prov) => {
            return prov.connectionState === ConnectionState.STATES.CONNECTED;
          }),
        );
        break;
      case ConnectionState.STATES.NO_SSH_CREDENTIALS:
      case ConnectionState.STATES.LOST:
      case ConnectionState.STATES.UNSUPPORTED:
      case ConnectionState.STATES.UNREACHABLE:
      case ConnectionState.STATES.ERRORED:
        setProvidersConnected(
          providers.filter((prov) => {
            return prov.connectionState === ConnectionState.STATES.CONNECTED;
          }),
        );
        break;
      default:
        break;
    }
  });

  useCustomEventListener(
    EVENT_PROVIDER_WARNINGS,
    (data: EventProviderWarnings) => {
      logCtx.debugCrossbar(
        'ros.provider.warnings',
        data.warnings,
        '',
        data.provider.id,
      );
    },
  );

  // useEffect(() => {
  //   // remove provider not in the providers list
  //   const availableProviderKeys = providers.map((prov) => {
  //     return prov.id;
  //   });
  //   console.log(`KEY CHANGED: ${JSON.stringify(availableProviderKeys)}`);
  //   const newMap = new Map();
  //   mapProviderRosNodes.forEach((provider));
  // }, [providers]);

  useEffect(() => {
    providers.forEach((provider) => {
      if (provider.connectionState === ConnectionState.STATES.UNKNOWN) {
        let doConnect: boolean = true;
        if (providerLaunchesMap.has(provider.id)) {
          doConnect = providerLaunchesMap.get(provider.id)
            ?.autoConnect as boolean;
          if (doConnect) {
            // trigger connect to the new provider if not deactivated by configuration
            connectToProvider(provider);
          }
        } else {
          // we have no configuration => it is a discovered provider => trigger connect to the new provider
          connectToProvider(provider);
        }
      }
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [providers, providerLaunchesMap]);

  /** Load stored provider configurations upon initialization */
  useEffect(() => {
    if (initializedSystem && !initializedProviders) {
      const newProviders: CrossbarIOProvider[] = [];
      const provLaunchMap: Map<string, ProviderLaunchConfiguration> = new Map();
      let localHostFound = false;
      providerLaunches.forEach((cfg) => {
        if (
          ['localhost', '127.0.0.1'].includes(cfg.host) ||
          isLocalHost(cfg.host)
        ) {
          localHostFound = true;
        }
        const np = new CrossbarIOProvider(
          settingsCtx,
          cfg.host,
          cfg.rosVersion,
          cfg.port,
          cfg.useSSL,
          logCtx,
        );
        np.isLocalHost = localHostFound;
        newProviders.push(np);
        provLaunchMap.set(np.id, cfg);
      });
      if (!localHostFound) {
        // if no stored launch configuration for local provider found, try to connect with default values
        // TODO add configuration for local provider start
        const localCfg = new ProviderLaunchConfiguration(
          'localhost',
          rosInfo?.version ? rosInfo.version : settingsCtx.get('rosVersion'),
        );
        localCfg.daemon.enable = true;
        localCfg.discovery.enable = true;
        localCfg.terminal.enable = true;
        localCfg.autoConnect = true;
        localCfg.autostart = true;
        const np = new CrossbarIOProvider(
          settingsCtx,
          localCfg.host,
          localCfg.rosVersion,
          localCfg.port,
          localCfg.useSSL,
          logCtx,
        );
        newProviders.push(np);
        provLaunchMap.set(np.id, localCfg);
      }
      setProviderLaunchesMap(provLaunchMap);
      setProvidersAddQueue(newProviders);
      setInitializedProviders(true);
    }
    // }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    initializedSystem,
    initializedProviders,
    providerLaunches,
    setProviderLaunchesMap,
    setProvidersAddQueue,
    setInitializedProviders,
  ]);

  /** A provider was enqueued. If it is a new one, add it to providers lists. */
  useEffect(() => {
    if (providersAddQueue.length > 0) {
      const prov = providersAddQueue.pop();
      let hostnames = prov?.hostnames;
      if (!hostnames) {
        hostnames = prov?.host() ? [prov?.host()] : [];
      }
      const provider = getProviderByHosts(hostnames, prov?.crossbar.port, null);
      if (provider === null) {
        if (prov) {
          setProviders((oldValue) => {
            // create new sorted list
            const newProviders = [...oldValue, prov];
            newProviders.sort((a, b) => -b.name().localeCompare(a.name()));
            return newProviders;
          });
          // update providerLaunchesMap; on start new provider, the configuration will be stored first and provider created while startConfig
          if (!providerLaunchesMap.has(prov.id)) {
            const lc: ProviderLaunchConfiguration | undefined =
              getProviderLaunchConfigByHost(
                prov.crossbar.host,
                prov.crossbar.port,
              );
            if (lc) {
              providerLaunchesMap.set(prov.id, lc);
            }
          }
        }
      } else {
        // provider already registered, try to connect
        connectToProvider(provider);
      }
      setProvidersAddQueue([...providersAddQueue]);
    }
  }, [
    providersAddQueue,
    getProviderByHosts,
    setProvidersAddQueue,
    setProviders,
    providerLaunchesMap,
    getProviderLaunchConfigByHost,
    connectToProvider,
  ]);

  useEffect(() => {
    setInitialized(initializedProviders && initializedSystem);
  }, [initializedProviders, initializedSystem, setInitialized]);

  // Effect to initialize RosContext
  useEffect(() => {
    init();
  }, []);

  const attributesMemo = useMemo(
    () => ({
      initialized,
      rosInfo,
      systemInfo,
      multimasterManager,
      providers,
      providersConnected,
      providersConnectedPast,
      mapProviderRosNodes,
      connectToProvider,
      startProvider,
      startConfig,
      refreshProviderList,
      closeProviders,
      updateNodeList,
      updateLaunchList,
      reloadLaunchFile,
      getProviderById,
      getProviderByHost,
      registerSubscriber,
      unregisterSubscriber,
      updateFilterRosTopic,
      getProviderName,
      getProviderLaunchConfig,
      isLocalHost,
      saveProviderConfig,
      deleteProviderConfig,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      initialized,
      rosInfo,
      systemInfo,
      multimasterManager,
      providers,
      providersConnected,
      providersConnectedPast,
      mapProviderRosNodes,
    ],
  );

  return (
    <RosContext.Provider value={attributesMemo}>{children}</RosContext.Provider>
  );
}

export default RosContext;
