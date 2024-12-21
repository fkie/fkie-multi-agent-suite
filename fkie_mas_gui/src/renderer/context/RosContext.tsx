import { TFileRange, TLaunchArg, TRosInfo, TSystemInfo } from "@/types";
import { useDebounceCallback } from "@react-hook/debounce";
import { Model } from "flexlayout-react";
import { SnackbarKey, useSnackbar } from "notistack";
import React, { createContext, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { ConnectConfig } from "ssh2";
import { ReloadFileAlertComponent, RestartNodesAlertComponent } from "../components/UI";
import {
  LaunchArgument,
  LaunchLoadRequest,
  PATH_EVENT_TYPE,
  ProviderLaunchConfiguration,
  RosNode,
  SubscriberFilter,
  SubscriberNode,
  getBaseName,
  getFileName,
} from "../models";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../pages/NodeManager/layout";
import {
  EVENT_EDITOR_SELECT_RANGE,
  EVENT_OPEN_COMPONENT,
  eventEditorSelectRange,
  eventOpenComponent,
} from "../pages/NodeManager/layout/events";
import FileEditorPanel from "../pages/NodeManager/panels/FileEditorPanel";
import SingleTerminalPanel from "../pages/NodeManager/panels/SingleTerminalPanel";
import TopicEchoPanel from "../pages/NodeManager/panels/TopicEchoPanel";
import { CmdType, ConnectionState } from "../providers";
import Provider from "../providers/Provider";
import {
  EVENT_PROVIDER_AUTH_REQUEST,
  EVENT_PROVIDER_DISCOVERED,
  EVENT_PROVIDER_PATH_EVENT,
  EVENT_PROVIDER_REMOVED,
  EVENT_PROVIDER_RESTART_NODES,
  EVENT_PROVIDER_ROS_NODES,
  EVENT_PROVIDER_SCREENS,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from "../providers/eventTypes";
import {
  EventProviderAuthRequest,
  EventProviderDiscovered,
  EventProviderPathEvent,
  EventProviderRemoved,
  EventProviderRestartNodes,
  EventProviderRosNodes,
  EventProviderScreens,
  EventProviderState,
  EventProviderWarnings,
} from "../providers/events";
import { xor } from "../utils/index";
import { LoggingContext } from "./LoggingContext";
import { LAUNCH_FILE_EXTENSIONS, SettingsContext, getDefaultPortFromRos } from "./SettingsContext";

export interface IRosProviderContext {
  initialized: boolean;
  rosInfo: TRosInfo | null;
  systemInfo: TSystemInfo | null;
  providers: Provider[];
  providersConnected: Provider[];
  mapProviderRosNodes: Map<string, RosNode[]>;
  nodeMap: Map<string, RosNode>;
  layoutModel: Model | null;
  connectToProvider: (provider: Provider) => Promise<boolean>;
  startProvider: (provider: Provider, forceStartWithDefault: boolean) => Promise<boolean>;
  startConfig: (config: ProviderLaunchConfiguration, connectConfig: ConnectConfig) => Promise<boolean>;
  removeProvider: (providerId: string) => void;
  getProviderName: (providerId: string) => string;
  refreshProviderList: () => void;
  closeProviders: () => void;
  updateNodeList: (providerId: string) => void;
  updateLaunchList: (providerId: string) => void;
  reloadLaunchFile: (providerId: string, modifiedFile: string) => Promise<void>;
  getProviderById: (providerId: string, includeNotAvailable: boolean) => Provider | undefined;
  getProviderByHost: (hostName: string) => Provider | null;
  getLocalProvider: () => Provider[];
  registerSubscriber: (providerId: string, topic: string, messageType: string, filter: SubscriberFilter) => void;
  unregisterSubscriber: (providerId: string, topic: string) => void;
  updateFilterRosTopic: (provider: Provider, topicName: string, msg: SubscriberFilter) => void;
  isLocalHost: (host: string) => void;
  addProvider: (provider: Provider) => void;
  setLayoutModel: (model: Model) => void;
}

export const DEFAULT = {
  initialized: false,
  rosInfo: null,
  systemInfo: null,
  providers: [],
  providersConnected: [],
  mapProviderRosNodes: new Map(), // Map<providerId: string, nodes: RosNode[]>
  nodeMap: new Map(),
  layoutModel: null,
  connectToProvider: () => new Promise<false>(() => {}),
  startProvider: () => new Promise<boolean>(() => {}),
  startConfig: () => new Promise<boolean>(() => {}),
  removeProvider: () => null,
  getProviderName: () => "",
  refreshProviderList: () => {},
  closeProviders: () => {},
  updateNodeList: () => {},
  reloadLaunchFile: () => new Promise<void>(() => {}),
  updateLaunchList: () => {},
  getProviderById: () => undefined,
  getProviderByHost: () => null,
  getLocalProvider: () => [],
  registerSubscriber: () => null,
  unregisterSubscriber: () => null,
  updateFilterRosTopic: () => null,
  isLocalHost: () => null,
  addProvider: () => null,
  setLayoutModel: () => null,
};

interface IRosProviderComponent {
  children: React.ReactNode;
}

// (ms) time to debounce callbacks
// useful to prevent unnecessary updates
const debounceTimeout = 100;

export const RosContext = createContext<IRosProviderContext>(DEFAULT);

export function RosProviderReact(props: IRosProviderComponent): ReturnType<React.FC<IRosProviderComponent>> {
  const { children } = props;
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const { enqueueSnackbar } = useSnackbar();

  const [initialized, setInitialized] = useState(DEFAULT.initialized);
  const [rosInfo, setRosInfo] = useState<TRosInfo | null>(null);
  const [systemInfo, setSystemInfo] = useState<TSystemInfo | null>(null);
  const [providersAddQueue, setProvidersAddQueue] = useState<Provider[]>([]);
  const [providers, setProviders] = useState<Provider[]>([]);
  const [providersConnected, setProvidersConnected] = useState<Provider[]>([]);
  const [layoutModel, setLayoutModel] = useState<Model | null>(null);

  const [mapProviderRosNodes, setMapProviderRosNodes] = useState(DEFAULT.mapProviderRosNodes);
  // nodeMap: Map<string, RosNode>
  const [nodeMap, setNodeMap] = useState(new Map());

  /** Remove all disconnected provider and their discovered provider. */
  const clearProviders = useCallback(() => {
    const idsSavedProviders: string[] = [];
    // get ids of providers which are stored local and are connected
    providers.forEach((prov) => {
      if (prov.discovered === undefined && prov.connectionState === ConnectionState.STATES.CONNECTED) {
        idsSavedProviders.push(prov.id);
      }
    });
    // update discovered lists by removing not connected provider ids
    providers.forEach((prov) => {
      if (prov.discovered !== undefined) {
        prov.discovered = prov.discovered.filter((pid) => idsSavedProviders.indexOf(pid) !== -1);
      }
    });
    // remove all discovered provider with no parent provider
    // and not stored
    setProviders(
      providers.filter((prov) => {
        // remove provider deleted by user
        return (
          prov.discovered === undefined ||
          prov.discovered?.length ||
          prov.connectionState === ConnectionState.STATES.CONNECTED
        );
      })
    );
    setProvidersConnected(
      providers.filter((prov) => {
        if (prov.connectionState === ConnectionState.STATES.CONNECTED) {
          // provider is connected
          if (prov.discovered === undefined || prov.discovered?.length) {
            // and it is configured by user or discoverer is still connected
            return true;
          }
        }
        return false;
      })
    );
  }, [providers, setProvidersConnected, setProviders]);

  /** Returns true if given host is one of local IPv4 addresses and local host names.  */
  function isLocalHost(host: string) {
    if (host === "localhost" || host === systemInfo?.osInfo?.hostname) {
      return true;
    }
    const localIps: string[] = [];
    systemInfo?.networkInterfaces?.forEach((ni) => {
      if (host === ni.ip4) {
        localIps.push(ni.ip4);
      }
    });
    return localIps.length > 0;
  }

  function addProvider(provider: Provider) {
    if (!getProviderById(provider.id)) {
      setProviders((prev) => [...prev, provider]);
    }
  }

  /** Search and return a provider using its id */
  const getProviderById = useCallback(
    (providerId: string, includeNotAvailable: boolean = true) => {
      return providers.find((provider) => {
        return (provider.isAvailable() || includeNotAvailable) && provider.id === providerId;
      });
    },
    [providers]
  );

  /**
   * Search and return a provider using its host name
   */
  const getProviderByHost = useCallback(
    (hostName: string) => {
      if (initialized) {
        const p = providers.find((provider) => {
          return (
            provider.isAvailable() && (provider.connection.host === hostName || provider.hostnames.includes(hostName))
          );
        });

        if (p) return p;
      }

      return null;
    },
    [initialized, providers]
  );

  /**
   * Search and return a provider using known hosts and port
   */
  const getProviderByHosts = useCallback(
    (hosts: string[], port: number = 0, defaultValue: Provider | null = null) => {
      const p = providers.find((provider) => {
        let result = true;
        if (port !== 0 && provider.connection.port !== 0) {
          result = provider.connection.port === port;
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
    [providers]
  );

  /**
   * Search and return a provider it they are local
   */
  const getLocalProvider = useCallback(() => {
    const p = providers.filter((provider) => {
      return provider.isLocalHost;
    });
    return p;
  }, [providers]);

  /**
   * Trigger updateLaunchContent() of the provider.
   */
  const removeProvider = useCallback(
    async (providerId: string) => {
      logCtx.debug(`Triggering update of ROS launch files from ${providerId}`, "");
      setProviders((prev) =>
        prev.filter((prov) => {
          if (prov.id === providerId) {
            prov.close();
            return false;
          }
          return true;
        })
      );
      const provider = getProviderById(providerId);
      await provider?.updateLaunchContent();
    },
    [getProviderById, logCtx]
  );

  /**
   * Trigger updateRosNodes() of the provider.
   */
  const updateNodeList = useCallback(
    async (providerId: string) => {
      logCtx.debug(`Triggering update of ROS nodes from ${providerId}`, "", false);
      const provider = getProviderById(providerId);
      await provider?.updateRosNodes({});
    },
    [getProviderById, logCtx]
  );

  /**
   * Trigger updateLaunchContent() of the provider.
   */
  const updateLaunchList = useCallback(
    async (providerId: string) => {
      logCtx.debug(`Triggering update of ROS launch files from ${providerId}`, "");
      const provider = getProviderById(providerId);
      await provider?.updateLaunchContent();
    },
    [getProviderById, logCtx]
  );

  /**
   * Close the connection to all registered providers
   * Useful when new providers wants to be registered
   * as it prevents re-registration of URIs
   */
  const closeProviders = useCallback(() => {
    providers.forEach((prov) => {
      prov.close();
    });
  }, [providers]);

  /**
   * Launch a file [filePath] on a given [provider], using arguments [args]
   */
  const launchFile = useCallback(
    async (provider: Provider, filePath: string, args: LaunchArgument[], reload = false) => {
      if (!filePath) return { success: false, error: `Invalid file path` };
      if (!provider || !provider.isAvailable()) return { success: false, error: `Invalid/unavailable provider` };

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
      const rosPackage = ""; // ROS package name.
      const launch = ""; // Launch file in the package path.
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
        `${provider.rosState.masteruri ? provider.rosState.masteruri : ""}`,
        provider.connection.host
      );

      console.log(`LOAD LAUNCH: ${JSON.stringify(request)}`);
      const resultLaunchLoadFile = await provider.launchLoadFile(request, reload);

      if (!resultLaunchLoadFile) {
        return {
          success: false,
          error: `Invalid response for [launchLoadFile], check DAEMON screen output`,
          reply: null,
        };
      }

      if (resultLaunchLoadFile.status.code === "OK") {
        // trigger node's update (will force a reload using useEffect hook)
        updateNodeList(provider.id);
        // updateLaunchList(provider.name());
        return { success: true, error: "", reply: resultLaunchLoadFile };
      }

      if (resultLaunchLoadFile.status.code === "PARAMS_REQUIRED") {
        return {
          success: false,
          error: "Please fill all arguments",
          reply: resultLaunchLoadFile,
        };
      }

      if (resultLaunchLoadFile.status.code === "ERROR") {
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
    ]
  );

  function createRestartNodesAlertComponent(
    key: SnackbarKey | undefined,
    message: string,
    provider: Provider,
    changedNodes: string[],
    _modifiedFile: string
  ) {
    return (
      <RestartNodesAlertComponent
        id={key}
        message={message}
        provider={provider}
        nodeList={changedNodes}
        onReload={(_providerId, nodeList) => {
          // restart is handled by hostTreeViewPanel using queue
          const nodes = provider.rosNodes.filter((rosNode) => nodeList.includes(rosNode.name));
          // TODO: use modifiedFile e.g. node.launchPath = modifiedFile;
          emitCustomEvent(EVENT_PROVIDER_RESTART_NODES, new EventProviderRestartNodes(provider, nodes));
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
      if (LAUNCH_FILE_EXTENSIONS.find((fe) => modifiedFile.indexOf(fe) !== -1)) {
        const provider = getProviderById(providerId) as Provider;
        if (!provider) return;

        // Launch file again using last arguments

        // get args from launch file saved on provider object
        let args: LaunchArgument[] = [];
        if (provider.launchFiles) {
          const filteredLaunchFile = provider.launchFiles.find((lf) => lf.path === modifiedFile);
          if (filteredLaunchFile) {
            args = filteredLaunchFile.args;
          }
        }

        const result = await launchFile(provider, modifiedFile, args, true);
        if (result && result.success) {
          logCtx.success(`Launch file [${getFileName(modifiedFile)}] reloaded`, `File: ${modifiedFile}`);

          // check if nodes have to be restarted
          if (result.reply && result.reply.changed_nodes && result.reply.changed_nodes.length > 0) {
            // ask use if nodes should be restarted
            enqueueSnackbar(`Do you want to restart ${result.reply.changed_nodes.length} nodes on: `, {
              persist: true,
              anchorOrigin: {
                vertical: "top",
                horizontal: "right",
              },
              preventDuplicate: true,
              content: (key, message) =>
                createRestartNodesAlertComponent(key, `${message}`, provider, result.reply.changed_nodes, modifiedFile),
            });
          }
        }
        if (result && !result.success) {
          logCtx.error(`Launch file error: ${result.error}`, `Provider: ${provider.name()} File: ${modifiedFile}`);
        }
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [getProviderById, launchFile]
  );

  /** Connects to the given provider and add it to the list. */
  const connectToProvider: (prov: Provider) => Promise<boolean> = useCallback(
    async (prov) => {
      // check / add the provider
      const provider = getProviderByHosts(prov.hostnames, prov.connection.port, prov) as Provider;

      if (provider.connection.connected()) {
        return true;
      }
      try {
        if (provider.isAvailable()) {
          provider.setConnectionState(ConnectionState.STATES.CONNECTING, "");
          // already connected
          await provider.getDaemonVersion();
          provider.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          return true;
        }
      } catch (error: unknown) {
        logCtx.debug(
          `Could not initialize provider [${provider.name()}] (${
            provider.type
          }) in [ws://${provider.connection.host}:${provider.connection.port}]`,
          `Error: ${JSON.stringify(error)}`
        );
        provider.setConnectionState(ConnectionState.STATES.ERRORED, JSON.stringify(error));
        return false;
      }
      provider.setConnectionState(ConnectionState.STATES.CONNECTING, "");
      provider.isLocalHost = isLocalHost(provider.connection.host);
      try {
        if (await provider.init()) {
          return true;
        }
        const error = `Could not initialize provider [${provider.name()}] (${
          provider.type
        }) in [ws://${provider.connection.host}:${provider.connection.port}]`;
        // const details = `Initialization failed, please check your provider configuration; autostart: ${launchCfg?.autostart}`;
        logCtx.error(error, "");
        provider.errorDetails = `${error}`;
        provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, JSON.stringify(error));
      } catch (error: unknown) {
        logCtx.debug(
          `Could not initialize provider [${provider.name()}] (${
            provider.type
          }) in [ws://${provider.connection.host}:${provider.connection.port}]`,
          `Error: ${JSON.stringify(error)}`
        );
        provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, JSON.stringify(error));
      }
      return false;
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      logCtx,
      // startProvider,
      getProviderByHosts,
      setProvidersAddQueue,
    ]
  );

  /**
   * Forces an update on the provider list for all connected provider.
   */
  const refreshProviderList = useDebounceCallback(() => {
    // remove discoverd provider
    const newProviders = providers.filter((prov) => {
      return prov.discovered === undefined;
    });
    setProviders(newProviders);
    newProviders.forEach((provider) => {
      try {
        provider.updateProviderList();
        provider.getDaemonVersion().catch((err) => {
          logCtx.debug(`refreshProvider ${provider.name()} failed`, JSON.stringify(err), false);
          connectToProvider(provider);
        });
      } catch (error: unknown) {
        // ignore errors while refresh
        logCtx.debug(`refreshProviderList failed`, JSON.stringify(error), false);
      }
    });
  }, debounceTimeout);

  const startConfig: (config: ProviderLaunchConfiguration, connectConfig: ConnectConfig | null) => Promise<boolean> =
    useCallback(
      async (config, connectConfig = null) => {
        if (!window.commandExecutor) return false;

        let allStarted = true;
        try {
          const isLocal = isLocalHost(config.host);
          const credential = isLocal ? null : connectConfig ? connectConfig : { host: config.host };

          const port = config.port
            ? config.port
            : getDefaultPortFromRos(Provider.defaultType, config.rosVersion, config.ros1MasterUri, config.networkId);
          // check and add provider if new
          let provider = getProviderByHosts([config.host], port, null) as Provider;
          if (!provider) {
            provider = new Provider(
              settingsCtx,
              config.host,
              config.rosVersion,
              port,
              config.networkId,
              config.useSSL,
              logCtx
            );
            provider.isLocalHost = isLocal;
            provider.startConfiguration = config;
            // add provider using add queue
            setProvidersAddQueue((oldValues) => [...oldValues, provider]);
            // return false;
          }

          if (!(config.daemon.enable || config.discovery.enable || config.terminal.enable)) {
            // use default configuration if no one is configured to start
            config.daemon.enable = true;
            config.discovery.enable = true;
            config.terminal.enable = true;
          }

          provider.setConnectionState(ConnectionState.STATES.STARTING, "");
          // Find running system nodes
          const systemNodes = provider.rosNodes.filter((n) => n.system_node);
          if (config.forceStop && systemNodes?.length > 0) {
            // stop all requested system nodes, daemon as last node
            let nodesToStop: RosNode[] = [];
            let syncNode;
            let daemonNode;
            let discoveryNode;
            if (config.sync.enable) {
              syncNode = systemNodes.find((n) => n.id.includes("mas_sync") || n.id.includes("master_sync"));
            }
            if (config.daemon.enable) {
              daemonNode = systemNodes.find((n) => n.id.includes("mas_daemon") || n.id.includes("node_manager_daemon"));
            }
            if (config.discovery.enable) {
              discoveryNode = systemNodes.find(
                (n) => n.id.includes("mas_discovery") || n.id.includes("master_discovery")
              );
            }
            // we have to stop in right order to be able to use stop_node() method of the provider
            if (config.rosVersion === "1") {
              nodesToStop = [syncNode, daemonNode, discoveryNode];
            } else {
              nodesToStop = [discoveryNode, daemonNode];
            }
            await Promise.all(
              nodesToStop.map(async (node) => {
                if (node) {
                  logCtx.debug(`Stopping running ${node.name} on host '${config.host}'`, "");
                  await provider.stopNode(node.id);
                }
              })
            );
            // wait a little bit until the ros node is unregistered
            // await delay(1000);
            config.forceStop = false;
          }
          // Start Daemon
          if (config.daemon.enable) {
            logCtx.debug(`Starting daemon on host '${config.host}'`, "");
            const cmd = config.daemonStartCmd();
            if (cmd.result) {
              const resultStartDaemon = await window.commandExecutor?.exec(credential, cmd.message);
              if (resultStartDaemon) {
                if (!resultStartDaemon.result) {
                  if (resultStartDaemon.connectConfig) {
                    logCtx.error(`Request password`, `${JSON.stringify(resultStartDaemon.connectConfig)}`);
                    emitCustomEvent(
                      EVENT_PROVIDER_AUTH_REQUEST,
                      new EventProviderAuthRequest(provider, config, resultStartDaemon.connectConfig)
                    );
                    allStarted = false;
                    return false;
                  } else {
                    logCtx.error(`Failed to start daemon on host '${config.host}'`, resultStartDaemon.message);
                    provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, resultStartDaemon.message);
                    allStarted = false;
                    return false;
                  }
                }
              }
            } else {
              logCtx.error(`Failed to create daemon start command: ${cmd.message}`, JSON.stringify(config));
            }
          }
          // Start Discovery
          if (config.discovery.enable) {
            logCtx.debug(`Starting master-discovery on host '${config.host}'`, "");
            const cmd = config.masterDiscoveryStartCmd();
            if (cmd.result) {
              const resultStartDiscovery = await window.commandExecutor?.exec(credential, cmd.message);
              if (resultStartDiscovery) {
                if (!resultStartDiscovery.result) {
                  if (resultStartDiscovery.connectConfig) {
                    logCtx.error(`Request password`, `${JSON.stringify(resultStartDiscovery.connectConfig)}`);
                    emitCustomEvent(
                      EVENT_PROVIDER_AUTH_REQUEST,
                      new EventProviderAuthRequest(provider, config, resultStartDiscovery.connectConfig)
                    );
                    allStarted = false;
                    return false;
                  } else {
                    logCtx.error(`Failed to start discovery on host '${config.host}'`, resultStartDiscovery.message);
                    provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, resultStartDiscovery.message);
                    allStarted = false;
                    return false;
                  }
                }
              }
            } else {
              logCtx.error(`Failed to create discovery start command: ${cmd.message}`, JSON.stringify(config));
            }
          }

          // Restart Sync
          if (config.sync.enable) {
            logCtx.debug(`Starting master-sync on host '${config.host}'`, "");
            const cmd = config.masterSyncStartCmd();
            if (cmd.result) {
              const resultStartSync = await window.commandExecutor?.exec(credential, cmd.message);
              if (resultStartSync) {
                if (!resultStartSync.result) {
                  if (resultStartSync.connectConfig) {
                    logCtx.error(`Request password`, `${JSON.stringify(resultStartSync.connectConfig)}`);
                    emitCustomEvent(
                      EVENT_PROVIDER_AUTH_REQUEST,
                      new EventProviderAuthRequest(provider, config, resultStartSync.connectConfig)
                    );
                    allStarted = false;
                    return false;
                  } else {
                    logCtx.error(`Failed to start sync on host '${config.host}'`, resultStartSync.message);
                    provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, resultStartSync.message);
                    allStarted = false;
                    return false;
                  }
                }
              }
            } else {
              logCtx.error(`Failed to create sync start command: ${cmd.message}`, JSON.stringify(config));
            }
          }

          // Start terminal manager
          if (config.terminal.enable) {
            logCtx.debug(`Starting Terminal-Manager on host '${config.host}'`, "");

            const cmd = config.terminalStartCmd();
            if (cmd.result) {
              const resultStartTerminal = await window.commandExecutor?.exec(credential, cmd.message);
              if (resultStartTerminal) {
                if (!resultStartTerminal.result) {
                  if (resultStartTerminal.connectConfig) {
                    logCtx.error(`Request password`, `${JSON.stringify(resultStartTerminal.connectConfig)}`);
                    emitCustomEvent(
                      EVENT_PROVIDER_AUTH_REQUEST,
                      new EventProviderAuthRequest(provider, config, resultStartTerminal.connectConfig)
                    );
                    allStarted = false;
                    return false;
                  } else {
                    logCtx.error(`Failed to start terminal on host '${config.host}'`, resultStartTerminal.message);
                    provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, resultStartTerminal.message);
                    allStarted = false;
                    return false;
                  }
                }
              }
            } else {
              logCtx.error(`Failed to create terminal start command: ${cmd.message}`, JSON.stringify(config));
            }
          }

          // wait a little longer to make sure the processes are fully started
          setTimeout(() => {
            logCtx.success(`Provider on host '${config.host}' started successfully`, "");
            connectToProvider(provider);
          }, 3000);
        } catch (error) {
          logCtx.error(`Error starting host: ${config.host}`, `${error}`);
          allStarted = false;
          const provider = getProviderByHosts([config.host], config.port, null);
          if (provider !== null) {
            if (`${error}`.includes("Missing SSH credentials")) {
              // add state (no SSH credentials) to provider
              provider.setConnectionState(ConnectionState.STATES.AUTHZ, "start aborted");
            } else {
              provider.setConnectionState(
                ConnectionState.STATES.UNREACHABLE,
                `Error starting host: ${config.host}: ${error}`
              );
            }
          } else {
            console.warn(`provider for host ${config.host}:${config.port} not found`);
          }
        }
        return allStarted;
      },
      // eslint-disable-next-line react-hooks/exhaustive-deps
      [isLocalHost, getProviderByHosts, settingsCtx, logCtx]
    );

  const startProvider: (provider: Provider, forceStartWithDefault: boolean) => Promise<boolean> = useCallback(
    async (provider: Provider, forceStartWithDefault: boolean = true) => {
      provider.setConnectionState(ConnectionState.STATES.STARTING, "");
      const cfg = provider.startConfiguration;
      if (cfg) {
        const result = await startConfig(cfg, null);
        return result;
      }
      if (forceStartWithDefault) {
        // use default configuration to start the provider
        const defaultCfg = new ProviderLaunchConfiguration(
          provider.host(),
          provider.rosVersion,
          provider.connection?.port
        );
        defaultCfg.daemon.enable = true;
        defaultCfg.discovery.enable = true;
        defaultCfg.terminal.enable = true;
        defaultCfg.autoConnect = true;
        defaultCfg.autostart = false;
        const result = await startConfig(defaultCfg, null);
        return result;
      }
      return false;
    },
    [startConfig]
  );

  const startMasterSync = useCallback(
    async (host: string, rosVersion: string) => {
      if (!window.commandExecutor) return false;
      const isLocal = isLocalHost(host);
      const credential = isLocal ? null : { host: host };
      if (!isLocal && !credential) {
        logCtx.error(`'Missing SSH credentials to starting master-sync on host '${host}'`, "");
        return false;
      }
      logCtx.debug(`Starting master-sync on host '${host}'`, "");
      // check and add provider if new
      const provider = getProviderByHosts([host]) as Provider;

      const launchCfg = new ProviderLaunchConfiguration(host, rosVersion);
      launchCfg.daemon.enable = false;
      launchCfg.discovery.enable = false;
      launchCfg.sync.enable = true;
      launchCfg.sync.doNotSync = [];
      launchCfg.sync.syncTopics = [];
      launchCfg.terminal.enable = false;
      launchCfg.forceStop = true;
      const cmd = launchCfg.masterSyncStartCmd();
      if (cmd.result) {
        const resultStartSync = await window.commandExecutor?.exec(credential, cmd.message);
        if (resultStartSync) {
          if (!resultStartSync.result) {
            if (resultStartSync.connectConfig) {
              logCtx.error(`Request password`, `${JSON.stringify(resultStartSync.connectConfig)}`);
              emitCustomEvent(
                EVENT_PROVIDER_AUTH_REQUEST,
                new EventProviderAuthRequest(provider, launchCfg, resultStartSync.connectConfig)
              );
              return false;
            } else {
              logCtx.error(`Failed to start sync on host '${launchCfg.host}'`, resultStartSync.message);
              return false;
            }
          }
        }
      }
      return true;
    },
    [isLocalHost, logCtx, getProviderByHosts]
  );

  const startDynamicReconfigureClient = useCallback(
    async (nodeName: string, masteruri: string) => {
      if (!window.commandExecutor) return { result: false, message: "dynamic reconfigure not available in Browser" };
      logCtx.debug(`Starting Dynamic Reconfigure GUI for '${nodeName}'`, "");
      if (!masteruri) {
        const msg = `Start dynamic reconfigure failed: unknown ROS_MASTER_URI for node ${nodeName}`;
        logCtx.error(msg, "");
        return { result: false, message: msg };
      }
      const config = new ProviderLaunchConfiguration("localhost", "1");
      config.ros1MasterUri = masteruri;
      const cmd = config.dynamicReconfigureClientCmd(nodeName, masteruri);
      if (cmd.result) {
        const resultStartSync = await window.commandExecutor?.exec(null, cmd.message);
        if (resultStartSync) {
          if (!resultStartSync.result) {
            if (resultStartSync.connectConfig) {
              logCtx.error(`Request password`, `${JSON.stringify(resultStartSync.connectConfig)}`);
              emitCustomEvent(
                EVENT_PROVIDER_AUTH_REQUEST,
                new EventProviderAuthRequest(
                  getProviderByHosts(["localhost"]) as Provider,
                  config,
                  resultStartSync.connectConfig
                )
              );
              return false;
            } else {
              logCtx.error(
                `Failed to start dynamic reconfigure node on host '${config.host}'`,
                resultStartSync.message
              );
              return false;
            }
          } else {
            return true;
          }
        }
      }
      return false;
    },
    [logCtx]
  );

  const getProviderName = (providerId: string) => {
    const name = providers.filter((item) => item.id === providerId)[0]?.name();
    return name || "";
  };

  /**
   * Register subscriptions for URI callbacks for a given provider
   */
  const registerSubscriber = useCallback(
    async (providerId: string, topic: string, messageType: string, filter: SubscriberFilter) => {
      const provider = getProviderById(providerId) as Provider;
      if (!provider) {
        logCtx.error(`Can not start subscriber for: ${topic}`, `Provider not found: ${providerId}`);
        return;
      }

      // start ros node with given subscriber
      const sNode = new SubscriberNode(topic, messageType);
      if (filter !== undefined) {
        sNode.filter = filter;
      }
      await provider.startSubscriber(sNode);
    },
    [getProviderById, logCtx]
  );

  const unregisterSubscriber = useCallback(
    async (providerId: string, topic: string) => {
      const provider = getProviderById(providerId) as Provider;
      if (!provider) {
        logCtx.error(`Can not stop subscriber for: ${topic}`, `Provider not found: ${providerId}`);
        return;
      }

      // stop ros node for given topic
      const result = await provider.stopSubscriber(topic);
      if (result) {
        logCtx.debug(`Stopped subscriber node for '${topic} on '${provider.name()}'`, "");
      } else {
        logCtx.error(`Can not stop subscriber node for: ${topic} on '${provider.name()}`, `${result}`);
      }
    },
    [getProviderById, logCtx]
  );

  /**
   * Update the filter for given topic
   */
  const updateFilterRosTopic = useCallback(async (provider: Provider, topicName: string, msg: SubscriberFilter) => {
    if (!provider.isAvailable()) {
      return;
    }
    await provider.updateFilterRosTopic(topicName, msg);
  }, []);

  function createReloadFileAlertComponent(
    key: SnackbarKey | undefined,
    message: string,
    provider: Provider,
    modifiedFile: string,
    modification: PATH_EVENT_TYPE,
    launchFilePath: string,
    onReload: (providerId: string, modifiedFile: string) => Promise<void>
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

  const openEditor = useCallback(
    async (
      providerId: string,
      rootLaunch: string,
      path: string,
      fileRange: TFileRange,
      launchArgs: TLaunchArg[],
      externalKeyModifier: boolean
    ) => {
      const provider = getProviderById(providerId);
      if (provider) {
        const id = `editor-${provider.connection.host}-${provider.connection.port}-${rootLaunch}`;
        // open in external window depending on setting and key modifier and if no tab already existing
        const openExternal: boolean =
          xor(settingsCtx.get("editorOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);
        const hasExtEditor = await window.editorManager?.has(id);
        if (hasExtEditor) {
          // inform external window about new selected range
          window.editorManager?.emitFileRange(id, path, fileRange, launchArgs);
        } else if (openExternal && provider && window.editorManager) {
          // open in new window
          window.editorManager?.open(
            id,
            provider.connection.host,
            provider.connection.port,
            path,
            rootLaunch,
            fileRange,
            launchArgs
          );
        } else {
          // inform already open tab about new node selection
          emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(id, path, fileRange, launchArgs));
          // open new editor in a tab. Checks for existing tabs are performed in NodeManager
          emitCustomEvent(
            EVENT_OPEN_COMPONENT,
            eventOpenComponent(
              id,
              getBaseName(rootLaunch),
              <FileEditorPanel
                tabId={id}
                providerId={providerId}
                currentFilePath={path}
                rootFilePath={rootLaunch}
                fileRange={fileRange}
                launchArgs={launchArgs}
              />,
              true,
              LAYOUT_TAB_SETS[settingsCtx.get("editorOpenLocation")],
              new LayoutTabConfig(true, "editor", null, {
                id: id,
                host: provider.connection.host,
                port: provider.connection.port,
                rootLaunch: rootLaunch,
                path: path,
                fileRange: fileRange,
                launchArgs: launchArgs,
              })
            )
          );
        }
      }
    },
    [getProviderById, emitCustomEvent, layoutModel]
  );

  const openSubscriber = useCallback(
    async (
      providerId: string,
      topic: string,
      showOptions: boolean,
      defaultNoData: boolean,
      externalKeyModifier: boolean,
      forceOpenTerminal: boolean
    ) => {
      const provider = getProviderById(providerId);
      if (provider) {
        const id = `echo-${provider.connection.host}-${provider.connection.port}-${topic}`;
        // open in external window depending on setting and key modifier and if no tab already existing
        const openExternal: boolean =
          xor(settingsCtx.get("subscriberOpenExternal") as boolean, externalKeyModifier) &&
          !layoutModel?.getNodeById(id);
        if (forceOpenTerminal) {
          try {
            const terminalCmd = await provider.cmdForType(CmdType.ECHO, "", topic, "", "");
            const result = await window.commandExecutor?.execTerminal(
              null, // we start the subscriber always local
              `"echo ${topic}"`,
              terminalCmd.cmd
            );
            if (!result?.result) {
              logCtx.error(`Can't open subscriber in external terminal for ${topic}`, `${result?.message}`, true);
            }
          } catch (error) {
            logCtx.error(`Can't open subscriber in external terminal for ${topic}`, `${error}`, true);
          }
          return;
        }
        if (provider && window.subscriberManager && (openExternal || (await window.subscriberManager?.has(id)))) {
          // open in new window
          // we do not check for existing subscriber, it is done by IPC with given id
          window.subscriberManager?.open(
            id,
            provider.connection.host,
            provider.connection.port,
            topic,
            showOptions,
            defaultNoData
          );
        } else {
          emitCustomEvent(
            EVENT_OPEN_COMPONENT,
            eventOpenComponent(
              id,
              topic,
              <TopicEchoPanel
                showOptions={showOptions}
                defaultProvider={providerId}
                defaultTopic={topic}
                defaultNoData={defaultNoData}
              />,
              true,
              LAYOUT_TAB_SETS[settingsCtx.get("subscriberOpenLocation")],
              new LayoutTabConfig(
                true,
                `${CmdType.ECHO}`,
                // {
                //   type: CmdType.ECHO,
                //   providerId,
                //   topicName: topic,
                //   nodeName: "",
                //   screen: "",
                //   cmd: "",
                // },
                null,
                null,
                {
                  id: id,
                  host: provider.connection.host,
                  port: provider.connection.port,
                  topic: topic,
                  showOptions: showOptions,
                  noData: defaultNoData,
                }
              )
            )
          );
        }
      }
    },
    [getProviderById, emitCustomEvent, settingsCtx, layoutModel]
  );

  const openTerminal = useCallback(
    async (
      type: string,
      providerId: string,
      node: string,
      screen: string,
      cmd: string,
      externalKeyModifier: boolean,
      forceOpenTerminal: boolean
    ) => {
      const provider = getProviderById(providerId);
      if (provider) {
        const id = `terminal-${type}-${provider.connection.host}-${provider.connection.port}-${screen ? screen : node}`;
        // open in external window depending on setting and key modifier and if no tab already existing
        const openExternal: boolean =
          type === CmdType.SCREEN
            ? xor(settingsCtx.get("screenOpenExternal") as boolean, externalKeyModifier)
            : xor(settingsCtx.get("logOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);
        if (forceOpenTerminal) {
          try {
            // create a terminal command
            const terminalCmd = await provider.cmdForType(type, node, "", screen, cmd);
            const result = await window.commandExecutor?.execTerminal(
              provider.isLocalHost ? null : { host: provider.host() },
              `"${type.toLocaleUpperCase()} ${node}@${provider.host()}"`,
              terminalCmd.cmd
            );
            if (!result?.result) {
              logCtx.error(`Can't open external terminal on ${provider.host()}`, `${result?.message}`, true);
            }
          } catch (error) {
            logCtx.error(`Can't open external terminal on ${provider.host()}`, `${error}`, true);
          }
          return;
        }
        if (provider && window.terminalManager && (openExternal || (await window.terminalManager?.has(id)))) {
          // open in new window
          // we do not check for existing subscriber, it is done by IPC with given id
          window.terminalManager?.open(id, provider.connection.host, provider.connection.port, type, node, screen, cmd);
        } else {
          emitCustomEvent(
            EVENT_OPEN_COMPONENT,
            eventOpenComponent(
              id,
              node ? node : `${type}_${provider.connection.host}`,
              <SingleTerminalPanel
                id={id}
                type={type}
                providerId={providerId}
                nodeName={node}
                screen={screen}
                cmd={cmd}
              />,
              true,
              LAYOUT_TAB_SETS.BORDER_BOTTOM,
              new LayoutTabConfig(true, type, null, null, null, {
                id: id,
                host: provider.connection.host,
                port: provider.connection.port,
                cmdType: type,
                node: node,
                screen: screen,
                cmd: cmd,
              })
            )
          );
        }
      }
    },
    [getProviderById, emitCustomEvent, layoutModel]
  );

  // initialize ROS and system Info
  const init = async () => {
    setInitialized(() => false);

    // get local ROS Info
    if (window.rosInfo?.getInfo) {
      const rinfo = await window.rosInfo.getInfo();
      setRosInfo(rinfo);
    }
    // get local System Info
    if (window.systemInfo?.getInfo) {
      setSystemInfo(await window.systemInfo.getInfo());
    }
    setInitialized(true);
  };

  useCustomEventListener(
    EVENT_PROVIDER_DISCOVERED,
    (data: EventProviderDiscovered) => {
      // trigger add new provider
      logCtx.debug(
        `trigger add new provider: ${data.provider.rosState.name}`,
        `RosState details: ${JSON.stringify(data.provider.rosState)}`
      );
      setProvidersAddQueue((oldValue) => [...oldValue, data.provider]);
    },
    [setProvidersAddQueue]
  );

  useCustomEventListener(
    EVENT_PROVIDER_REMOVED,
    (data: EventProviderRemoved) => {
      // trigger remove provider
      logCtx.debug(
        `trigger provider removed: ${data.provider.rosState.name}`,
        `RosState details: ${JSON.stringify(data.provider.rosState)}`
      );

      setProviders((prev) =>
        prev.filter((prov) => {
          return (
            prov.discovered === undefined || // by user connected provider cannot be removed by event
            (data.provider.connection.port !== prov.connection.port &&
              data.provider.connection.host !== prov.connection.host)
          );
        })
      );
    },
    [setProviders]
  );

  /** Handle events caused by changed files. */
  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, (data: EventProviderPathEvent) => {
    if (data.path.affected.length === 0 && data.provider.className === "Provider") {
      // no affected launch files => it is a binary
      const nodes: string[] = [];
      data.provider.launchFiles.forEach((launch) => {
        launch.nodes.forEach((node) => {
          if (node.executable) {
            if (data.path.srcPath.endsWith(node.executable)) {
              if (node.node_name) {
                nodes.push(node.node_name);
              }
            }
          }
        });
      });
      if (nodes.length > 0) {
        // ask use if nodes should be restarted
        enqueueSnackbar(`Binary changed, do you want to restart ${nodes.length} nodes on: `, {
          persist: true,
          anchorOrigin: {
            vertical: "top",
            horizontal: "right",
          },
          preventDuplicate: true,
          content: (key, message) =>
            createRestartNodesAlertComponent(key, `${message}`, data.provider, nodes, data.path.srcPath),
        });
      }
    }
    data.path.affected.forEach((arg: string) => {
      enqueueSnackbar(`Do you want to reload file [${getFileName(arg)}]`, {
        persist: true,
        anchorOrigin: {
          vertical: "top",
          horizontal: "right",
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
            reloadLaunchFile
          );
        },
      });
    });
  });

  useCustomEventListener(
    EVENT_PROVIDER_ROS_NODES,
    (data: EventProviderRosNodes) => {
      // add nodes to map
      const newMap = new Map();
      const newNodeMap = new Map();
      // remove provider not in the providers list anymore
      const availableProviderKeys = providers.map((prov) => {
        return prov.id;
      });
      // remove not listed provider
      const providersRemoved: string[] = [];
      mapProviderRosNodes.forEach((nodes, provId) => {
        if (availableProviderKeys.includes(provId)) {
          newMap.set(provId, nodes);
        } else {
          providersRemoved.push(provId);
        }
      });
      nodeMap.forEach((node, id) => {
        if (!id.startsWith(data.provider.id)) {
          let removed = false;
          if (providersRemoved.length > 0) {
            // remove not used nodes fom nodeMap
            providersRemoved.forEach((provId) => {
              if (id.startsWith(provId)) {
                removed = true;
              }
            });
          }
          if (!removed) {
            newNodeMap.set(id, node);
          }
        }
      });
      newMap.set(data.provider.id, data.nodes);
      data.nodes.forEach((node) => {
        newNodeMap.set(node.idGlobal, node);
      });
      setNodeMap(newNodeMap);
      logCtx.debug(`ros nodes updated for ${data.provider.id}: ${data.nodes.length} nodes`, "");
      // logCtx.debug(`ros nodes updated for ${data.provider.id}: ${JSON.stringify(data.nodes)}`, "");

      setMapProviderRosNodes(newMap);
    },
    [providers, mapProviderRosNodes, setMapProviderRosNodes]
  );

  useCustomEventListener(EVENT_PROVIDER_SCREENS, (data: EventProviderScreens) => {
    let updated = false;
    // update screens for affected node
    data.screens.forEach((nodeScreen) => {
      const matchingNode = data.provider.rosNodes.find(
        (node) => node.id === nodeScreen.name || node.name === nodeScreen.name
      );
      if (matchingNode) {
        matchingNode.screens = nodeScreen.screens;
        updated = true;
      }
    });
    if (updated) {
      // inform listener about updated nodes
      emitCustomEvent(EVENT_PROVIDER_ROS_NODES, new EventProviderRosNodes(data.provider, data.provider.rosNodes));
    }
  });

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    const { provider, newState, oldState, details } = data;
    console.log(
      `trigger connection state ${provider.id}: new: ${newState}, old: ${oldState}, ${JSON.stringify(details)}`
    );
    switch (newState) {
      case ConnectionState.STATES.CONNECTING:
        // this state is set by provider itself while connect
        break;
      case ConnectionState.STATES.STARTING:
        // this state is set by this context while it starts nodes
        break;
      case ConnectionState.STATES.SERVER_CONNECTED:
        break;
      case ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED:
        break;
      case ConnectionState.STATES.CONNECTED:
        // trigger updates on state change
        clearProviders();
        break;
      case ConnectionState.STATES.CLOSED:
        mapProviderRosNodes.set(provider.id, []);
        clearProviders();
        break;
      case ConnectionState.STATES.AUTHZ:
      case ConnectionState.STATES.LOST:
      case ConnectionState.STATES.UNSUPPORTED:
      case ConnectionState.STATES.UNREACHABLE:
      case ConnectionState.STATES.ERRORED:
        clearProviders();
        break;
      default:
        break;
    }
  });

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data: EventProviderWarnings) => {
    logCtx.debugInterface("ros.provider.warnings", JSON.stringify(data.warnings), "", data.provider.id);
  });

  useEffect(() => {
    providers.forEach((provider) => {
      if (provider.connectionState === ConnectionState.STATES.UNKNOWN) {
        // we have no configuration => it is a discovered provider => trigger connect to the new provider
        connectToProvider(provider);
      }
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [providers]);

  /** A provider was enqueued. If it is a new one, add it to providers lists. */
  useEffect(() => {
    if (providersAddQueue.length > 0) {
      const prov = providersAddQueue.pop();
      let hostnames = prov?.hostnames;
      if (!hostnames) {
        hostnames = prov?.host() ? [prov?.host()] : [];
      }
      const provider = getProviderByHosts(hostnames, prov?.connection.port, null);
      if (provider === null) {
        if (prov) {
          setProviders((oldValue) => {
            // create new sorted list
            const newProviders = [...oldValue, prov];
            newProviders.sort((a, b) => -b.name().localeCompare(a.name()));
            return newProviders;
          });
        }
      } else {
        // provider already registered, try to connect
        if (provider.discovered !== undefined && prov?.discovered !== undefined) {
          // update discovered list
          const discoverer = prov?.discovered?.at(0);
          if (discoverer && provider.discovered.indexOf(discoverer) === -1) {
            provider.discovered.push(discoverer);
          }
        }
        connectToProvider(provider);
      }
      setProvidersAddQueue([...providersAddQueue]);
    }
  }, [providersAddQueue]);

  // Effect to initialize RosContext
  useEffect(() => {
    init();
  }, []);

  const attributesMemo = useMemo(
    () => ({
      initialized,
      rosInfo,
      systemInfo,
      providers,
      providersConnected,
      mapProviderRosNodes,
      nodeMap,
      layoutModel,
      connectToProvider,
      startProvider,
      startConfig,
      startMasterSync,
      startDynamicReconfigureClient,
      removeProvider,
      refreshProviderList,
      closeProviders,
      updateNodeList,
      updateLaunchList,
      reloadLaunchFile,
      getProviderById,
      getProviderByHost,
      getLocalProvider,
      registerSubscriber,
      unregisterSubscriber,
      updateFilterRosTopic,
      getProviderName,
      isLocalHost,
      addProvider,
      setLayoutModel,
      openEditor,
      openSubscriber,
      openTerminal,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [initialized, rosInfo, systemInfo, providers, providersConnected, mapProviderRosNodes, setMapProviderRosNodes]
  );

  return <RosContext.Provider value={attributesMemo}>{children}</RosContext.Provider>;
}

export default RosContext;
