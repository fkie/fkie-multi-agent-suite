import { SnackbarKey, useSnackbar } from "notistack";
import React, { createContext, useCallback, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { ConnectConfig } from "ssh2";

import { ErrorAlertComponent, ReloadFileAlertComponent, RestartNodesAlertComponent } from "@/renderer/components/UI";
import {
  LaunchArgument,
  LaunchLoadReply,
  LaunchLoadRequest,
  PATH_EVENT_TYPE,
  ProviderLaunchConfiguration,
  RosNode,
  RosQos,
  SubscriberFilter,
  SubscriberNode,
  URI,
  getFileName,
} from "@/renderer/models";
import ConnectionState from "@/renderer/providers/ConnectionState";
import Provider from "@/renderer/providers/Provider";
import {
  EVENT_PROVIDER_AUTH_REQUEST,
  EVENT_PROVIDER_DISCOVERED,
  EVENT_PROVIDER_NODE_BINARY_MODIFIED,
  EVENT_PROVIDER_PATH_EVENT,
  EVENT_PROVIDER_REMOVED,
  EVENT_PROVIDER_RESTART_NODES,
  EVENT_PROVIDER_ROS_NODES,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from "@/renderer/providers/eventTypes";
import {
  EventProviderAuthRequest,
  EventProviderDiscovered,
  EventProviderNodeBinaryModified,
  EventProviderPathEvent,
  EventProviderRemoved,
  EventProviderRestartNodes,
  EventProviderRosNodes,
  EventProviderState,
  EventProviderWarnings,
} from "@/renderer/providers/events";
import { TResult, TRosInfo, TSystemInfo } from "@/types";
import { useAlwaysCurrentRef } from "../hooks/useAlwaysCurrentRef";
import { useLoggingContext } from "../hooks/useLoggingContext";
import { useSettingsContext } from "../hooks/useSettingsContext";
import { RmwSelection, TProviderLaunchParams } from "../models/ProviderLaunchConfiguration";
import { LAUNCH_FILE_EXTENSIONS, getDefaultPortFromRos } from "./SettingsContext";

// ─────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────

export type TLocalNode = {
  providerId: string;
  node: string;
};

export interface IRosContext {
  initialized: boolean;
  rosInfo: TRosInfo | null;
  systemInfo: TSystemInfo | null;
  providers: Provider[];
  mapProviderRosNodes: Map<string, RosNode[]>;
  nodeMap: Map<string, RosNode>;
  localNodes: TLocalNode[];
  createProvider(host: string, rosVersion: string, port?: number, networkId?: number, useSSL?: boolean): Provider;
  connectToProvider: (provider: Provider) => Promise<boolean>;
  startProvider: (provider: Provider, forceStartWithDefault: boolean) => Promise<boolean>;
  startMasterSync: (host: string, rosVersion: string, masteruri?: string) => void;
  startDynamicReconfigureClient: (nodeName: string, masteruri: string) => Promise<TResult>;
  startConfig: (config: ProviderLaunchConfiguration, connectConfig: ConnectConfig | null) => Promise<boolean>;
  removeProvider: (providerId: string) => void;
  getProviderName: (providerId: string) => string;
  refreshProviderList: () => void;
  closeProviders: () => void;
  updateNodeList: (providerId: string) => void;
  updateLaunchList: (providerId: string) => void;
  reloadLaunchFile: (providerId: string, modifiedFile: string) => Promise<void>;
  getProviderById: (providerId: string | undefined, includeNotAvailable?: boolean) => Provider | undefined;
  getLocalProvider: () => Provider[];
  registerSubscriber: (
    provider: Provider,
    topic: string,
    messageType: string,
    filter: SubscriberFilter,
    qos: RosQos | undefined
  ) => void;
  unregisterSubscriber: (provider: Provider, topic: string) => void;
  updateFilterRosTopic: (provider: Provider, topicName: string, msg: SubscriberFilter) => void;
  isLocalHost: (host: string) => boolean;
  addProvider: (provider: Provider) => void;
  updateLocalNodes: (providerId: string, nodes: string[]) => void;
  setShowSnackbarReloadLaunchNotification: (show: boolean) => void;
  setShowSnackbarBinaryChangedNotification: (show: boolean) => void;
  hiddenConnect(configParams: TProviderLaunchParams);
}

type TLoadLaunchResult = {
  success: boolean;
  error: string;
  reply?: LaunchLoadReply | null;
};

/** Result of a single service start attempt (daemon / discovery / sync / terminal). */
type TServiceStartResult = {
  started: boolean;
  authRequested: boolean;
};

interface IRosProviderComponent {
  children: React.ReactNode;
}

// ─────────────────────────────────────────────
// Context
// ─────────────────────────────────────────────

export const RosContext = createContext<IRosContext | null>(null);

export function RosProviderReact(props: IRosProviderComponent): ReturnType<React.FC<IRosProviderComponent>> {
  const { children } = props;
  const logCtx = useLoggingContext();
  const settingsCtx = useSettingsContext();
  const { enqueueSnackbar } = useSnackbar();

  // ── State ──────────────────────────────────
  const [initialized, setInitialized] = useState(false);
  const [rosInfo, setRosInfo] = useState<TRosInfo | null>(null);
  const [systemInfo, setSystemInfo] = useState<TSystemInfo | null>(null);
  const [providersAddQueue, setProvidersAddQueue] = useState<Provider[]>([]);
  const [providers, setProviders] = useState<Provider[]>([]);
  const [localNodes, setLocalNodes] = useState<TLocalNode[]>([]);
  const [hiddenProviders, setHiddenProviders] = useState<Provider[]>([]);
  const [showSnackbarReloadLaunchNotification, setShowSnackbarReloadLaunchNotification] = useState<boolean>(true);
  const [showSnackbarBinaryChangedNotification, setShowSnackbarBinaryChangedNotification] = useState<boolean>(true);
  const [mapProviderRosNodes, setMapProviderRosNodes] = useState(new Map<string, RosNode[]>());
  const [nodeMap, setNodeMap] = useState(new Map<string, RosNode>());

  // ── Stable refs (avoids stale-closure bugs in callbacks / event handlers) ──
  const logCtxRef = useAlwaysCurrentRef(logCtx);
  const settingsCtxRef = useAlwaysCurrentRef(settingsCtx);
  const providersRef = useAlwaysCurrentRef<Provider[]>(providers);
  const systemInfoRef = useAlwaysCurrentRef<TSystemInfo | null>(systemInfo);
  const showReloadRef = useAlwaysCurrentRef(showSnackbarReloadLaunchNotification);
  const showBinaryRef = useAlwaysCurrentRef(showSnackbarBinaryChangedNotification);
  const mapProviderRosNodesRef = useAlwaysCurrentRef(mapProviderRosNodes);
  const nodeMapRef = useAlwaysCurrentRef(nodeMap);

  // ─────────────────────────────────────────────
  // Provider helpers
  // ─────────────────────────────────────────────

  /** Create a configured Provider instance. */
  const createProvider = useCallback(
    (host: string, rosVersion: string, port = 0, networkId = 0, useSSL = false): Provider => {
      return new Provider(logCtxRef, settingsCtxRef, host, rosVersion, port, networkId, useSSL);
    },
    [logCtxRef, settingsCtxRef]
  );

  /** Returns true if the given host resolves to the local machine. */
  const isLocalHost = useCallback(
    (host: string): boolean => {
      const info = systemInfoRef.current;
      if (host === "localhost" || host === info?.osInfo?.hostname) return true;
      return (info?.networkInterfaces ?? []).some((ni) => ni.ip4 === host);
    },
    [systemInfoRef]
  );

  /** Return a provider by id. By default also returns unavailable providers. */
  const getProviderById = useCallback(
    (providerId: string | undefined, includeNotAvailable = true): Provider | undefined => {
      if (!providerId) return undefined;
      return providersRef.current.find((p) => (p.isAvailable() || includeNotAvailable) && p.id === providerId);
    },
    [providersRef]
  );

  /**
   * Find a provider by hostnames + optional port.
   * If none is found and `defaultValue` is provided, enqueue it for addition.
   */
  const getProviderByHosts = useCallback(
    (hosts: string[], port = 0, defaultValue: Provider | null = null): Provider | null => {
      const found = providersRef.current.find((p) => {
        if (port !== 0 && p.connection.port !== 0 && p.connection.port !== port) return false;
        return hosts.some((h) => p.hostnames?.includes(h));
      });
      if (found) return found;
      if (defaultValue !== null) {
        setProvidersAddQueue((prev) => [...prev, defaultValue]);
      }
      return defaultValue;
    },
    [providersRef]
  );

  /** Return all providers that are on localhost. */
  const getLocalProvider = useCallback((): Provider[] => {
    return providersRef.current.filter((p) => p.isLocalHost);
  }, [providersRef]);

  /** Return the display name of a provider. */
  const getProviderName = useCallback(
    (providerId: string): string => {
      return providersRef.current.find((p) => p.id === providerId)?.name() ?? "";
    },
    [providersRef]
  );

  /** Add a provider if not already registered. */
  const addProvider = useCallback(
    (provider: Provider): void => {
      if (!getProviderById(provider.id)) {
        setProviders((prev) => [...prev, provider]);
      }
    },
    [getProviderById]
  );

  /** Remove all disconnected providers that were not created by the user. */
  const clearProviders = useCallback((): void => {
    const connectedUserIds = providersRef.current
      .filter((p) => p.isCreatedByUser() && p.connectionState === ConnectionState.STATES.CONNECTED)
      .map((p) => p.id);

    setProviders((prev) =>
      prev.filter((p) => {
        if (
          p.isCreatedByUser() ||
          p.cleanDiscoverer(connectedUserIds).length > 0 ||
          p.connectionState === ConnectionState.STATES.CONNECTED
        ) {
          return true;
        }
        p.close();
        return false;
      })
    );
  }, [providersRef]);

  /** Close the connection to all registered providers. */
  const closeProviders = useCallback((): void => {
    for (const p of providersRef.current) {
      p.close();
    }
  }, [providersRef]);

  /** Remove a provider by id and close its connection. */
  const removeProvider = useCallback(
    (providerId: string): void => {
      logCtx.debug(`Remove provider ${providerId}`, "");
      setProviders((prev) =>
        prev.filter((p) => {
          if (p.id === providerId) {
            p.close();
            return false;
          }
          return true;
        })
      );
    },
    [logCtx]
  );

  /** Update the local-node list for a given provider. */
  const updateLocalNodes = useCallback((providerId: string, nodes: string[]): void => {
    setLocalNodes((prev) => [
      ...prev.filter((ln) => ln.providerId !== providerId),
      ...nodes.map((node) => ({ providerId, node })),
    ]);
  }, []);

  // ─────────────────────────────────────────────
  // Node / launch update helpers
  // ─────────────────────────────────────────────

  /** Trigger a full node-list refresh for a provider. */
  const updateNodeList = useCallback(
    async (providerId: string, force?: boolean): Promise<void> => {
      logCtx.debug(`Triggering update of ROS nodes from ${providerId}`, "");
      const provider = getProviderById(providerId);
      if (!provider) return;
      await provider.updateRosNodes({}, force);
      await provider.updateLaunchContent();
      await provider.updateScreens();
      await provider.updateTimeDiff();
      await provider.updateDiagnostics(null);
      await provider.getLifecycle();
      await provider.getComposable();
      await provider.updateRosServices();
      await provider.updateRosTopics();
    },
    [getProviderById, logCtx]
  );

  /** Trigger a launch-file list refresh for a provider. */
  const updateLaunchList = useCallback(
    async (providerId: string): Promise<void> => {
      logCtx.debug(`Triggering update of ROS launch files from ${providerId}`, "");
      const provider = getProviderById(providerId);
      if (!provider) return;
      await provider.updateLaunchContent();
      await provider.mergeNodeStates();
    },
    [getProviderById, logCtx]
  );

  // ─────────────────────────────────────────────
  // Launch helpers
  // ─────────────────────────────────────────────

  /** Load (or reload) a launch file on the given provider. */
  const launchFile = useCallback(
    async (
      provider: Provider,
      filePath: string,
      args: LaunchArgument[],
      reload = false
    ): Promise<TLoadLaunchResult> => {
      if (!filePath) return { success: false, error: "Invalid file path" };
      if (!provider?.isAvailable()) return { success: false, error: "Invalid/unavailable provider" };

      const request = new LaunchLoadRequest(
        "",
        "",
        filePath,
        args,
        true,
        false,
        provider.rosState.masteruri ?? "",
        provider.connection.host
      );

      logCtx.debug(`launch launch file: ${JSON.stringify(request)}`);
      const reply = await provider.launchLoadFile(request, reload);

      if (!reply) {
        return {
          success: false,
          error: "Invalid response for [launchLoadFile], check DAEMON screen output",
          reply: null,
        };
      }

      switch (reply.status.code) {
        case "OK":
          logCtx.debug("load launch file was successful, update launch list..");
          return { success: true, error: reply.status.msg ?? "", reply };
        case "PARAMS_REQUIRED":
          return { success: false, error: "Please fill all arguments", reply };
        case "ERROR":
          enqueueSnackbar(`Could not load file: ${reply.status.msg}`, {
            persist: true,
            anchorOrigin: { vertical: "top", horizontal: "right" },
            preventDuplicate: true,
            content: (key, message) => (
              <ErrorAlertComponent id={key} message={message} details={`${reply.status.msg}`} />
            ),
          });
          return { success: false, error: `Reported error: ${reply.status.msg}`, reply };
        default:
          enqueueSnackbar(`Could not load file: ${reply.status.msg}`, {
            persist: true,
            anchorOrigin: { vertical: "top", horizontal: "right" },
            preventDuplicate: true,
            content: (key, message) => (
              <ErrorAlertComponent id={key} message={message} details={`${reply.status.msg}`} />
            ),
          });

          return { success: false, error: `Could not load file: ${reply.status.msg}`, reply };
      }
    },
    [logCtx]
  );

  /** Reload a launch file: unload then re-load with the last known arguments. */
  const reloadLaunchFile = useCallback(
    async (providerId: string, modifiedFile: string): Promise<void> => {
      if (!LAUNCH_FILE_EXTENSIONS.some((ext) => modifiedFile.includes(ext))) return;

      const provider = getProviderById(providerId);
      if (!provider) return;

      const filteredLaunchFile = provider.launchFiles?.find((lf) => lf.path === modifiedFile);
      const args: LaunchArgument[] = filteredLaunchFile?.args ?? [];

      const result = await launchFile(provider, modifiedFile, args, true);

      if (result.success) {
        logCtx.success(
          `Launch file [${getFileName(modifiedFile)}] reloaded`,
          `File: ${modifiedFile}`,
          `${getFileName(modifiedFile)} reloaded`
        );
        if (result.reply?.changed_nodes && result.reply.changed_nodes.length > 0) {
          emitCustomEvent(
            EVENT_PROVIDER_NODE_BINARY_MODIFIED,
            new EventProviderNodeBinaryModified(provider, result.reply.changed_nodes)
          );
          enqueueSnackbar("Configuration changed!", {
            persist: true,
            anchorOrigin: { vertical: "top", horizontal: "right" },
            preventDuplicate: true,
            content: (key, message) =>
              createRestartNodesAlertComponent(
                key,
                `${message}`,
                provider,
                result.reply?.changed_nodes ?? [],
                modifiedFile
              ),
          });
        }
      } else {
        logCtx.error(
          `Launch file error: ${result.error}`,
          `Provider: ${provider.name()} File: ${modifiedFile}`,
          "not reloaded"
        );
      }
    },
    [getProviderById, launchFile, logCtx, enqueueSnackbar]
  );

  // ─────────────────────────────────────────────
  // Subscriber helpers
  // ─────────────────────────────────────────────

  const registerSubscriber = useCallback(
    async (
      provider: Provider,
      topic: string,
      messageType: string,
      filter: SubscriberFilter,
      qos: RosQos | undefined
    ): Promise<void> => {
      if (!provider) {
        logCtx.error(`Can not start subscriber for: ${topic}`, "Provider not found", "subscriber not started");
        return;
      }
      const sNode = new SubscriberNode(topic, messageType);
      sNode.qos = qos;
      if (filter !== undefined) sNode.filter = filter;
      await provider.startSubscriber(sNode);
    },
    [logCtx]
  );

  const unregisterSubscriber = useCallback(
    async (provider: Provider, topic: string): Promise<void> => {
      if (!provider) {
        logCtx.error(`Can not stop subscriber for: ${topic}`, "Provider not found", "subscriber not unregistered");
        return;
      }
      const result = await provider.stopSubscriber(topic);
      if (result) {
        logCtx.debug(`Stopped subscriber node for '${topic}' on '${provider.name()}'`, "");
      } else {
        logCtx.error(
          `Can not stop subscriber node for: ${topic} on '${provider.name()}'`,
          `${result}`,
          "subscriber not stopped"
        );
      }
    },
    [logCtx]
  );

  const updateFilterRosTopic = useCallback(
    async (provider: Provider, topicName: string, msg: SubscriberFilter): Promise<void> => {
      if (!provider.isAvailable()) return;
      await provider.updateFilterRosTopic(topicName, msg);
    },
    []
  );

  // ─────────────────────────────────────────────
  // Connection helpers
  // ─────────────────────────────────────────────

  /** Connect to a provider (adds it to the list if not already registered). */
  const connectToProvider = useCallback(
    async (prov: Provider): Promise<boolean> => {
      const provider = getProviderByHosts(prov.hostnames, prov.connection.port, prov) as Provider;

      if (provider.connection.connected()) {
        provider.setConnectionState(ConnectionState.STATES.CONNECTED, "");
        return true;
      }

      const hintMsg =
        "Is the daemon running?\nIs the hostname being resolved to the correct IP address?\nPlease check the details in the console by pressing F12.";

      // Already available – just refresh the daemon version.
      if (provider.isAvailable()) {
        try {
          provider.setConnectionState(ConnectionState.STATES.CONNECTING, "");
          await provider.getDaemonVersion();
          provider.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          return true;
        } catch (error: unknown) {
          logCtx.debug(
            `Could not initialize provider [${provider.name()}] (${provider.type}) in [ws://${provider.connection.host}:${provider.connection.port}]`,
            `Error: ${JSON.stringify(error)}\n${hintMsg}`
          );
          provider.setConnectionState(ConnectionState.STATES.ERRORED, JSON.stringify(error));
          return false;
        }
      }

      // Full init.
      provider.setConnectionState(ConnectionState.STATES.CONNECTING, "");
      provider.isLocalHost = isLocalHost(provider.connection.host);
      try {
        if (await provider.init()) return true;
        const error = `Could not initialize provider [${provider.name()}] (${provider.type}) in [ws://${provider.connection.host}:${provider.connection.port}]`;
        logCtx.error(error, hintMsg, "connection failed");
        provider.errorDetails = error;
        provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, error);
      } catch (error: unknown) {
        logCtx.debug(
          `Could not initialize provider [${provider.name()}] (${provider.type}) in [ws://${provider.connection.host}:${provider.connection.port}]`,
          `Error: ${JSON.stringify(error)}\n${hintMsg}`
        );
        provider.setConnectionState(ConnectionState.STATES.UNREACHABLE, JSON.stringify(error));
      }
      return false;
    },
    [getProviderByHosts, isLocalHost, logCtx]
  );

  /** Refresh all user-created providers; reconnect any that have become unreachable. */
  const refreshProviderList = useCallback((): void => {
    setProviders((prev) =>
      prev.filter((p) => {
        if (!p.isCreatedByUser()) {
          p.close();
          return false;
        }
        try {
          p.updateProviderList();
          p.getDaemonVersion().catch((err) => {
            logCtx.debug(`refreshProvider ${p.name()} failed`, JSON.stringify(err));
            connectToProvider(p);
          });
        } catch (error: unknown) {
          logCtx.debug("refreshProviderList failed", JSON.stringify(error));
        }
        return true;
      })
    );
  }, [connectToProvider, logCtx]);

  // ─────────────────────────────────────────────
  // Service start helper (shared by startConfig)
  // ─────────────────────────────────────────────

  /**
   * Execute a single service start command (daemon, discovery, sync, terminal).
   * Returns whether the service started successfully and whether an auth request was emitted.
   */
  const startService = useCallback(
    async (
      credential: ConnectConfig | null,
      cmdResult: { result: boolean; message: string },
      provider: Provider,
      config: ProviderLaunchConfiguration,
      serviceLabel: string
    ): Promise<TServiceStartResult> => {
      if (!cmdResult.result) {
        logCtx.error(
          `Failed to create ${serviceLabel} start command: ${cmdResult.message}`,
          JSON.stringify(config),
          `${config.params.host} ${serviceLabel} not started`
        );
        return { started: false, authRequested: false };
      }

      const result = await window.commandExecutor?.exec(credential, cmdResult.message);
      if (!result) return { started: true, authRequested: false };

      if (!result.result) {
        if (result.connectConfig) {
          logCtx.error("Request password", JSON.stringify(result.connectConfig), "password request");
          emitCustomEvent(
            EVENT_PROVIDER_AUTH_REQUEST,
            new EventProviderAuthRequest(provider, config, result.connectConfig)
          );
          return { started: false, authRequested: true };
        }
        logCtx.error(
          `Error while starting ${serviceLabel} on host '${config.params.host}'`,
          result.message,
          `${config.params.host} ${serviceLabel} not started`
        );
        provider.setConnectionState(ConnectionState.STATES.ERRORED, result.message);
        return { started: false, authRequested: false };
      }

      logCtx.success(
        `${serviceLabel} on host '${config.params.host}' started successfully`,
        "",
        `${config.params.host} ${serviceLabel} started`
      );
      return { started: true, authRequested: false };
    },
    [logCtx]
  );

  // ─────────────────────────────────────────────
  // startConfig
  // ─────────────────────────────────────────────

  const startConfig = useCallback(
    async (config: ProviderLaunchConfiguration, connectConfig: ConnectConfig | null = null): Promise<boolean> => {
      if (!window.commandExecutor) return false;

      console.log(`config: ${JSON.stringify(config)}`);
      let allStarted = true;
      try {
        const isLocal = isLocalHost(config.params.host);
        const credential: ConnectConfig | null = isLocal ? null : (connectConfig ?? { host: config.params.host });

        const port =
          config.params.port ||
          getDefaultPortFromRos(
            Provider.defaultType,
            config.params.rosVersion,
            config.params.ros1MasterUri.uri,
            config.params.networkId
          );

        // ── Resolve / create provider ─────────────
        let provider = getProviderByHosts([config.params.host], port, null) as Provider | null;
        if (!provider) {
          provider = new Provider(
            logCtxRef,
            settingsCtxRef,
            config.params.host,
            config.params.rosVersion,
            port,
            config.params.networkId,
            config.params.useSSL
          );
          provider.isLocalHost = isLocal;
          provider.startConfiguration = config;
          if (isLocal) {
            const info = systemInfoRef.current;
            if (info?.osInfo?.hostname) provider.addHosts([info.osInfo.hostname]);
            for (const iface of info?.networkInterfaces ?? []) {
              if (iface.ip4) provider.addHosts([iface.ip4]);
              if (iface.ip6) provider.addHosts([iface.ip6]);
            }
          }
          setProvidersAddQueue((prev) => [...prev, provider as Provider]);
        }

        // Default to daemon + discovery + terminal if nothing is enabled.
        if (
          !(
            config.params.daemon.enable ||
            config.params.discovery.enable ||
            config.params.terminal.enable ||
            config.params.sync.enable
          )
        ) {
          config.params.daemon.enable = true;
          config.params.discovery.enable = true;
          config.params.terminal.enable = true;
        }

        if (config.params.daemon.enable || config.params.discovery.enable) {
          provider.setConnectionState(ConnectionState.STATES.STARTING, "");
        }

        // ── Stop running system nodes if requested ─
        const systemNodes = provider.rosNodes.filter((n) => n.system_node);
        if (config.params.force.stop && systemNodes.length > 0) {
          const pick = (key: string) => systemNodes.find((n) => n.id.includes(key));
          const nodesToStop: RosNode[] = [];

          if (config.params.rosVersion === "1") {
            if (config.params.sync.enable) {
              const n = pick("mas_sync") ?? pick("master_sync");
              if (n) nodesToStop.push(n);
            }
            if (config.params.daemon.enable) {
              const n = pick("mas/_daemon") ?? pick("node_manager_daemon");
              if (n) nodesToStop.push(n);
            }
            if (config.params.discovery.enable) {
              const n = pick("mas/_discovery") ?? pick("master_discovery");
              if (n) nodesToStop.push(n);
            }
          } else {
            if (config.params.discovery.enable) {
              const n = pick("mas/_discovery") ?? pick("master_discovery");
              if (n) nodesToStop.push(n);
            }
            if (config.params.daemon.enable) {
              const n = pick("mas/_daemon") ?? pick("node_manager_daemon");
              if (n) nodesToStop.push(n);
            }
          }

          await Promise.all(
            nodesToStop.map(async (node) => {
              logCtx.info(`Stopping running ${node.name} on host '${config.params.host}'`, "");
              await provider?.stopNode(node.id);
            })
          );
          config.params.force.stop = false;
        }

        // ── Start services ─────────────────────────
        if (config.params.daemon.enable) {
          const sr = await startService(credential, config.daemonStartCmd(), provider, config, "daemon");
          if (sr.authRequested) return false;
          if (!sr.started) allStarted = false;
        }

        if (config.params.discovery.enable) {
          const sr = await startService(credential, config.masterDiscoveryStartCmd(), provider, config, "discovery");
          if (sr.authRequested) return false;
          if (!sr.started) allStarted = false;
        }

        if (config.params.sync.enable) {
          const sr = await startService(credential, config.masterSyncStartCmd(), provider, config, "sync");
          if (sr.authRequested) return false;
          if (!sr.started) allStarted = false;
        }

        if (config.params.terminal.enable) {
          const sr = await startService(credential, config.terminalStartCmd(), provider, config, "terminal");
          if (sr.authRequested) return false;
          if (!sr.started) allStarted = false;
        }

        // Connect after giving services time to start.
        setTimeout(() => connectToProvider(provider as Provider), 2000);
      } catch (error) {
        logCtx.error(`Error starting host: ${config.params.host}`, `${error}`, `${config.params.host} start failed`);
        allStarted = false;
        const p = getProviderByHosts([config.params.host], config.params.port, null);
        if (p) {
          if (`${error}`.includes("Missing SSH credentials")) {
            p.setConnectionState(ConnectionState.STATES.AUTHZ, "start aborted");
          } else {
            p.setConnectionState(
              ConnectionState.STATES.UNREACHABLE,
              `Error starting host: ${config.params.host}: ${error}`
            );
          }
        } else {
          console.warn(`provider for host ${config.params.host}:${config.params.port} not found`);
        }
      }
      return allStarted;
    },
    [connectToProvider, getProviderByHosts, isLocalHost, logCtx, logCtxRef, settingsCtxRef, startService, systemInfoRef]
  );

  // ─────────────────────────────────────────────
  // startProvider / startMasterSync / startDynamicReconfigureClient
  // ─────────────────────────────────────────────

  const startProvider = useCallback(
    async (provider: Provider, forceStartWithDefault = true): Promise<boolean> => {
      provider.setConnectionState(ConnectionState.STATES.STARTING, "");
      if (provider.startConfiguration) {
        return startConfig(provider.startConfiguration, null);
      }
      if (!forceStartWithDefault) return false;

      const defaultCfg = new ProviderLaunchConfiguration();
      defaultCfg.params.host = provider.host();
      defaultCfg.params.rosVersion = provider.rosVersion;
      defaultCfg.params.port = provider.connection?.port;
      const domainId = provider.rosState?.ros_domain_id;
      if (domainId !== undefined) defaultCfg.params.networkId = Number.parseInt(domainId);
      defaultCfg.params.daemon.enable = true;
      defaultCfg.params.discovery.enable = true;
      defaultCfg.params.terminal.enable = true;
      defaultCfg.params.autoConnect = true;
      defaultCfg.params.autostart = false;
      defaultCfg.params.rmw.current = (provider.rosState?.rmw_implementation || "") as RmwSelection;
      defaultCfg.params.zenohConfigOverride = settingsCtx.get("zenohConfigOverride") as string;
      return startConfig(defaultCfg, null);
    },
    [startConfig]
  );

  const startMasterSync = useCallback(
    async (host: string, rosVersion: string, masteruri?: string): Promise<boolean> => {
      if (!window.commandExecutor) return false;
      const isLocal = isLocalHost(host);
      const credential = isLocal ? null : { host };
      if (!isLocal && !credential) {
        logCtx.error(`Missing SSH credentials to start master-sync on host '${host}'`, "");
        return false;
      }
      logCtx.debug(`Starting master-sync on host '${host}'`, "");
      const provider = getProviderByHosts([host]) as Provider;

      const launchCfg = new ProviderLaunchConfiguration();
      launchCfg.params.host = host;
      launchCfg.params.rosVersion = rosVersion;
      launchCfg.params.daemon.enable = false;
      launchCfg.params.discovery.enable = false;
      launchCfg.params.sync.enable = true;
      launchCfg.params.sync.doNotSync = [];
      launchCfg.params.sync.syncTopics = [];
      launchCfg.params.terminal.enable = false;
      launchCfg.params.force.start = true;
      launchCfg.params.force.stop = true;
      if (masteruri) launchCfg.params.ros1MasterUri = { enable: true, uri: masteruri };

      const cmd = launchCfg.masterSyncStartCmd();
      if (!cmd.result) return false;

      const result = await window.commandExecutor?.exec(credential, cmd.message);
      if (!result) return true;

      if (!result.result) {
        if (result.connectConfig) {
          logCtx.error("Request password", JSON.stringify(result.connectConfig));
          emitCustomEvent(
            EVENT_PROVIDER_AUTH_REQUEST,
            new EventProviderAuthRequest(provider, launchCfg, result.connectConfig)
          );
          return false;
        }
        logCtx.error(`Failed to start sync on host '${launchCfg.params.host}'`, result.message);
        return false;
      }
      return true;
    },
    [getProviderByHosts, isLocalHost, logCtx]
  );

  const startDynamicReconfigureClient = useCallback(
    async (nodeName: string, masteruri: string): Promise<TResult> => {
      if (!window.commandExecutor) return { result: false, message: "dynamic reconfigure not available in Browser" };

      logCtx.debug(`Starting Dynamic Reconfigure GUI for '${nodeName}'`, "");
      if (!masteruri) {
        const msg = `Start dynamic reconfigure failed: unknown ROS_MASTER_URI for node ${nodeName}`;
        logCtx.error(msg, "");
        return { result: false, message: msg };
      }

      const config = new ProviderLaunchConfiguration();
      config.params.rosVersion = "1";
      config.params.ros1MasterUri = { enable: true, uri: masteruri };
      const cmd = config.dynamicReconfigureClientCmd(nodeName, masteruri);
      if (!cmd.result) return { result: false, message: cmd.message };

      const result = await window.commandExecutor?.exec(null, cmd.message);
      if (!result) return { result: false, message: "no result from executor" };

      if (!result.result) {
        if (result.connectConfig) {
          logCtx.error("Request password", JSON.stringify(result.connectConfig));
          emitCustomEvent(
            EVENT_PROVIDER_AUTH_REQUEST,
            new EventProviderAuthRequest(getProviderByHosts(["localhost"]) as Provider, config, result.connectConfig)
          );
          return { result: false, message: "password requested" };
        }
        logCtx.error(`Failed to start dynamic reconfigure node on host '${config.params.host}'`, result.message);
        return { result: false, message: `Failed to start dynamic reconfigure node on host '${config.params.host}'` };
      }
      return { result: true, message: "" };
    },
    [getProviderByHosts, logCtx]
  );

  // ─────────────────────────────────────────────
  // UI alert components (stable refs via useCallback)
  // ─────────────────────────────────────────────

  const createRestartNodesAlertComponent = useCallback(
    (
      key: SnackbarKey | undefined,
      message: string,
      provider: Provider,
      changedNodes: string[],
      modifiedFile: string
    ): JSX.Element => (
      <RestartNodesAlertComponent
        id={key}
        message={message}
        provider={provider}
        nodeList={changedNodes}
        onReload={(_providerId, nodeList) => {
          const nodes = provider.rosNodes.filter((n) => nodeList.includes(n.name));
          // TODO: use modifiedFile, e.g. node.launchPath = modifiedFile
          void modifiedFile;
          emitCustomEvent(EVENT_PROVIDER_RESTART_NODES, new EventProviderRestartNodes(provider, nodes));
        }}
      />
    ),
    []
  );

  const createReloadFileAlertComponent = useCallback(
    (
      key: SnackbarKey | undefined,
      message: string,
      provider: Provider,
      modifiedFile: string,
      modification: PATH_EVENT_TYPE,
      launchFilePath: string,
      onReload: (providerId: string, modifiedFile: string) => Promise<void>
    ): JSX.Element => (
      <ReloadFileAlertComponent
        id={key}
        message={message}
        provider={provider}
        modifiedFile={modifiedFile}
        modification={modification}
        launchFile={launchFilePath}
        onReload={onReload}
        onReloaded={(_providerId: string, launchFile: string, requester: string) => {
          logCtx.success(
            `Launch file [${getFileName(launchFile)}] reloaded`,
            `Requester: ${requester}\nFile: ${launchFile}`,
            "reloaded"
          );
        }}
      />
    ),
    [logCtx]
  );

  const hiddenConnect = useCallback(
    (configParams: TProviderLaunchParams) => {
      const provider = new Provider(
        logCtxRef,
        settingsCtxRef,
        configParams.host,
        configParams.rosVersion,
        configParams.port,
        configParams.networkId,
        configParams.useSSL
      );
      provider.init();
      provider.startConfiguration = new ProviderLaunchConfiguration(configParams);
      setHiddenProviders((prev) => [...prev, provider]);
    },
    [logCtxRef, settingsCtxRef]
  );

  // ─────────────────────────────────────────────
  // Initialisation
  // ─────────────────────────────────────────────

  const init = useCallback(async (): Promise<void> => {
    setInitialized(false);
    if (window.rosInfo?.getInfo) setRosInfo(await window.rosInfo.getInfo());
    if (window.systemInfo?.getInfo) setSystemInfo(await window.systemInfo.getInfo());
    setInitialized(true);
  }, []);

  useEffect(() => {
    init();
  }, [init]);

  // ─────────────────────────────────────────────
  // Event listeners
  // ─────────────────────────────────────────────

  useCustomEventListener(EVENT_PROVIDER_DISCOVERED, (data: EventProviderDiscovered) => {
    logCtx.debug(
      `trigger add new provider: ${data.provider.rosState.name}`,
      `RosState details: ${JSON.stringify(data.provider.rosState)}`
    );
    setProvidersAddQueue((prev) => [...prev, data.provider]);
  });

  useCustomEventListener(EVENT_PROVIDER_REMOVED, (data: EventProviderRemoved) => {
    logCtx.debug(
      `trigger provider removed: ${data.provider.rosState.name}`,
      `RosState details: ${JSON.stringify(data.provider.rosState)}`
    );
    setProviders((prev) =>
      prev.filter((p) => {
        if (
          p.isCreatedByUser() ||
          (data.provider.connection.port !== p.connection.port && data.provider.connection.host !== p.connection.host)
        ) {
          return true;
        }
        p.close();
        return false;
      })
    );
  });

  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, (data: EventProviderPathEvent) => {
    // Binary changed (no affected launch files).
    if ((!data.path.affected || data.path.affected.length === 0) && data.provider.className === "Provider") {
      const nodes: string[] = [];
      for (const launch of data.provider.launchFiles) {
        for (const node of launch.nodes ?? []) {
          if (node.executable && data.path.srcPath.endsWith(node.executable) && node.node_name) {
            nodes.push(node.node_name);
          }
        }
      }
      if (showBinaryRef.current && nodes.length > 0) {
        emitCustomEvent(EVENT_PROVIDER_NODE_BINARY_MODIFIED, new EventProviderNodeBinaryModified(data.provider, nodes));
        enqueueSnackbar("Binary changed!", {
          persist: true,
          anchorOrigin: { vertical: "top", horizontal: "right" },
          preventDuplicate: true,
          content: (key, message) =>
            createRestartNodesAlertComponent(key, `${message}`, data.provider, nodes, data.path.srcPath),
        });
      }
    }
    // Affected launch files – ask to reload.
    if (showReloadRef.current) {
      for (const arg of data.path.affected ?? []) {
        enqueueSnackbar(`Do you want to reload file [${getFileName(arg)}]`, {
          persist: true,
          anchorOrigin: { vertical: "top", horizontal: "right" },
          preventDuplicate: true,
          content: (key, message) =>
            createReloadFileAlertComponent(
              key,
              `${message}`,
              data.provider,
              data.path.srcPath,
              data.path.eventType,
              arg,
              reloadLaunchFile
            ),
        });
      }
    }
  });

  useCustomEventListener(
    EVENT_PROVIDER_ROS_NODES,
    (data: EventProviderRosNodes) => {
      const currentProviders = providersRef.current;
      const availableIds = new Set(currentProviders.map((p) => p.id));
      const currentMap = mapProviderRosNodesRef.current;
      const currentNodeMap = nodeMapRef.current;

      const newMap = new Map<string, RosNode[]>();
      const newNodeMap = new Map<string, RosNode>();
      const removedIds = new Set<string>();

      // Carry over existing entries; drop providers no longer in the list.
      currentMap.forEach((nodes, provId) => {
        if (availableIds.has(provId)) {
          newMap.set(provId, nodes);
        } else {
          removedIds.add(provId);
        }
      });

      // Carry over existing node entries; skip the current provider and removed ones.
      currentNodeMap.forEach((node, id) => {
        if (!id.startsWith(data.provider.id) && ![...removedIds].some((rid) => id.startsWith(rid))) {
          newNodeMap.set(id, node);
        }
      });

      // Insert updated nodes for this provider.
      newMap.set(data.provider.id, data.nodes);
      for (const node of data.nodes) {
        newNodeMap.set(node.idGlobal, node);
      }

      setNodeMap(newNodeMap);
      setMapProviderRosNodes(newMap);
    },
    [providers]
  );

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    const { provider, newState, oldState, details } = data;
    console.log(
      `trigger connection state ${provider.id}: new: ${newState}, old: ${oldState}, ${JSON.stringify(details)}`
    );
    console.log(`STATE: ${provider.startConfiguration?.params.id}: ${newState}`);
    const hiddenProv = hiddenProviders.find((p) => {
      console.log(`  ph: ${p.startConfiguration?.params.id}`);
      console.log(`  pr: ${provider.startConfiguration?.params.id}`);
      return p.startConfiguration?.params.id === provider.startConfiguration?.params.id;
    });
    console.log(`hiddenProv: ${hiddenProv?.startConfiguration?.params.id}: ${newState}`);
    switch (newState) {
      case ConnectionState.STATES.CONNECTED:
        clearProviders();
        if (hiddenProv) {
          if (!providers.find((p) => p.hostnames.findIndex((h) => h === hiddenProv.host()) !== -1))
            setProviders((prev) => [...prev, hiddenProv]);
          setHiddenProviders((prev) =>
            prev.filter((p) => p.startConfiguration?.params.id !== provider.startConfiguration?.params.id)
          );
        }
        break;
      case ConnectionState.STATES.CLOSED:
        mapProviderRosNodesRef.current.set(provider.id, []);
        if (hiddenProv) {
          setHiddenProviders((prev) =>
            prev.filter((p) => p.startConfiguration?.params.id !== provider.startConfiguration?.params.id)
          );
        }
        clearProviders();
        break;
      case ConnectionState.STATES.AUTHZ:
      case ConnectionState.STATES.LOST:
      case ConnectionState.STATES.UNSUPPORTED:
      case ConnectionState.STATES.UNREACHABLE:
      case ConnectionState.STATES.ERRORED:
        if (hiddenProv) {
          setHiddenProviders((prev) =>
            prev.filter((p) => p.startConfiguration?.params.id !== provider.startConfiguration?.params.id)
          );
        }
        clearProviders();
        break;
      default:
        break;
    }
  });

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data: EventProviderWarnings) => {
    logCtx.debugInterface(URI.ROS_PROVIDER_WARNINGS, JSON.stringify(data.warnings), "", data.provider.id);
  });

  // ─────────────────────────────────────────────
  // Effects
  // ─────────────────────────────────────────────

  /** Auto-connect newly added providers that have no configuration yet. */
  useEffect(() => {
    for (const p of providers) {
      if (p.connectionState === ConnectionState.STATES.UNKNOWN) {
        connectToProvider(p);
      }
    }
  }, [providers, connectToProvider]);

  /** Process the provider add queue one entry at a time. */
  useEffect(() => {
    if (providersAddQueue.length === 0) return;

    // Immutably dequeue the last entry.
    setProvidersAddQueue((prev) => {
      if (prev.length === 0) return prev;
      const next = [...prev];
      const prov = next.pop();
      if (!prov) return prev;

      const hostnames = prov.hostnames?.length ? prov.hostnames : prov.host() ? [prov.host()] : [];
      const existing = getProviderByHosts(hostnames, prov.connection.port, null);

      if (existing === null) {
        setProviders((old) => {
          const updated = [...old, prov];
          updated.sort((a, b) => -b.name().localeCompare(a.name()));
          return updated;
        });
      } else {
        if (existing.isDiscovered() && prov.isDiscovered()) {
          existing.addDiscoverer(prov.discoveredBy?.at(0));
        }
        connectToProvider(existing);
      }

      return next;
    });
  }, [providersAddQueue, connectToProvider, getProviderByHosts]);

  // ─────────────────────────────────────────────
  // Memoised context value
  // ─────────────────────────────────────────────

  const attributesMemo = useMemo<IRosContext>(
    () => ({
      initialized,
      rosInfo,
      systemInfo,
      providers,
      mapProviderRosNodes,
      nodeMap,
      localNodes,
      createProvider,
      connectToProvider,
      startProvider,
      startConfig,
      startMasterSync,
      startDynamicReconfigureClient,
      removeProvider,
      getProviderName,
      refreshProviderList,
      closeProviders,
      updateNodeList,
      reloadLaunchFile,
      updateLaunchList,
      getProviderById,
      getLocalProvider,
      registerSubscriber,
      unregisterSubscriber,
      updateFilterRosTopic,
      isLocalHost,
      addProvider,
      updateLocalNodes,
      setShowSnackbarReloadLaunchNotification,
      setShowSnackbarBinaryChangedNotification,
      hiddenConnect,
    }),
    [
      initialized,
      rosInfo,
      systemInfo,
      providers,
      mapProviderRosNodes,
      nodeMap,
      localNodes,
      createProvider,
      connectToProvider,
      startProvider,
      startConfig,
      startMasterSync,
      startDynamicReconfigureClient,
      removeProvider,
      getProviderName,
      refreshProviderList,
      closeProviders,
      updateNodeList,
      reloadLaunchFile,
      updateLaunchList,
      getProviderById,
      getLocalProvider,
      registerSubscriber,
      unregisterSubscriber,
      updateFilterRosTopic,
      isLocalHost,
      addProvider,
      updateLocalNodes,
      hiddenConnect,
    ]
  );

  return <RosContext.Provider value={attributesMemo}>{children}</RosContext.Provider>;
}

export default RosContext;
