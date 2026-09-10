import { Model } from "flexlayout-react";
import React, { createContext, useCallback, useMemo, useState } from "react";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "@/renderer/components/layout";
import { emitEditorSelectRange, emitOpenComponent } from "@/renderer/components/layout/events";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { getBaseName, LaunchNodeInfo } from "@/renderer/models";
import { xor } from "@/renderer/utils/index";
import {
  CmdType,
  CmdTypes,
  TEditorConfig,
  TFileRange,
  TLaunchArg,
  TParameterRequest,
  TPublisherConfig,
  TSubscriberConfig,
} from "@/types";
import { TServiceConfig } from "@/types/ServiceManager";
import { TTerminalConfig } from "@/types/TerminalManager";
import { useSetting } from "../hooks/useSetting";
import { createEditorId } from "../monaco/utils";
import { isElectron, openBrowserSite } from "../utils/popout";

export type TNavSelection = {
  triggerId: string;
  selected: string[];
  selectedNodes: string[];
  selectedProviders: string[];
};

const NO_SELECTION = {
  triggerId: "",
  selected: [],
  selectedNodes: [],
  selectedProviders: [],
};

export type TServiceCallerProps = {
  providerId: string;
  serviceName: string | undefined;
  serviceType: string | undefined;
  externalKeyModifier: boolean;
  forceOpenTerminal: boolean;
};

export interface INavigationContext {
  selection: TNavSelection;
  setSelected: (triggerId: string, nodes: string[], addToHistory?: boolean) => void;
  history: TNavSelection[];
  setSelectedFromHistory: (triggerId: string) => TNavSelection;
  layoutModel: Model | null;
  setLayoutModel: (model: Model | null) => void;
  openEditor: (
    providerId: string,
    rootLaunch: string,
    path: string,
    fileRange: TFileRange | null,
    launchArgs: TLaunchArg[],
    topLevelLaunchArgs: TLaunchArg[],
    externalKeyModifier: boolean
  ) => void;
  openEditorForParameter: (
    providerId: string,
    nodeName: string,
    paramName: string,
    paramValue?: string,
    paramType?: string,
    externalKeyModifier?: boolean
  ) => Promise<void>;
  startPublisher: (
    providerId: string,
    topicName: string | undefined,
    topicType: string | undefined,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean
  ) => void;
  openSubscriber: (
    providerId: string,
    topic: string,
    showOptions: boolean,
    defaultNoData: boolean,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean
  ) => void;
  openServiceCaller: (args: TServiceCallerProps) => void;
  openServiceIntrospection: (args: TServiceCallerProps) => void;
  openActionSendGoal: (args: TServiceCallerProps) => void;
  openActionIntrospection: (args: TServiceCallerProps) => void;
  openTerminal: (
    type: CmdType,
    providerId: string,
    node: string,
    screen: string,
    cmd: string,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean,
    noPopout?: boolean
  ) => Promise<void>;
}

export const NavigationContext = createContext<INavigationContext | null>(null);

interface INavigationProvider {
  children: React.ReactNode;
}

export function NavigationProvider({ children }: INavigationProvider): JSX.Element {
  const logCtx = useLoggingContext();
  const rosCtx = useRosContext();
  const [editorOpenExternal] = useSetting<boolean>("editorOpenExternal");
  const [editorOpenLocation] = useSetting<string>("editorOpenLocation");
  const [publisherOpenExternal] = useSetting<boolean>("publisherOpenExternal");
  const [publisherOpenLocation] = useSetting<string>("publisherOpenLocation");
  const [subscriberOpenExternal] = useSetting<boolean>("subscriberOpenExternal");
  const [subscriberOpenLocation] = useSetting<string>("subscriberOpenLocation");
  const [serviceOpenExternal] = useSetting<boolean>("serviceOpenExternal");
  const [screenOpenExternal] = useSetting<boolean>("screenOpenExternal");
  const [logOpenExternal] = useSetting<boolean>("logOpenExternal");
  const [openAsPopout] = useSetting<boolean>("openAsPopout");

  const [selection, setSelection] = useState<TNavSelection>(NO_SELECTION);

  const [history, setHistory] = useState<TNavSelection[]>([]);
  const [layoutModel, setLayoutModel] = useState<Model | null>(null);

  const isProviderId = useCallback(
    (id: string) => {
      return !!rosCtx.getProviderById(id);
    },
    [rosCtx]
  );

  // useCallback keeps stable function references for context consumers
  const setSelected = useCallback(
    (triggerId: string, ids: string[], addToHistory = true): void => {
      setSelection((prevSelection) => {
        const providerIds = ids.filter((id) => isProviderId(id)) || [];
        const nodeIds: string[] = [];
        for (const id of ids) {
          const n = rosCtx.nodeMap.get(id);
          if (n) {
            nodeIds.push(id);
          }
        }
        if (providerIds.length === 0 && addToHistory) {
          setHistory((prev) => (prevSelection ? [...prev, prevSelection] : []));
        }
        return { triggerId: triggerId, selected: ids, selectedNodes: nodeIds, selectedProviders: providerIds };
      });
    },
    [isProviderId, rosCtx.nodeMap]
  );

  const setSelectedFromHistory = useCallback(
    (triggerId: string): TNavSelection => {
      if (!history.length) {
        setSelected(triggerId, []);
        return NO_SELECTION;
      }
      const newHistory = history.slice(0, -1);
      const last = history[history.length - 1];
      setHistory(newHistory);
      setSelected(triggerId, last.selected, false);
      return last;
    },
    [history, setSelected]
  );

  const openEditor = useCallback(
    async (
      providerId: string,
      rootLaunch: string,
      path: string,
      fileRange: TFileRange | null,
      launchArgs: TLaunchArg[],
      topLevelLaunchArgs: TLaunchArg[],
      externalKeyModifier: boolean,
      selectParameter?: TParameterRequest
    ): Promise<void> => {
      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;

      const id = createEditorId(rootLaunch, provider.id);
      const openExternal = xor(editorOpenExternal, externalKeyModifier) && !layoutModel?.getNodeById(id);

      const hasExtEditor = await window.editorManager?.has(id);
      if (hasExtEditor) {
        window.editorManager?.emitFileRange(id, path, fileRange, launchArgs, selectParameter);
        return;
      }
      const editorProps: TEditorConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        path,
        rootLaunch,
        fileRange,
        launchArgs,
        topLevelLaunchArgs,
        selectParameter,
      };

      if (openExternal && !isElectron()) {
        openBrowserSite("editor", id, editorProps, openAsPopout);
        return;
      }

      if (openExternal && window.editorManager) {
        window.editorManager.open(editorProps);
        return;
      }
      emitEditorSelectRange({
        editorId: id,
        filePath: path,
        fileRange: fileRange,
        launchArgs: launchArgs,
        selectParameter: selectParameter,
      });
      emitOpenComponent({
        id: id,
        title: getBaseName(rootLaunch),
        closable: true,
        component: LAYOUT_TABS.EDITOR,
        toNodeId: LAYOUT_TAB_SETS[editorOpenLocation],
        config: {
          contentId: { domainId: provider.connection.domainId },
          insideDomainLayout: true,
          openExternal: true,
          editorConfig: editorProps,
        },
      });
    },
    [rosCtx, editorOpenExternal, editorOpenLocation, layoutModel, openAsPopout]
  );

  /** open the launch file of a node and select/insert one of its parameters */
  const openEditorForParameter = useCallback(
    async (
      providerId: string,
      nodeName: string,
      paramName: string,
      paramValue?: string,
      paramType?: string,
      externalKeyModifier: boolean = false
    ): Promise<void> => {
      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;

      const node = provider.rosNodes.find((n) => n.name === nodeName);
      if (!node || node.launchInfo.size === 0) {
        logCtx.error(`Could not find launch file for node: [${nodeName}]`, "", "no launch files");
        return;
      }

      // Prefer the launch file recorded on the node; otherwise fall back to the
      // first available launch file that provides this node.
      // TODO: ask user if multiple launch files provide this node
      const direct = node.launchPath ? node.launchInfo.get(node.launchPath) : undefined;
      const [rootLaunch, launchInfo]: [string | null, LaunchNodeInfo | undefined] = direct
        ? [direct.launch_name, direct]
        : Array.from(node.launchInfo.entries())[0];

      if (!rootLaunch || !launchInfo) {
        logCtx.error(`Could not find launch file for node: [${nodeName}]`, "", "no launch files");
        return;
      }

      await openEditor(
        provider.id,
        rootLaunch,
        launchInfo.file_name || "",
        launchInfo.file_range,
        launchInfo.launch_context_arg || [],
        launchInfo.topLevelArgs,
        externalKeyModifier,
        {
          nodeName: node.name,
          paramName,
          paramValue,
          paramType,
          // location of the node definition - used instead of the node name lookup
          fileRange: launchInfo.file_range,
        }
      );
    },
    [rosCtx, logCtx, openEditor]
  );

  const startPublisher = useCallback(
    async (
      providerId: string,
      topicName: string | undefined,
      topicType: string | undefined,
      externalKeyModifier: boolean,
      forceOpenTerminal: boolean
    ): Promise<void> => {
      const provider = rosCtx.getProviderById(providerId) || rosCtx.getLocalProvider()[0];
      if (!provider) return;

      const topic = topicName || "";
      const type = topicType || "";
      const id = `pub-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(publisherOpenExternal, externalKeyModifier) && !layoutModel?.getNodeById(id);

      if (forceOpenTerminal) {
        try {
          const env = provider.createRosEnv();
          const terminalCmd = await provider.cmdForType(CmdTypes.PUB, "", topic, "", "", env);
          const result = await window.commandExecutor?.execTerminal(null, `"pub ${topic}"`, terminalCmd.cmd);
          if (!result?.result) {
            logCtx.error(
              `Can't start publisher in external terminal for ${topic}`,
              `${result?.message}`,
              "publisher not started"
            );
          }
        } catch (error) {
          logCtx.error(`Can't start publisher in external terminal for ${topic}`, `${error}`, "publisher not started");
        }
        return;
      }

      const publisherProps: TPublisherConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        topicName: topic,
        topicType: type,
      };

      if (openExternal && !isElectron()) {
        openBrowserSite("publisher", id, publisherProps, openAsPopout);
        return;
      }

      if (window.publishManager && (openExternal || (await window.publishManager?.has(id)))) {
        window.publishManager.start(publisherProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.TOPIC_PUBLISHER,
        toNodeId: LAYOUT_TAB_SETS[publisherOpenLocation],
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          publisherConfig: publisherProps,
        },
      });
    },
    [rosCtx, publisherOpenExternal, publisherOpenLocation, layoutModel, logCtx, openAsPopout]
  );

  const openSubscriber = useCallback(
    async (
      providerId: string,
      topic: string,
      showOptions: boolean,
      defaultNoData: boolean,
      externalKeyModifier: boolean,
      forceOpenTerminal: boolean
    ): Promise<void> => {
      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;

      const id = `echo-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(subscriberOpenExternal, externalKeyModifier) && !layoutModel?.getNodeById(id);

      if (forceOpenTerminal) {
        try {
          const env = provider.createRosEnv();
          const terminalCmd = await provider.cmdForType(CmdTypes.ECHO, "", topic, "", "", env);
          const result = await window.commandExecutor?.execTerminal(null, `"echo ${topic}"`, terminalCmd.cmd);
          if (!result?.result) {
            logCtx.error(
              `Can't open subscriber in external terminal for ${topic}`,
              `${result?.message};\nLook for details into terminal output of this GUI.`,
              "subscriber not started"
            );
          }
        } catch (error) {
          logCtx.error(`Can't open subscriber in external terminal for ${topic}`, `${error}`, "subscriber not started");
        }
        return;
      }

      const subscriberProps: TSubscriberConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        topic,
        showOptions,
        noData: defaultNoData,
      };

      if (openExternal && !isElectron()) {
        openBrowserSite("subscriber", id, subscriberProps, openAsPopout);
        return;
      }

      if (window.subscriberManager && (openExternal || (await window.subscriberManager?.has(id)))) {
        window.subscriberManager.open(subscriberProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.TOPIC_ECHO,
        toNodeId: LAYOUT_TAB_SETS[subscriberOpenLocation],
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          subscriberConfig: subscriberProps,
        },
      });
    },
    [rosCtx, subscriberOpenExternal, subscriberOpenLocation, layoutModel, logCtx, openAsPopout]
  );

  const openServiceCaller = useCallback(
    async (args: TServiceCallerProps): Promise<void> => {
      const provider = rosCtx.getProviderById(args.providerId) || rosCtx.getLocalProvider()[0];
      if (!provider) return;

      const topic = args.serviceName || "";
      const type = args.serviceType || "";
      const id = `service-call-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(serviceOpenExternal, args.externalKeyModifier) && !layoutModel?.getNodeById(id);

      // if (forceOpenTerminal) {
      //   try {
      //     const env = provider.createRosEnv();
      //     const terminalCmd = await provider.cmdForType(CmdTypes.SERVICE_CALL, "", topic, "", "", env);
      //     const result = await window.commandExecutor?.execTerminal(null, `"pub ${topic}"`, terminalCmd.cmd);
      //     if (!result?.result) {
      //       logCtx.error(
      //         `Can't call service in external terminal for ${topic}`,
      //         `${result?.message}`,
      //         "service not started"
      //       );
      //     }
      //   } catch (error) {
      //     logCtx.error(`Can't start service in external terminal for ${topic}`, `${error}`, "service not started");
      //   }
      //   return;
      // }

      const serviceProps: TServiceConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        serviceName: topic,
        serviceType: type,
        htmlName: "serviceCaller",
      };

      if (openExternal && !isElectron()) {
        openBrowserSite(serviceProps.htmlName, id, serviceProps, openAsPopout);
        return;
      }

      if (window.serviceManager && (openExternal || (await window.serviceManager?.has(id)))) {
        window.serviceManager.start(serviceProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.SERVICE_CALLER,
        toNodeId: LAYOUT_TAB_SETS.BORDER_RIGHT,
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          serviceCallerConfig: serviceProps,
        },
      });
    },
    [rosCtx, serviceOpenExternal, layoutModel, openAsPopout]
  );

  const openServiceIntrospection = useCallback(
    async (args: TServiceCallerProps): Promise<void> => {
      const provider = rosCtx.getProviderById(args.providerId) || rosCtx.getLocalProvider()[0];
      if (!provider) return;

      const topic = args.serviceName || "";
      const type = args.serviceType || "";
      const id = `service-introspection-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(serviceOpenExternal, args.externalKeyModifier) && !layoutModel?.getNodeById(id);

      const serviceProps: TServiceConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        serviceName: topic,
        serviceType: type,
        htmlName: "serviceIntrospection",
      };

      if (openExternal && !isElectron()) {
        openBrowserSite(serviceProps.htmlName, id, serviceProps, openAsPopout);
        return;
      }
      if (window.serviceManager && (openExternal || (await window.serviceManager?.has(id)))) {
        window.serviceManager.start(serviceProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.SERVICE_INTROSPECTION,
        toNodeId: LAYOUT_TAB_SETS.BORDER_RIGHT,
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          serviceIntrospectionConfig: serviceProps,
        },
      });
    },
    [rosCtx, serviceOpenExternal, layoutModel, openAsPopout]
  );

  const openActionSendGoal = useCallback(
    async (args: TServiceCallerProps): Promise<void> => {
      const provider = rosCtx.getProviderById(args.providerId) || rosCtx.getLocalProvider()[0];
      if (!provider) return;

      const topic = args.serviceName || "";
      const type = args.serviceType || "";
      const id = `action-get-goal-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(serviceOpenExternal, args.externalKeyModifier) && !layoutModel?.getNodeById(id);

      const serviceProps: TServiceConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        serviceName: topic,
        serviceType: type,
        htmlName: "actionSendGoal",
      };

      if (openExternal && !isElectron()) {
        openBrowserSite(serviceProps.htmlName, id, serviceProps, openAsPopout);
        return;
      }
      if (window.serviceManager && (openExternal || (await window.serviceManager?.has(id)))) {
        window.serviceManager.start(serviceProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.ACTION_SEND_GOAL,
        toNodeId: LAYOUT_TAB_SETS.BORDER_RIGHT,
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          actionConfig: serviceProps,
        },
      });
    },
    [rosCtx, serviceOpenExternal, layoutModel, openAsPopout]
  );

  const openActionIntrospection = useCallback(
    async (args: TServiceCallerProps): Promise<void> => {
      const provider = rosCtx.getProviderById(args.providerId) || rosCtx.getLocalProvider()[0];
      if (!provider) return;

      const topic = args.serviceName || "";
      const type = args.serviceType || "";
      const id = `action-introspection-${provider.connection.host}-${provider.connection.port}-${topic}`;
      const openExternal = xor(serviceOpenExternal, args.externalKeyModifier) && !layoutModel?.getNodeById(id);

      const serviceProps: TServiceConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        serviceName: topic,
        serviceType: type,
        htmlName: "actionIntrospection",
      };

      if (openExternal && !isElectron()) {
        openBrowserSite(serviceProps.htmlName, id, serviceProps, openAsPopout);
        return;
      }
      if (window.serviceManager && (openExternal || (await window.serviceManager?.has(id)))) {
        window.serviceManager.start(serviceProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: topic || "unknown",
        closable: true,
        component: LAYOUT_TABS.ACTION_INTROSPECTION,
        toNodeId: LAYOUT_TAB_SETS.BORDER_RIGHT,
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: true,
          actionIntrospectionConfig: serviceProps,
        },
      });
    },
    [rosCtx, serviceOpenExternal, layoutModel, openAsPopout]
  );

  const openTerminal = useCallback(
    async (
      type: CmdType,
      providerId: string,
      node: string,
      screen: string,
      cmd: string,
      externalKeyModifier: boolean,
      forceOpenTerminal: boolean,
      noPopout?: boolean
    ): Promise<void> => {
      logCtx.debug(`Start terminal ${type}@${providerId} for ${node}`);
      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;

      const id = `terminal-${type}-${provider.connection.host}-${provider.connection.port}-${screen || node}`;
      const openExternal =
        type === CmdTypes.SCREEN
          ? xor(screenOpenExternal, externalKeyModifier)
          : xor(logOpenExternal, externalKeyModifier) && !layoutModel?.getNodeById(id);

      const env = provider.createRosEnv();
      if (forceOpenTerminal) {
        try {
          const terminalCmd = await provider.cmdForType(type, node, "", screen, cmd, env);
          const result = await window.commandExecutor?.execTerminal(
            provider.isLocalHost ? null : { host: provider.host() },
            `"${type.toLocaleUpperCase()} ${node}@${provider.host()}"`,
            terminalCmd.cmd
          );
          if (!result?.result) {
            logCtx.error(
              `Can't open external terminal on ${provider.host()}`,
              `${result?.message}`,
              "no external terminal"
            );
          }
        } catch (error) {
          logCtx.error(`Can't open external terminal on ${provider.host()}`, `${error}`, "no external terminal");
        }
        return;
      }

      const terminalProps: TTerminalConfig = {
        id,
        providerId: provider.id,
        host: provider.connection.host,
        port: provider.connection.port,
        cmdType: type,
        node,
        screen,
        cmd,
        env,
      };
      if (openExternal && !isElectron()) {
        openBrowserSite("terminal", id, terminalProps, openAsPopout);
        return;
      }

      if (window.terminalManager && (openExternal || (await window.terminalManager?.has(id)))) {
        window.terminalManager.open(terminalProps);
        return;
      }

      emitOpenComponent({
        id: id,
        title: node || `${type}_${provider.connection.host}`,
        closable: true,
        component: LAYOUT_TABS.TERMINAL,
        toNodeId: LAYOUT_TAB_SETS.BORDER_BOTTOM,
        config: {
          insideDomainLayout: true,
          contentId: { domainId: provider.connection.domainId },
          openExternal: !noPopout,
          terminalType: type,
          terminalConfig: terminalProps,
        },
      });
    },
    [rosCtx, layoutModel, logCtx, logOpenExternal, screenOpenExternal, openAsPopout]
  );

  const value = useMemo<INavigationContext>(
    () => ({
      selection,
      setSelected,
      history,
      setSelectedFromHistory,
      layoutModel,
      setLayoutModel,
      openEditor,
      openEditorForParameter,
      openSubscriber,
      openServiceCaller,
      openServiceIntrospection,
      openActionSendGoal,
      openActionIntrospection,
      openTerminal,
      startPublisher,
    }),
    [
      selection,
      setSelected,
      history,
      setSelectedFromHistory,
      layoutModel,
      openEditor,
      openEditorForParameter,
      openSubscriber,
      openServiceCaller,
      openServiceIntrospection,
      openActionSendGoal,
      openActionIntrospection,
      openTerminal,
      startPublisher,
    ]
  );

  return <NavigationContext.Provider value={value}>{children}</NavigationContext.Provider>;
}

export default NavigationContext;
