import { Model } from "flexlayout-react";
import React, { createContext, useCallback, useMemo, useState } from "react";

import { getBaseName } from "@/renderer/models";
import {
  EVENT_EDITOR_SELECT_RANGE,
  EVENT_OPEN_COMPONENT,
  eventEditorSelectRange,
  eventOpenComponent,
} from "@/renderer/pages/NodeManager/layout/events";
import FileEditorPanel from "@/renderer/pages/NodeManager/panels/FileEditorPanel";
import { xor } from "@/renderer/utils/index";
import { TFileRange, TLaunchArg } from "@/types";
import { emitCustomEvent } from "react-custom-events";
import { useLoggingContext } from "../hooks/useLoggingContext";
import { useRosContext } from "../hooks/useRosContext";
import { useSettingsContext } from "../hooks/useSettingsContext";
import { createEditorId } from "../monaco/utils";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../pages/NodeManager/layout";
import SingleTerminalPanel from "../pages/NodeManager/panels/SingleTerminalPanel";
import TopicEchoPanel from "../pages/NodeManager/panels/TopicEchoPanel";
import TopicPublishPanel from "../pages/NodeManager/panels/TopicPublishPanel";
import { CmdType } from "../providers";

export interface INavigationContext {
  selectedNodes: string[];
  setSelectedNodes: (nodes: string[], addToHistory?: boolean) => void;
  nodesHistory: string[][];
  setSelectedFromHistory: () => string[];
  selectedProviders: string[];
  setSelectedProviders: (providers: string[], forceUpdate?: boolean) => void;
  layoutModel: Model | null;
  setLayoutModel: (model: Model | null) => void;
  openEditor: (
    providerId: string,
    rootLaunch: string,
    path: string,
    fileRange: TFileRange | null,
    launchArgs: TLaunchArg[],
    externalKeyModifier: boolean
  ) => void;
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

function arraysEqual(a: string[], b: string[]): boolean {
  if (a === b) return true;
  if (a.length !== b.length) return false;
  return a.every((v, i) => v === b[i]);
}

export function NavigationProvider({ children }: INavigationProvider): JSX.Element {
  const logCtx = useLoggingContext();
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  const [selectedNodes, _setSelectedNodes] = useState<string[]>([]);
  const [nodesHistory, setNodesHistory] = useState<string[][]>([]);
  const [selectedProviders, _setSelectedProviders] = useState<string[]>([]);
  const [layoutModel, setLayoutModel] = useState<Model | null>(null);

  // useCallback keeps stable function references for context consumers
  const setSelectedProviders = useCallback(
    (providers: string[], forceUpdate = false): void => {
      if (forceUpdate || !arraysEqual(selectedProviders, providers)) {
        _setSelectedProviders(providers);
      }
    },
    [selectedProviders]
  );

  const setSelectedNodes = useCallback(
    (nodes: string[], addToHistory = true): void => {
      if (!arraysEqual(selectedNodes, nodes)) {
        setNodesHistory((prev) => (addToHistory ? [...prev, selectedNodes] : []));
        _setSelectedNodes(nodes);
      }
    },
    [selectedNodes]
  );

  const setSelectedFromHistory = useCallback((): string[] => {
    if (!nodesHistory.length) {
      _setSelectedNodes([]);
      return [];
    }
    const newHistory = nodesHistory.slice(0, -1);
    const last = nodesHistory[nodesHistory.length - 1];
    setNodesHistory(newHistory);
    _setSelectedNodes(last);
    return last;
  }, [nodesHistory]);

  const openEditor = useCallback(
    async (
      providerId: string,
      rootLaunch: string,
      path: string,
      fileRange: TFileRange | null,
      launchArgs: TLaunchArg[],
      externalKeyModifier: boolean
    ): Promise<void> => {
      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;

      const id = createEditorId(rootLaunch, provider.id);
      const openExternal =
        xor(settingsCtx.get("editorOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);

      const hasExtEditor = await window.editorManager?.has(id);
      if (hasExtEditor) {
        window.editorManager?.emitFileRange(id, path, fileRange, launchArgs);
        return;
      }

      if (openExternal && window.editorManager) {
        window.editorManager.open(
          id,
          provider.connection.host,
          provider.connection.port,
          path,
          rootLaunch,
          fileRange,
          launchArgs
        );
        return;
      }

      emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(id, path, fileRange, launchArgs));
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          getBaseName(rootLaunch),
          <FileEditorPanel
            editorId={id}
            provider={provider}
            currentFilePath={path}
            rootFilePath={rootLaunch}
            fileRange={fileRange}
            launchArgs={launchArgs}
          />,
          true,
          LAYOUT_TAB_SETS[settingsCtx.get("editorOpenLocation") as string],
          new LayoutTabConfig(true, "editor", null, {
            id,
            host: provider.connection.host,
            port: provider.connection.port,
            rootLaunch,
            path,
            fileRange,
            launchArgs,
          })
        )
      );
    },
    [rosCtx, settingsCtx, layoutModel]
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
      const openExternal =
        xor(settingsCtx.get("publisherOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);

      if (forceOpenTerminal) {
        try {
          const env = provider.startConfiguration?.getEnv() || [];
          const terminalCmd = await provider.cmdForType(CmdType.PUB, "", topic, "", "", env);
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

      if (window.publishManager && (openExternal || (await window.publishManager?.has(id)))) {
        window.publishManager.start(id, provider.connection.host, provider.connection.port, topic, type);
        return;
      }

      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          topic || "unknown",
          <TopicPublishPanel topicName={topicName} topicType={topicType} providerId={providerId} />,
          true,
          LAYOUT_TAB_SETS[settingsCtx.get("publisherOpenLocation") as string],
          new LayoutTabConfig(true, `${CmdType.PUB}`, null, null, null, null, {
            id,
            host: provider.connection.host,
            port: provider.connection.port,
            topicName: topic,
            topicType: type,
          })
        )
      );
    },
    [rosCtx, settingsCtx, layoutModel, logCtx]
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
      const openExternal =
        xor(settingsCtx.get("subscriberOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);

      if (forceOpenTerminal) {
        try {
          const env = provider.startConfiguration?.getEnv() || [];
          const terminalCmd = await provider.cmdForType(CmdType.ECHO, "", topic, "", "", env);
          const result = await window.commandExecutor?.execTerminal(null, `"echo ${topic}"`, terminalCmd.cmd);
          if (!result?.result) {
            logCtx.error(
              `Can't open subscriber in external terminal for ${topic}`,
              `${result?.message}`,
              "subscriber not started"
            );
          }
        } catch (error) {
          logCtx.error(`Can't open subscriber in external terminal for ${topic}`, `${error}`, "subscriber not started");
        }
        return;
      }

      if (window.subscriberManager && (openExternal || (await window.subscriberManager?.has(id)))) {
        window.subscriberManager.open(
          id,
          provider.connection.host,
          provider.connection.port,
          topic,
          showOptions,
          defaultNoData
        );
        return;
      }

      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          topic,
          <TopicEchoPanel
            showOptions={showOptions}
            provider={provider}
            defaultTopic={topic}
            defaultNoData={defaultNoData}
          />,
          true,
          LAYOUT_TAB_SETS[settingsCtx.get("subscriberOpenLocation") as string],
          new LayoutTabConfig(true, `${CmdType.ECHO}`, null, null, {
            id,
            host: provider.connection.host,
            port: provider.connection.port,
            topic,
            showOptions,
            noData: defaultNoData,
          })
        )
      );
    },
    [rosCtx, settingsCtx, layoutModel, logCtx]
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
        type === CmdType.SCREEN
          ? xor(settingsCtx.get("screenOpenExternal") as boolean, externalKeyModifier)
          : xor(settingsCtx.get("logOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);

      const env = provider.startConfiguration?.getEnv() || [];
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

      if (window.terminalManager && (openExternal || (await window.terminalManager?.has(id)))) {
        window.terminalManager.open(
          id,
          provider.connection.host,
          provider.connection.port,
          `${type}`,
          node,
          screen,
          cmd,
          env
        );
        return;
      }

      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          node || `${type}_${provider.connection.host}`,
          <SingleTerminalPanel
            id={id}
            type={type}
            provider={provider}
            nodeName={node}
            screen={screen}
            cmd={cmd}
            env={env}
          />,
          true,
          LAYOUT_TAB_SETS.BORDER_BOTTOM,
          noPopout
            ? undefined
            : new LayoutTabConfig(true, type, null, null, null, {
                id,
                host: provider.connection.host,
                port: provider.connection.port,
                cmdType: type,
                node,
                screen,
                cmd,
                env,
              })
        )
      );
    },
    [rosCtx, settingsCtx, layoutModel, logCtx]
  );

  const value = useMemo<INavigationContext>(
    () => ({
      selectedNodes,
      setSelectedNodes,
      nodesHistory,
      setSelectedFromHistory,
      selectedProviders,
      setSelectedProviders,
      layoutModel,
      setLayoutModel,
      openEditor,
      openSubscriber,
      openTerminal,
      startPublisher,
    }),
    [
      selectedNodes,
      nodesHistory,
      selectedProviders,
      layoutModel,
      setSelectedFromHistory,
      setSelectedNodes,
      setSelectedProviders,
      openEditor,
      openSubscriber,
      openTerminal,
      startPublisher,
    ]
  );

  return <NavigationContext.Provider value={value}>{children}</NavigationContext.Provider>;
}

export default NavigationContext;
