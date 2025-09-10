import { Model } from "flexlayout-react";
import React, { createContext, useContext, useMemo, useState } from "react";

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
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../pages/NodeManager/layout";
import SingleTerminalPanel from "../pages/NodeManager/panels/SingleTerminalPanel";
import TopicEchoPanel from "../pages/NodeManager/panels/TopicEchoPanel";
import TopicPublishPanel from "../pages/NodeManager/panels/TopicPublishPanel";
import { CmdType } from "../providers";
import LoggingContext from "./LoggingContext";
import RosContext from "./RosContext";
import { SettingsContext } from "./SettingsContext";

export interface INavigationContext {
  selectedNodes: string[];
  setSelectedNodes: (nodes: string[], addToHistory: boolean) => void;
  nodesHistory: string[][];
  setSelectedFromHistory: () => string[];
  selectedProviders: string[];
  setSelectedProviders: (providers: string[]) => void;
  layoutModel: Model | null;
  setLayoutModel: (model: Model) => void;
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
    forceOpenTerminal: boolean
  ) => Promise<void>;
}

export const DEFAULT = {
  selectedNodes: [],
  nodesHistory: [],
  selectedProviders: [],
  modifiedFiles: [],
  layoutModel: null,
  setLayoutModel: (): void => {},
  setSelectedNodes: (): void => {},
  setSelectedFromHistory: (): string[] => {
    return [];
  },
  setSelectedProviders: (): void => {},
  openEditor: (): void => {},
  openSubscriber: (): void => {},
  openTerminal: (): Promise<void> => {
    return Promise.resolve();
  },
  startPublisher: (): void => {},
};

interface INavigationProvider {
  children: React.ReactNode;
}

export const NavigationContext = createContext<INavigationContext>(DEFAULT);

export function NavigationProvider({ children }: INavigationProvider): ReturnType<React.FC<INavigationProvider>> {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [selectedNodes, _setSelectedNodes] = useState<string[]>(DEFAULT.selectedNodes);
  const [nodesHistory, setNodesHistory] = useState<string[][]>([]);
  const [selectedProviders, setSelectedProviders] = useState<string[]>(DEFAULT.selectedProviders);
  const [layoutModel, setLayoutModel] = useState<Model | null>(null);

  const setSelectedNodes: (nodes: string[], addToHistory: boolean) => void = (nodes, addToHistory = true) => {
    if (JSON.stringify(selectedNodes) !== JSON.stringify(nodes)) {
      if (addToHistory) {
        setNodesHistory((prev) => [...prev, selectedNodes]);
      } else {
        setNodesHistory([]);
      }
      _setSelectedNodes(nodes);
    }
  };

  const setSelectedFromHistory: () => string[] = () => {
    const result = nodesHistory.pop();
    setNodesHistory([...nodesHistory]);
    _setSelectedNodes(result || []);
    return result || [];
  };

  async function openEditor(
    providerId: string,
    rootLaunch: string,
    path: string,
    fileRange: TFileRange | null,
    launchArgs: TLaunchArg[],
    externalKeyModifier: boolean
  ): Promise<void> {
    const provider = rosCtx.getProviderById(providerId);
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
            LAYOUT_TAB_SETS[settingsCtx.get("editorOpenLocation") as string],
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
  }

  async function startPublisher(
    providerId: string,
    topicName: string | undefined,
    topicType: string | undefined,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean
  ): Promise<void> {
    const provider = rosCtx.getProviderById(providerId);
    if (provider) {
      const id = `pub-${provider.connection.host}-${provider.connection.port}-${topicName}`;
      // open in external window depending on setting and key modifier and if no tab already existing
      const openExternal: boolean =
        xor(settingsCtx.get("publisherOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);
      if (forceOpenTerminal) {
        try {
          const terminalCmd = await provider.cmdForType(CmdType.PUB, "", topicName || "", "", "");
          const result = await window.commandExecutor?.execTerminal(
            null, // we start the publisher always local
            `"pub ${topicName}"`,
            terminalCmd.cmd
          );
          if (!result?.result) {
            logCtx.error(`Can't start publisher in external terminal for ${topicName}`, `${result?.message}`, true);
          }
        } catch (error) {
          logCtx.error(`Can't start publisher in external terminal for ${topicName}`, `${error}`, true);
        }
        return;
      }
      if (provider && window.publishManager && (openExternal || (await window.publishManager?.has(id)))) {
        // open in new window
        // we do not check for existing publisher, it is done by IPC with given id
        window.publishManager?.start(
          id,
          provider.connection.host,
          provider.connection.port,
          topicName || "",
          topicType || ""
        );
      } else {
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            id,
            topicName || "unknown",
            <TopicPublishPanel topicName={topicName} topicType={topicType} providerId={providerId} />,
            true,
            LAYOUT_TAB_SETS[settingsCtx.get("publisherOpenLocation") as string],
            new LayoutTabConfig(true, `${CmdType.PUB}`, null, null, null, null, {
              id: id,
              host: provider.connection.host,
              port: provider.connection.port,
              topicName: topicName || "",
              topicType: topicType || "",
            })
          )
        );
      }
    }
  }

  async function openSubscriber(
    providerId: string,
    topic: string,
    showOptions: boolean,
    defaultNoData: boolean,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean
  ): Promise<void> {
    const provider = rosCtx.getProviderById(providerId);
    if (provider) {
      const id = `echo-${provider.connection.host}-${provider.connection.port}-${topic}`;
      // open in external window depending on setting and key modifier and if no tab already existing
      const openExternal: boolean =
        xor(settingsCtx.get("subscriberOpenExternal") as boolean, externalKeyModifier) && !layoutModel?.getNodeById(id);
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
            LAYOUT_TAB_SETS[settingsCtx.get("subscriberOpenLocation") as string],
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
  }

  async function openTerminal(
    type: CmdType,
    providerId: string,
    node: string,
    screen: string,
    cmd: string,
    externalKeyModifier: boolean,
    forceOpenTerminal: boolean
  ): Promise<void> {
    const provider = rosCtx.getProviderById(providerId);
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
        window.terminalManager?.open(
          id,
          provider.connection.host,
          provider.connection.port,
          `${type}`,
          node,
          screen,
          cmd
        );
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
  }

  const attributesMemo = useMemo(
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
    [nodesHistory, selectedNodes, selectedProviders, setSelectedFromHistory]
  );

  return <NavigationContext.Provider value={attributesMemo}>{children}</NavigationContext.Provider>;
}

export default NavigationContext;
