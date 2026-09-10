import RefreshIcon from "@mui/icons-material/Refresh";
import StopIcon from "@mui/icons-material/Stop";
import {
  Alert,
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  FormLabel,
  IconButton,
  LinearProgress,
  Paper,
  Stack,
  Tooltip,
} from "@mui/material";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import HostTreeView from "@/renderer/components/HostTreeView/HostTreeView";
import HostTreeViewActions from "@/renderer/components/HostTreeView/HostTreeViewActions";
import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "@/renderer/components/layout";
import {
  emitOpenComponent,
  EVENT_FILTER_NODES,
  EVENT_KILL_NODES,
  EVENT_SHOW_SCREENS,
  TEventId,
  TEventKillNodes,
  TEventShowScreens,
} from "@/renderer/components/layout/events";
import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import ListSelectionModal from "@/renderer/components/SelectionModal/ListSelectionModal";
import MapSelectionModal, { MapSelectionItem } from "@/renderer/components/SelectionModal/MapSelectionModal";
import { DraggablePaper } from "@/renderer/components/UI";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import useQueue from "@/renderer/hooks/useQueue";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { Result, RosNode, RosNodeStatus } from "@/renderer/models";
import { ConnectionState, EventProviderRestartNodes } from "@/renderer/providers/events";
import { EVENT_PROVIDER_RESTART_NODES } from "@/renderer/providers/eventTypes";
import { TResultClearPath } from "@/renderer/providers/ProviderConnection";
import { findIn } from "@/renderer/utils/index";
import { CmdType, CmdTypes, TFileRange } from "@/types";
import { contentToId, TContentId } from "../../../components/layout/LayoutTabConfig";

type TProviderNodes = {
  providerId: string;
  nodes: RosNode[];
};

type QueueActionType = "STOP" | "START" | "KILL" | "UNREGISTER" | "CLEAR_LOG" | "DYNAMIC_RECONFIGURE";

interface TQueueAction {
  action: QueueActionType;
  node?: RosNode;
  service?: string;
  masteruri?: string;
}

type TMenuOptionsParam = {
  name: string;
  callback: () => void;
};

type TMenuOptionsNode = {
  node: RosNode;
  callback: () => void;
};

type TMenuOptionsEditor = {
  node: RosNode;
  external: boolean;
};

type TMenuOptionsScreen = {
  nodeName: string;
  providerId: string;
  screen: string;
  external?: boolean;
  callback?: () => void;
};

type TPendingRestart = {
  nodes: RosNode[];
  onlyWithLaunch: boolean;
  ignoreTimer: boolean;
  notBefore: number; // timestamp in ms since epoch
};

const queueActionMeta: Record<QueueActionType, { successText: string }> = {
  STOP: { successText: "stopped" },
  START: { successText: "started" },
  KILL: { successText: "killed" },
  UNREGISTER: { successText: "unregistered" },
  CLEAR_LOG: { successText: "logs cleared" },
  DYNAMIC_RECONFIGURE: { successText: "dynamic reconfigure started" },
};

interface HostTreeViewPanelProps {
  contentId?: TContentId;
}

export default function HostTreeViewPanel(props: HostTreeViewPanelProps): JSX.Element {
  const { contentId } = props;
  // context objects
  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();

  // state variables
  const [showButtonsForKeyModifiers] = useSetting<boolean>("showButtonsForKeyModifiers");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [buttonLocation] = useSetting<string>("buttonLocation");
  const [nodeLoggerOpenLocation] = useSetting<string>("nodeLoggerOpenLocation");
  const [nodeParamOpenLocation] = useSetting<string>("nodeParamOpenLocation");
  const [ignoreProcessesOnShutdown] = useSetting<string>("ignoreProcessesOnShutdown");

  const [filterText, setFilterText] = useState<string>("");
  const [providerNodes, setProviderNodes] = useState<TProviderNodes[]>([]);
  const [selectedProviderItems, setSelectedProviderItems] = useState<string[]>([]);
  const [selectedNodeItems, setSelectedNodeItems] = useState<string[]>([]);
  const [nodesToStart, setNodesToStart] = useState<RosNode[]>();
  const [progressQueueMain, setProgressQueueMain] = useState<number>(0);
  const queue = useQueue<TQueueAction>(setProgressQueueMain);

  // variables with show dialog actions
  const [shutdownRos, setShutdownRos] = useState<"" | "only nodes" | "kill ros2">("");
  const [rosCleanPurge, setRosCleanPurge] = useState<boolean>(false);
  const [nodeScreens, setNodeScreens] = useState<TMenuOptionsScreen[]>([]);
  const [nodeParams, setNodeParams] = useState<TMenuOptionsParam[]>([]);
  const [nodeLogs, setNodeLogs] = useState<TMenuOptionsNode[]>([]);
  const [nodeLoggers, setNodeLoggers] = useState<TMenuOptionsNode[]>([]);
  const [nodeMultiLaunches, setNodeMultiLaunches] = useState<RosNode[]>([]);
  const [dynamicReconfigureItems, setDynamicReconfigureItems] = useState<RosNode[]>([]);
  const [nodesAwaitModal, setNodesAwaitModal] = useState<RosNode[]>([]);
  const [editNodeWithMultipleLaunchInfos, setEditNodeWithMultipleLaunchInfos] = useState<TMenuOptionsEditor>();
  const [pendingRestart, setPendingRestart] = useState<TPendingRestart | null>(null);
  const [killProcessQuestion, setKillProcessQuestion] = useState<
    { node: string; pid: number; cmdLine: string; callback?: () => Promise<void> }[]
  >([]);
  // remember last processed queue index to avoid double execution (e.g. in StrictMode)
  const lastProcessedIndexRef = useRef<number | null>(null);
  const pendingKillNodesRef = useRef<RosNode[]>([]);
  // pending SIGKILL timers, must be cancelled on clear/unmount
  const killTimersRef = useRef<number[]>([]);
  // guards against re-processing the same batch when an effect re-runs
  const handledStartBatchRef = useRef<RosNode[] | null>(null);
  const handledRestartRef = useRef<TPendingRestart | null>(null);

  /** Synchronous check whether an action for a node is already queued. */
  const isQueued = useCallback(
    (action: QueueActionType, nodeName: string): boolean => {
      return queue.has((item) => item.action === action && item.node?.name === nodeName);
    },
    [queue]
  );

  /** Central enqueue point – single place for logging and de-duplication. */
  const enqueue = useCallback(
    (items: TQueueAction[], source: string): void => {
      if (items.length === 0) return;
      console.log(
        `[queue] enqueue from '${source}': ${items.map((i) => `${i.action}:${i.node?.name ?? i.service ?? "?"}`).join(", ")}`
      );
      queue.update(items);
    },
    [queue]
  );

  /** Remove nodes from the pending SIGKILL list (e.g. stopped successfully or restarted). */
  const removePendingKill = useCallback((nodes: RosNode[]): void => {
    const ids = new Set(nodes.map((n) => n.id));
    pendingKillNodesRef.current = pendingKillNodesRef.current.filter((n) => !ids.has(n.id));
  }, []);

  /** Cancel all pending SIGKILL timers. */
  const cancelKillTimers = useCallback((): void => {
    for (const timerId of killTimersRef.current) {
      window.clearTimeout(timerId);
    }
    killTimersRef.current = [];
    pendingKillNodesRef.current = [];
  }, []);

  /**
   * Get list of nodes from a list of node.idGlobal

   */
  const getNodesFromIds = useCallback(
    (itemIds: string[]): RosNode[] => {
      const nodeList: RosNode[] = [];
      for (const item of itemIds) {
        const node = rosCtx.nodeMap.get(item);
        if (node) {
          nodeList.push(node);
        }
      }
      return nodeList;
    },
    [rosCtx.nodeMap]
  );

  const getSelectedNodes = useCallback((): RosNode[] => {
    return getNodesFromIds(selectedNodeItems);
  }, [selectedNodeItems, getNodesFromIds]);

  /**
   * Compute visibleNodes and countFilteredNodes based on providerNodes and filterText.
   * This replaces explicit state for these derived values.

   */
  const { visibleNodesGlobal, countFilteredNodes } = useMemo(() => {
    let nodeCount = 0;
    let nodeFilteredCount = 0;
    const newVisibleNodes: RosNode[] = [];

    for (const item of providerNodes) {
      const { nodes } = item;
      nodeCount += nodes.length;

      const filteredNodes = nodes.filter((node) => {
        if (filterText.length === 0) {
          return true;
        }
        return findIn(filterText, [node.name, node.group, node.providerName, node.guid || ""]);
      });

      nodeFilteredCount += filteredNodes.length;

      // remove nodes which are local in remote hosts
      for (const node of filteredNodes) {
        const isLocal = node.isLocalRunningNode() || node.launchInfo.size > 0;
        const existsAsRemoteLocal =
          rosCtx.localNodes.filter((lNode) => lNode.node === node.name && lNode.providerId !== node.providerId).length >
          0;

        if (isLocal || !existsAsRemoteLocal) {
          newVisibleNodes.push(node);
        }
      }
    }

    return {
      visibleNodesGlobal: newVisibleNodes,
      countFilteredNodes: nodeCount - nodeFilteredCount,
    };
  }, [providerNodes, filterText]);

  // Event listeners -----------------------------------------------------------------------------------

  // keep latest implementations for event handlers registered once
  const latestHandlersRef = useRef({ restartNodes, createSingleTerminalPanel });
  useEffect(() => {
    latestHandlersRef.current = { restartNodes, createSingleTerminalPanel };
  });

  const onProviderRestartNodes = useCallback((data: EventProviderRestartNodes): void => {
    latestHandlersRef.current.restartNodes(data.nodes, true);
  }, []);

  const onShowScreens = useCallback((data: TEventShowScreens): void => {
    for (const node of data.nodes) {
      for (const screen of node.screens || []) {
        latestHandlersRef.current.createSingleTerminalPanel(
          CmdTypes.SCREEN,
          node.providerId,
          node.name,
          screen,
          false,
          false
        );
      }
    }
  }, []);

  const onFilterNodes = useCallback((data: TEventId): void => {
    setFilterText(data.id);
  }, []);

  const onKillNodes = useCallback(
    (data: TEventKillNodes): void => {
      enqueue(
        data.nodes.map((node) => {
          return { node, action: "KILL" };
        }),
        "event:KILL"
      );
    },
    [enqueue]
  );

  useCustomEventListener(EVENT_PROVIDER_RESTART_NODES, onProviderRestartNodes);
  useCustomEventListener(EVENT_FILTER_NODES, onFilterNodes);
  useCustomEventListener(EVENT_KILL_NODES, onKillNodes);
  useCustomEventListener(EVENT_SHOW_SCREENS, onShowScreens);

  // Register Callbacks ----------------------------------------------------------------------------------

  /**
   * Create and open a new panel with a [NodeLoggerPanel] for a given node

   */
  const createLoggerPanel = useCallback(
    (node: RosNode): void => {
      const id = `node-logger-${node.idGlobal}`;
      const title = node.name;
      emitOpenComponent({
        id: id,
        title: title,
        closable: true,
        component: LAYOUT_TABS.NODE_LOGGER,
        toNodeId: LAYOUT_TAB_SETS[nodeLoggerOpenLocation],
        config: { nodeLoggerConfig: { id, node } },
      });
    },
    [nodeLoggerOpenLocation]
  );

  /**
   * Create and open a new panel with a [NodeLoggerPanel] for selected nodes

   */
  const createLoggerPanelFromId = useCallback(
    (itemIds: string[]): void => {
      const nodeList = getNodesFromIds(itemIds);
      if (nodeList.length > 0) {
        createLoggerPanel(nodeList[0]);
      }
    },
    [createLoggerPanel, getNodesFromIds]
  );

  /**
   * Create and open a new panel with a [SingleTerminalPanel] for a given node

   */
  async function createSingleTerminalPanel(
    type: CmdType,
    providerId: string,
    nodeName: string,
    screen: string,
    externalKeyModifier: boolean = false,
    openInTerminal: boolean = false,
    noPouout: boolean = false
  ): Promise<void> {
    return navCtx.openTerminal(type, providerId, nodeName, screen, "", externalKeyModifier, openInTerminal, noPouout);
  }

  /**
   * Create and open a new panel with a file editor for selected nodes

   */
  const createFileEditorPanel = useCallback(
    (nodes: RosNode[], external: boolean): void => {
      if (nodes.length === 0) return;

      const node = nodes[0];
      if (node.launchInfo.size === 0) {
        logCtx.error(
          `Could not find launch file for node: [${node.name}]`,
          `Node Info: ${JSON.stringify(node)}`,
          "no launch files"
        );
        return;
      }
      const launchInfos = Array.from(node.launchInfo.entries());
      if (launchInfos.length > 1) {
        // select launch file to edit
        setEditNodeWithMultipleLaunchInfos({ node: node, external: external });
      } else {
        const [rootLaunch, launchInfo] = launchInfos[0];
        console.log(`Open for ${node.name}: ${JSON.stringify(launchInfo)}`);
        navCtx.openEditor(
          node.providerId,
          rootLaunch,
          launchInfo.file_name || "",
          launchInfo.file_range,
          launchInfo.launch_context_arg || [],
          launchInfo.topLevelArgs,
          external
        );
      }
    },
    [logCtx, navCtx]
  );

  /**
   * Create and open a new panel with a [ParameterPanel] for selected nodes and/or providers

   */
  const createParameterPanel = useCallback(
    (nodes: RosNode[], providers: string[]): void => {
      const openLocation: string = LAYOUT_TAB_SETS[nodeParamOpenLocation];
      const params: TMenuOptionsParam[] = [];

      for (const node of nodes) {
        params.push({
          name: node.name,
          callback: () => {
            const id = `parameter-node-${node.idGlobal}`;
            emitOpenComponent({
              id: id,
              title: `${node.name}`,
              closable: true,
              component: LAYOUT_TABS.PARAMETER,
              toNodeId: openLocation,
              config: { parameterConfig: { id, nodes: [node], providers: [] } },
            });
          },
        });
      }

      for (const providerId of providers) {
        const provider = rosCtx.getProviderById(providerId);
        if (provider) {
          params.push({
            name: provider.name(),
            callback: () => {
              const id = `parameter-provider-${provider}`;
              emitOpenComponent({
                id: id,
                title: `${provider.name()}`,
                closable: true,
                component: LAYOUT_TABS.PARAMETER,
                toNodeId: openLocation,
                config: { parameterConfig: { id, nodes: [], providers: [providerId] } },
              });
            },
          });
        }
      }

      if (params.length >= 3) {
        setNodeParams(params);
      } else {
        for (const item of params) {
          item.callback();
        }
      }
    },
    [rosCtx, nodeParamOpenLocation]
  );

  async function startNodeQueued(node: RosNode | undefined): Promise<void> {
    if (!node) return;

    const provider = rosCtx.getProviderById(node.providerId);
    logCtx.debug(`start: ${node.name}`);

    if (!provider || !provider.isAvailable()) {
      queue.addStatus("START", node.name, false, `Provider ${node.providerName} not available`);
      return;
    }

    try {
      const result = await provider.startNode(node);
      const success = result.success ?? false;
      const details = result.details || (success ? "started" : "failed");
      queue.addStatus("START", node.name, success, details);
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      queue.addStatus("START", node.name, false, `Error starting node: ${message}`);
    }
  }

  /**
   * Extend node list with associated nodes (e.g. composable containers).

   */
  function updateWithAssociations(nodes: RosNode[]): RosNode[] {
    const newNodeList: RosNode[] = [];

    for (const node of nodes) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider) continue;
      newNodeList.push(...provider.getAssociatedNodes(node));
      newNodeList.push(node);
    }

    return newNodeList;
  }

  /**
   * Get the nodelet manager / composable container node for a node if needed.

   */
  function getNodeLetManager(node: RosNode, ignoreRunState: boolean, nodes2start: RosNode[]): RosNode | null {
    if (!node) return null;
    const composableParent = node.getLaunchComposableContainer();
    if (composableParent) {
      const provider = rosCtx.getProviderById(node.providerId);
      const nodeNms = provider?.rosNodes.filter((n) => n.name === composableParent) || [];
      const nodeNm = nodeNms.length > 0 ? nodeNms[0] : null;
      if (
        nodeNm &&
        (nodeNm.status !== RosNodeStatus.RUNNING ||
          (ignoreRunState && nodes2start.filter((item) => item.id === nodeNm.id).length > 0)) &&
        !isQueued("START", nodeNm.name)
      ) {
        return nodeNm;
      }
    }
    return null;
  }

  /**
   * Prepare nodes for start, respecting launch files, associations and skip conditions.
   * If multiple launch files are involved, the user gets a selection dialog.

   */
  function startNodesWithLaunchCheck(
    nodes: RosNode[],
    ignoreRunState: boolean = false,
    useLaunchFiles: { [key: string]: string } = {},
    ignoreTimer: boolean = false
  ): void {
    const withMultiLaunch: RosNode[] = [];
    const node2Start: RosNode[] = [];
    const withNoLaunch: string[] = [];
    const skippedNodes: Map<string, string> = new Map();

    let nodeList = nodes.map((node) => {
      node.launchPath = useLaunchFiles[node.name];
      return node;
    });

    // extend list with associated nodes (e.g. containers)
    nodeList = updateWithAssociations(nodeList);

    const add2start = (node: RosNode | null): void => {
      if (node && node2Start.filter((item) => item.id === node.id).length === 0) {
        node.ignore_timer = ignoreTimer;
        node2Start.push(node);
      }
    };

    for (const node of nodeList) {
      if (!ignoreRunState && node.status === RosNodeStatus.RUNNING) {
        skippedNodes.set(node.name, "already running");
        continue;
      }

      if (isQueued("START", node.name)) {
        skippedNodes.set(node.name, "already in the start queue");
        continue;
      }

      if (node.launchInfo.size > 0) {
        const managerNode = getNodeLetManager(node, ignoreRunState, nodes);
        if (managerNode) {
          managerNode.launchPath = node.launchPath;
        }
        add2start(managerNode);
        add2start(node);
        if (node.launchInfo.size > 1 && !node.launchPath) {
          withMultiLaunch.push(node);
        }
      } else if (!node.system_node) {
        withNoLaunch.push(node.name);
      }
    }

    if (skippedNodes.size > 0) {
      logCtx.debug(`Skipped ${skippedNodes.size} nodes`, JSON.stringify(Object.fromEntries(skippedNodes)));
    }
    if (withNoLaunch.length > 0) {
      for (const nodeName of withNoLaunch) {
        skippedNodes.set(nodeName, "no launch file");
      }
      logCtx.debug(`No launch file for ${withNoLaunch.length} nodes found`, JSON.stringify(node2Start));
    }

    if (withMultiLaunch.length > 0) {
      // let the user select the launch files.
      setNodesAwaitModal(node2Start);
      setNodeMultiLaunches(withMultiLaunch);
    } else if (node2Start.length > 0) {
      setNodesToStart(node2Start);
    }
  }

  function removePendingNodes(nodes: RosNode[]): void {
    const idsToRemove = new Set(nodes.map((n) => n.id));
    removePendingKill(nodes);

    setPendingRestart((prev) => {
      if (!prev) return null;
      const remaining = prev.nodes.filter((n) => !idsToRemove.has(n.id));
      return remaining.length > 0 ? { ...prev, nodes: remaining } : null;
    });
  }

  /**
   * Start nodes in the selected list

   */
  function startSelectedNodes(ignoreTimer: boolean = false): void {
    const nodes = getSelectedNodes();
    removePendingNodes(nodes);
    startNodesWithLaunchCheck(nodes, false, {}, ignoreTimer);
  }

  /**
   * Start nodes from a list of itemIds

   */
  const startNodesFromId = useCallback(
    (itemIds: string[]): void => {
      const nodeList = getNodesFromIds(itemIds);
      removePendingNodes(nodeList);
      startNodesWithLaunchCheck(nodeList);
    },
    [getNodesFromIds]
  );

  /** Stop node from queue and trigger the next one. */
  async function stopNodeQueued(node: RosNode | undefined): Promise<void> {
    if (node === undefined) {
      queue.addStatus("STOP", "undefined", false, "invalid node");
      return;
    }

    logCtx.debug(`stop: ${node.name}`, "");
    const provider = rosCtx.getProviderById(node.providerId);

    if (!provider || !provider.isAvailable()) {
      queue.addStatus("STOP", node.name, false, `Provider ${node.providerName} not available`);
      return;
    }

    if (node.status === RosNodeStatus.RUNNING) {
      const resultStopNode = await provider.stopNode(node.id);
      if (!resultStopNode.result) {
        queue.addStatus("STOP", node.name, false, resultStopNode.message);
      } else {
        // node terminated regularly -> no SIGKILL needed
        removePendingKill([node]);
        queue.addStatus("STOP", node.name, true, "stopped");
      }
    } else if ((node.screens || []).length > 0) {
      const resultTermNode = await provider.screenKillNode(node.id, "SIGTERM");
      if (!resultTermNode.result) {
        queue.addStatus("STOP", node.name, false, resultTermNode.message);
      } else {
        queue.addStatus("STOP", node.name, true, "sent SIGTERM to executable");
      }
    } else {
      removePendingKill([node]);
      queue.addStatus("STOP", node.name, false, "no screen to stop");
    }
  }

  /**
   * Stops the given nodes, optionally only those with launch files, and optionally restarts them.

   */
  function stopNodes(nodes: RosNode[], onlyWithLaunch?: boolean): number {
    const skipped: Record<string, string> = {};
    const nodeList = updateWithAssociations(nodes);
    const nodesToStop: RosNode[] = [];

    for (const node of nodeList) {
      if (node.system_node && nodeList.length > 1) {
        skipped[node.name] = "system node";
        continue;
      }
      if (onlyWithLaunch && node.launchInfo.size === 0) {
        skipped[node.name] = "stop only with launch files";
        continue;
      }
      // synchronous check, state based check would miss enqueues from the same tick
      if (isQueued("STOP", node.name)) {
        skipped[node.name] = "already in queue";
        continue;
      }
      const isRunning = node.status === RosNodeStatus.RUNNING || (node.screens?.length ?? 0) > 0;
      if (!isRunning) {
        skipped[node.name] = "not running";
        continue;
      }
      nodesToStop.push(node);
    }

    if (Object.keys(skipped).length > 0) {
      logCtx.debug(`Skipped stopping ${Object.keys(skipped).length} nodes`, JSON.stringify(skipped));
    }

    let maxKillTime = 0;
    if (nodesToStop.length > 0) {
      enqueue(
        nodesToStop.map((node) => ({ node, action: "STOP" as QueueActionType })),
        "stopNodes"
      );

      const nodesKillTimeout: RosNode[] = [];
      for (const node of nodesToStop) {
        if (!node.pid) continue;
        for (const launchInfo of node.launchInfo.values()) {
          if (launchInfo.sigkill_timeout) {
            nodesKillTimeout.push(node);
            maxKillTime = Math.max(maxKillTime, launchInfo.sigkill_timeout);
          }
        }
      }

      if (maxKillTime > 0) {
        pendingKillNodesRef.current = [...pendingKillNodesRef.current, ...nodesKillTimeout];
        const timerId = window.setTimeout(() => {
          killTimersRef.current = killTimersRef.current.filter((id) => id !== timerId);
          const toKill = pendingKillNodesRef.current.filter((n) => nodesKillTimeout.some((nk) => nk.id === n.id));
          pendingKillNodesRef.current = pendingKillNodesRef.current.filter(
            (n) => !nodesKillTimeout.some((nk) => nk.id === n.id)
          );
          if (toKill.length > 0) {
            enqueue(
              toKill.map((node) => ({ node, action: "KILL" as QueueActionType })),
              "sigkill-timer"
            );
          }
        }, maxKillTime);
        killTimersRef.current.push(timerId);
      }
    }

    return maxKillTime;
  }

  /**
   * Stop nodes in the selected list

   */
  function stopSelectedNodes(): void {
    stopNodes(getSelectedNodes());
  }

  /**
   * Stop nodes from a list of itemIds

   */
  const stopNodesFromId = useCallback(
    (itemIds: string[]): void => {
      const nodeList = getNodesFromIds(itemIds);
      stopNodes(nodeList);
    },
    [getNodesFromIds]
  );
  /**
   * Restart nodes given in the arguments

   */
  function restartNodes(nodeList: RosNode[], onlyWithLaunch: boolean, ignoreTimer: boolean = false): void {
    // Stop nodes, but do not restart immediately.
    const maxKillTime = stopNodes(nodeList, onlyWithLaunch);

    setPendingRestart((prev) => {
      // Merge node lists (unique by id)
      const nodeMap = new Map<string, RosNode>();
      for (const n of prev?.nodes || []) {
        nodeMap.set(n.id, n);
      }
      for (const n of nodeList) {
        nodeMap.set(n.id, n);
      }
      const mergedNodes = Array.from(nodeMap.values());

      const now = Date.now();
      const thisNotBefore = now + (maxKillTime > 0 ? maxKillTime + 500 : 0); // wait at least maxKillTime (+ small buffer)
      const mergedNotBefore = Math.max(prev?.notBefore ?? 0, thisNotBefore);

      return {
        nodes: mergedNodes,
        // If any restart requires "onlyWithLaunch", keep it restrictive
        onlyWithLaunch: prev ? prev.onlyWithLaunch && onlyWithLaunch : onlyWithLaunch,
        // If any restart ignores the timer, ignore the timer for all
        ignoreTimer: prev ? prev.ignoreTimer || ignoreTimer : ignoreTimer,
        notBefore: mergedNotBefore,
      };
    });
  }

  /**
   * Restart nodes in the selected list

   */
  function restartSelectedNodes(ignoreTimer: boolean = false): void {
    restartNodes(getSelectedNodes(), true, ignoreTimer);
  }

  /**
   * Kill selected nodes using provider

   */
  function killSelectedNodes(): void {
    const nodes2kill: RosNode[] = [];
    const nodes = getSelectedNodes();
    for (const node of nodes) {
      if (node.system_node && nodes.length > 1) continue;
      if (isQueued("KILL", node.name)) {
        continue;
      }
      nodes2kill.push(node);
    }
    enqueue(
      nodes2kill.map((node) => {
        return { node, action: "KILL" };
      }),
      "killSelectedNodes"
    );
  }

  /** Kill node in the queue and trigger the next one. */
  async function killNodeQueued(node: RosNode | undefined): Promise<void> {
    if (!node) return;

    const provider = rosCtx.getProviderById(node.providerId);

    if (!provider || !provider.isAvailable()) {
      queue.addStatus("KILL", node.name, false, `Provider ${node.providerName} not available`);
      return;
    }

    try {
      const result = await provider.screenKillNode(node.name);
      const success = result.result ?? false;
      const message = result.message || (success ? "killed" : "failed");
      queue.addStatus("KILL", node.name, success, message);
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      queue.addStatus("KILL", node.name, false, `Error killing node: ${message}`);
    }
  }

  /**
   * Unregister selected nodes using provider

   */
  function unregisterSelectedNodes(): void {
    const nodes2unregister: RosNode[] = [];
    const nodes = getSelectedNodes();
    for (const node of nodes) {
      if (node.system_node && nodes.length > 1) continue;
      if (!node.masteruri) continue;
      if (isQueued("UNREGISTER", node.name)) {
        continue;
      }
      nodes2unregister.push(node);
    }
    enqueue(
      nodes2unregister.map((node) => {
        return { node, action: "UNREGISTER" };
      }),
      "unregisterSelectedNodes"
    );
  }

  /** Unregister node in the queue and trigger the next one. */
  async function unregisterNodeQueued(node: RosNode | undefined): Promise<void> {
    if (!node) return;

    const provider = rosCtx.getProviderById(node.providerId);

    if (!provider || !provider.isAvailable()) {
      queue.addStatus("UNREGISTER", node.name, false, `Provider ${node.providerName} not available`);
      return;
    }

    try {
      const result = await provider.unregisterNode(node.id);
      const success = result.result ?? false;
      const message = result.message || (success ? "unregistered" : "failed");
      queue.addStatus("UNREGISTER", node.name, success, message);
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      queue.addStatus("UNREGISTER", node.name, false, `Error unregistering node: ${message}`);
    }
  }

  /**
   * Start dynamic reconfigure GUI for selected nodes

   */
  function startDynamicReconfigure(service: string, masteruri: string): void {
    if (
      queue.has(
        (elem) => elem.action === "DYNAMIC_RECONFIGURE" && elem.service === service && elem.masteruri === masteruri
      )
    ) {
      return;
    }
    enqueue([{ action: "DYNAMIC_RECONFIGURE", service: service, masteruri: masteruri }], "dynamicReconfigure");
  }

  /** start dynamic reconfigure in the queue and trigger the next one. */
  async function dynamicReconfigureQueued(service: string | undefined, masteruri: string | undefined): Promise<void> {
    if (service) {
      const result = await rosCtx.startDynamicReconfigureClient(service, masteruri || "");
      if (!result.result) {
        queue.addStatus("DYNAMIC_RECONFIGURE", service, false, result.message);
      } else {
        queue.addStatus("DYNAMIC_RECONFIGURE", service, true, "dynamic reconfigure started");
      }
    }
  }

  /**
   * Remove log files from ROS nodes using [rosclean purge] for a given provider

   */
  async function clearProviderLogs(providers: string[]): Promise<void> {
    if (!providers || providers.length === 0) return;

    try {
      const promises: Promise<void>[] = [];
      for (const providerId of providers) {
        const provider = rosCtx.getProviderById(providerId);
        if (!provider || !provider.isAvailable()) continue;

        const promise = provider
          .rosCleanPurge()
          .then((result: Result) => {
            if (!result.result) {
              logCtx.error("Could not delete logs", result.message, "logs not deleted");
            } else if (!result.message.includes("Purging ROS node logs")) {
              logCtx.error("Unexpected result during log purge", result.message, "logs not deleted");
            } else {
              logCtx.success("Logs removed successfully", result.message, "logs removed");
            }
          })
          .catch((err: unknown) => {
            const message = err instanceof Error ? err.message : String(err);
            logCtx.error(`Error purging logs for provider ${providerId}`, message, "logs not deleted");
          });

        promises.push(promise);
      }

      await Promise.all(promises);
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      logCtx.error("Could not clear logs for providers", message, "could not clear logs");
    }
  }

  /**
   * Delete log file of given nodes.

   */
  function clearLogs(nodes: RosNode[]): void {
    const nodes2clear: RosNode[] = [];
    for (const node of nodes) {
      if (node.system_node && nodes.length > 1) continue;
      if (isQueued("CLEAR_LOG", node.name)) {
        continue;
      }
      nodes2clear.push(node);
    }
    enqueue(
      nodes2clear.map((node) => {
        return { node, action: "CLEAR_LOG" };
      }),
      "clearLogs"
    );
  }

  /** Delete log file for node in the queue and trigger the next one. */
  async function clearNodeLogQueued(node: RosNode | undefined): Promise<void> {
    if (!node) return;

    const provider = rosCtx.getProviderById(node.providerId);

    if (!provider || !provider.isAvailable()) {
      queue.addStatus("CLEAR_LOG", node.name, false, `Provider ${node.providerName} not available`);
      return;
    }

    try {
      const results: TResultClearPath[] = await provider.clearLogPaths([node.name]);

      if (results.length === 0) {
        const msg = "No log deletion result returned";
        logCtx.error(
          `Could not delete log files for node: [${node.name}]`,
          JSON.stringify(results),
          "logs not deleted"
        );
        queue.addStatus("CLEAR_LOG", node.name, false, msg);
        return;
      }

      const firstResult = results[0];
      if (!firstResult.result) {
        logCtx.error(`Could not delete log files for node: [${node.name}]`, firstResult.message, "logs not deleted");
        queue.addStatus("CLEAR_LOG", node.name, false, firstResult.message);
      } else {
        queue.addStatus("CLEAR_LOG", node.name, true, `Removed log files for node: [${node.name}]`);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      queue.addStatus("CLEAR_LOG", node.name, false, `Error clearing logs: ${message}`);
    }
  }

  function refreshAllProvider(forceRefresh: boolean): void {
    for (const p of rosCtx.providers) {
      if (p.connectionState === ConnectionState.STATES.CONNECTED) {
        p.updateRosNodes({}, forceRefresh);
        p.updateTimeDiff();
        p.updateDiagnostics(null);
      }
    }
  }

  // Register useEffect Callbacks ----------------------------------------------------------------------------------

  /**
   * Synchronizes providerNodes with the current provider list.
   */
  useEffect(() => {
    const newProviderNodes: TProviderNodes[] = [];
    const selectedDomainId = contentId?.domainId;
    const selectedProviderId = contentId?.providerId;
    for (const [providerId, nodes] of rosCtx.mapProviderRosNodes) {
      if (selectedProviderId !== undefined) {
        if (providerId !== selectedProviderId) {
          continue;
        }
      } else if (selectedDomainId !== undefined) {
        // Otherwise filter by domainId if set
        const provider = rosCtx.getProviderById(providerId);
        const providerDomainId = provider?.connection?.domainId;
        if (providerDomainId !== selectedDomainId) {
          continue;
        }
      }
      newProviderNodes.push({ providerId, nodes });
    }
    setProviderNodes(newProviderNodes);
  }, [rosCtx.mapProviderRosNodes]);

  useEffect(() => {
    if (!nodesToStart) return;
    // an effect re-run (unstable deps, StrictMode) must not enqueue the same batch twice
    if (handledStartBatchRef.current === nodesToStart) return;
    handledStartBatchRef.current = nodesToStart;
    console.log(`Enqueueing ${nodesToStart.length} nodes to start: ${nodesToStart.map((n) => n.name).join(", ")}`);
    enqueue(
      nodesToStart.map((node) => ({ node, action: "START" as QueueActionType })),
      "nodesToStart-effect"
    );
    setNodesToStart(undefined);
  }, [nodesToStart]);

  useEffect(() => {
    if (!pendingRestart) return undefined;

    // already handled this exact restart request
    if (handledRestartRef.current === pendingRestart) return undefined;

    // queue still processing (STOP/KILL/UNREGISTER/... in progress)
    if (queue.currentIndex >= 0) return undefined;

    // wait until all kill process questions have been handled by the user
    if (killProcessQuestion.length > 0) return undefined;

    // ensure that at least maxKillTime has passed since the last stop
    const remaining = pendingRestart.notBefore - Date.now();
    if (remaining > 0) {
      const timer = setTimeout(() => {
        // force re-evaluation of this effect
        setPendingRestart((prev) => (prev ? { ...prev } : null));
      }, remaining + 50);
      return () => clearTimeout(timer);
    }

    handledRestartRef.current = pendingRestart;

    // nodes are restarted now, so cancel a still pending SIGKILL for them
    removePendingKill(pendingRestart.nodes);

    startNodesWithLaunchCheck(
      pendingRestart.nodes,
      true, // ignoreRunState: we really want to restart
      {},
      pendingRestart.ignoreTimer
    );

    setPendingRestart(null);
    return undefined;
  }, [pendingRestart, queue.currentIndex, killProcessQuestion]);

  // cleanup on unmount
  useEffect(() => {
    return () => {
      cancelKillTimers();
    };
  }, [cancelKillTimers]);

  /**
   * Queue action handlers for all supported queue actions.
   * Memoized to avoid unnecessary re-creations.
   */
  const queueActionHandlers = useMemo<Record<QueueActionType, (item: TQueueAction) => Promise<void>>>(
    () => ({
      START: (item: TQueueAction) => startNodeQueued(item.node),
      STOP: (item: TQueueAction) => stopNodeQueued(item.node),
      UNREGISTER: (item: TQueueAction) => unregisterNodeQueued(item.node),
      KILL: (item: TQueueAction) => killNodeQueued(item.node),
      CLEAR_LOG: (item: TQueueAction) => clearNodeLogQueued(item.node),
      DYNAMIC_RECONFIGURE: (item: TQueueAction) => dynamicReconfigureQueued(item.service, item.masteruri),
    }),
    []
  );

  /**
   * Execute the next item in the main queue and handle completion summary.
   */
  const performQueueMain = useCallback(
    async (index: number): Promise<void> => {
      if (index < 0) return;

      // read from the synchronous mirror, state in the closure may be one render behind
      const queueItem = queue.getAt(index);

      if (queueItem) {
        const handler = queueActionHandlers[queueItem.action];
        if (handler) {
          console.log(`Performing queue action: ${queueItem.action} for node: ${queueItem.node?.name}`);
          await handler(queueItem);
          console.log(`Completed queue action: ${queueItem.action} for node: ${queueItem.node?.name}`);
        } else {
          console.warn(`Unknown queue action: ${JSON.stringify(queueItem)}`);
        }
        return;
      }

      // ---- Queue finished ----
      for (const action of Object.keys(queueActionMeta) as QueueActionType[]) {
        const failed = queue.failed(action);
        if (failed.length > 0) {
          const infoDict = Object.fromEntries(failed.map((item) => [item.itemName, item.message]));
          logCtx.warn(
            `Failed to ${action.toLowerCase()} ${failed.length} nodes`,
            JSON.stringify(infoDict),
            `Failed to ${action.toLowerCase()} ${failed.length} nodes`
          );
        }

        const success = queue.success(action);
        if (success.length > 0) {
          const infoDict = Object.fromEntries(success.map((item) => [item.itemName, item.message]));
          logCtx.success(
            `${success.length} nodes ${queueActionMeta[action].successText}`,
            JSON.stringify(infoDict),
            `${success.length} nodes ${queueActionMeta[action].successText}`
          );
        }
      }

      queue.clear();
    },
    [logCtx, queue, queueActionHandlers]
  );

  // update queue
  useEffect(() => {
    // Prevent processing the same index multiple times.
    // This can happen in development, e.g. with React StrictMode.
    if (queue.currentIndex === lastProcessedIndexRef.current) {
      return;
    }

    lastProcessedIndexRef.current = queue.currentIndex;

    performQueueMain(queue.currentIndex);
  }, [queue.currentIndex, performQueueMain]);

  // --- Derived selection information for the action bar ---
  const selectedNodes = getSelectedNodes();
  const hasDynamicReconfigure = selectedNodes.some((node) => (node.dynamicReconfigureServices?.length ?? 0) > 0);
  const hasSelectedProviders = selectedProviderItems.length > 0;
  const canUnregisterSelectedNodes = selectedNodes.some((node) => (node.masteruri ?? "").length > 0);

  // --- Action handlers passed to HostTreeViewActions ---

  // Start: behavior identical to previous inline handler
  const handleStartClick = (options: { ignoreTimer: boolean }): void => {
    startSelectedNodes(options.ignoreTimer);
  };

  // Stop / Kill / Unregister: identical branching logic as before
  const handleStopClick = (options: { kill: boolean; unregister: boolean }): void => {
    if (options.kill) {
      if (options.unregister) {
        unregisterSelectedNodes();
      } else {
        killSelectedNodes();
      }
    } else {
      stopSelectedNodes();
    }
  };

  const handleRestartClick = (options: { ignoreTimer: boolean }): void => {
    restartSelectedNodes(options.ignoreTimer);
  };

  const handleKillClick = (): void => {
    killSelectedNodes();
  };

  const handleUnregisterClick = (): void => {
    unregisterSelectedNodes();
  };

  const handleEditClick = (options: { external: boolean }): void => {
    // Use current selection to open editor
    createFileEditorPanel(getSelectedNodes(), options.external);
  };

  const handleParametersClick = (): void => {
    if (hasSelectedProviders) {
      createParameterPanel([], selectedProviderItems);
    } else {
      createParameterPanel(getSelectedNodes(), []);
    }
  };

  const handleDynamicReconfigureClick = (): void => {
    let countDri = 0;
    const driItems: RosNode[] = [];
    const currentSelectedNodes = getSelectedNodes();

    for (const node of currentSelectedNodes) {
      countDri += node.dynamicReconfigureServices.length;
      driItems.push(node);
    }

    if (countDri > 2) {
      setDynamicReconfigureItems(driItems);
    } else {
      for (const node of driItems) {
        for (const dri of node.dynamicReconfigureServices) {
          startDynamicReconfigure(dri, node.masteruri ?? "");
        }
      }
    }
  };

  const handleScreensClick = (options: { external: boolean; openInTerminal: boolean }): void => {
    if (hasSelectedProviders) {
      const screens: TMenuOptionsScreen[] = [];

      for (const providerId of selectedProviderItems) {
        const provider = rosCtx.getProviderById(providerId);
        if (!provider) {
          continue;
        }

        for (const screenMap of provider.screens ?? []) {
          for (const screen of screenMap.screens ?? []) {
            screens.push({
              nodeName: screenMap.name,
              providerId: provider.id,
              screen,
              external: options.external,
            });
          }
        }
      }

      setNodeScreens(screens);

      if (screens.length === 0) {
        logCtx.info("no screens found", "", "no screens found");
      }
    } else {
      const screens: TMenuOptionsScreen[] = [];
      const currentSelectedNodes = getSelectedNodes();

      for (const node of currentSelectedNodes) {
        if ((node.screens?.length ?? 0) === 1 && node.screens) {
          screens.push({
            nodeName: node.name,
            providerId: node.providerId,
            screen: node.screens[0],
            callback: () => {
              if (node.screens) {
                createSingleTerminalPanel(
                  CmdTypes.SCREEN,
                  node.providerId,
                  node.name,
                  node.screens[0],
                  options.external,
                  options.openInTerminal
                );
              }
            },
          });
        } else if ((node.screens?.length ?? 0) > 1 && node.screens) {
          for (const screen of node.screens) {
            screens.push({
              nodeName: node.name,
              providerId: node.providerId,
              screen,
              callback: () => {
                createSingleTerminalPanel(
                  CmdTypes.SCREEN,
                  node.providerId,
                  node.name,
                  screen,
                  options.external,
                  options.openInTerminal
                );
              },
            });
          }
        } else {
          screens.push({
            nodeName: node.name,
            providerId: node.providerId,
            screen: "autodetect",
            callback: () => {
              createSingleTerminalPanel(
                CmdTypes.SCREEN,
                node.providerId,
                node.name,
                "",
                options.external,
                options.openInTerminal
              );
            },
          });
        }
      }

      if (screens.length >= 3) {
        setNodeScreens(screens);
      } else {
        for (const item of screens) {
          if (item.callback) {
            item.callback();
          } else {
            createSingleTerminalPanel(
              CmdTypes.SCREEN,
              item.providerId,
              item.nodeName,
              item.screen,
              item.external ?? options.external
            );
          }
        }
      }
    }
  };

  const handleLogsClick = (options: { external: boolean; openInTerminal: boolean }): void => {
    const logs: TMenuOptionsNode[] = [];
    const currentSelectedNodes = getSelectedNodes();

    for (const node of currentSelectedNodes) {
      logs.push({
        node,
        callback: () => {
          createSingleTerminalPanel(
            CmdTypes.LOG,
            node.providerId,
            node.name,
            "",
            options.external,
            options.openInTerminal
          );
        },
      });
    }

    if (logs.length >= 3) {
      setNodeLogs(logs);
    } else {
      for (const item of logs) {
        item.callback();
      }
    }
  };

  const handleLoggersClick = (): void => {
    const loggers: TMenuOptionsNode[] = [];
    const currentSelectedNodes = getSelectedNodes();

    for (const node of currentSelectedNodes) {
      loggers.push({
        node,
        callback: () => {
          createLoggerPanel(node);
        },
      });
    }

    if (loggers.length >= 3) {
      setNodeLoggers(loggers);
    } else {
      for (const item of loggers) {
        item.callback();
      }
    }
  };

  const handleClearLogsClick = (): void => {
    if (hasSelectedProviders) {
      setRosCleanPurge(true);
    } else {
      clearLogs(getSelectedNodes());
    }
  };

  const handleOpenTerminalOnHostsClick = (options: { external: boolean; openInTerminal: boolean }): void => {
    for (const providerId of selectedProviderItems) {
      createSingleTerminalPanel(CmdTypes.TERMINAL, providerId, "", "", options.external, options.openInTerminal, true);
    }
  };

  const handleShutdownRosClick = (options: { killRos2: boolean }): void => {
    if (selectedProviderItems.length === 1) {
      setShutdownRos(options.killRos2 ? "kill ros2" : "only nodes");
    }
  };

  const shutdownProvider = useCallback(
    async (id: string, killRos2: boolean) => {
      const provider = rosCtx.getProviderById(id);
      if (!provider) return;
      console.log(`shutdown ${provider.id}`);
      const result = await provider.shutdown(killRos2, ignoreProcessesOnShutdown.split(","));
      console.log(`finished shutdown ${provider.id} ${JSON.stringify(result)}`);
    },
    [rosCtx.getProviderById]
  );

  const createActions = () => {
    return (
      <Box height="100%">
        <HostTreeViewActions
          selectedProviderCount={selectedProviderItems.length}
          selectedNodesCount={selectedNodes.length}
          hasDynamicReconfigure={hasDynamicReconfigure}
          canUnregisterSelectedNodes={canUnregisterSelectedNodes}
          showButtonsForKeyModifiers={showButtonsForKeyModifiers}
          onStartClick={handleStartClick}
          onStopClick={handleStopClick}
          onRestartClick={handleRestartClick}
          onKillClick={handleKillClick}
          onUnregisterClick={handleUnregisterClick}
          onEditClick={handleEditClick}
          onParametersClick={handleParametersClick}
          onDynamicReconfigureClick={handleDynamicReconfigureClick}
          onScreensClick={handleScreensClick}
          onLogsClick={handleLogsClick}
          onLoggersClick={handleLoggersClick}
          onClearLogsClick={handleClearLogsClick}
          onOpenTerminalOnHostsClick={handleOpenTerminalOnHostsClick}
          onShutdownRosClick={handleShutdownRosClick}
        />
      </Box>
    );
  };

  return (
    <Box width="100%" height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
      <Stack spacing={0.5} direction="column" width="100%" height="100%">
        {queue.currentIndex < 0 && (
          <Stack direction="row" spacing={0.5} alignItems="center">
            {buttonLocation === BUTTON_LOCATIONS.LEFT && (
              <Tooltip title="Reload node list" placement="left" disableInteractive>
                <IconButton
                  size="small"
                  onClick={() => {
                    refreshAllProvider(true);
                  }}
                >
                  <RefreshIcon sx={{ fontSize: "inherit" }} />
                </IconButton>
              </Tooltip>
            )}
            <SearchBar
              key={"search-bar-host"}
              onSearch={(value) => {
                setFilterText(value);
              }}
              placeholder="Search nodes (OR: <space>, AND: +, NOT: !)"
              defaultValue={filterText}
              fullWidth
            />
            {buttonLocation === BUTTON_LOCATIONS.RIGHT && (
              <Tooltip title="Reload node list" placement="left" disableInteractive>
                <IconButton
                  size="small"
                  onClick={() => {
                    refreshAllProvider(true);
                  }}
                >
                  <RefreshIcon sx={{ fontSize: "inherit" }} />
                </IconButton>
              </Tooltip>
            )}
          </Stack>
        )}
        {queue.currentIndex >= 0 && (
          <Paper elevation={2}>
            <Stack alignItems="center" justifyItems="center" direction="row" spacing={0.5} sx={{ marginRight: 2 }}>
              <LinearProgress sx={{ width: "100%" }} variant="determinate" value={progressQueueMain} />
              <FormLabel>
                {queue.currentIndex}/{queue.queue.length}
              </FormLabel>
              <IconButton
                onClick={() => {
                  // cancel pending SIGKILL timers, otherwise nodes are killed after the user aborted
                  cancelKillTimers();
                  setPendingRestart(null);
                  queue.clear();
                }}
                size="small"
              >
                <StopIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Stack>
          </Paper>
        )}
        {countFilteredNodes > 0 && (
          <Alert severity="info" style={{ minWidth: 0 }}>
            {countFilteredNodes} nodes hidden by the filter
          </Alert>
        )}
        <Stack direction="row" height="100%" overflow="auto">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && <Box height="100%">{createActions()}</Box>}
          <HostTreeView
            triggerId={`host-tree-${contentToId(contentId)}`}
            visibleNodes={visibleNodesGlobal}
            isFiltered={filterText.length > 0}
            showLoggers={createLoggerPanelFromId}
            startNodes={startNodesFromId}
            stopNodes={stopNodesFromId}
            setSelectedProviderItems={setSelectedProviderItems}
            setSelectedNodeItems={setSelectedNodeItems}
          />

          {buttonLocation === BUTTON_LOCATIONS.RIGHT && <Box height="100%">{createActions()}</Box>}
        </Stack>
      </Stack>

      {rosCleanPurge && (
        <ConfirmModal
          title="Question"
          message="Confirm to remove all ros log files."
          onConfirmCallback={() => {
            setRosCleanPurge(false);
            clearProviderLogs(selectedProviderItems);
          }}
          onCancelCallback={() => {
            setRosCleanPurge(false);
          }}
        />
      )}
      {nodeParams.length > 0 && (
        <ListSelectionModal
          title="Select nodes to open parameter"
          list={nodeParams.reduce((prev: string[], item) => {
            prev.push(`${item.name}`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const nodeWithOpt = nodeParams.find((nodeItem) => `${nodeItem.name}` === item);
              if (nodeWithOpt?.callback) {
                nodeWithOpt.callback();
              }
            }
            setNodeParams([]);
          }}
          onCancelCallback={() => {
            setNodeParams([]);
          }}
        />
      )}
      {nodeScreens.length > 0 && (
        <ListSelectionModal
          title="Select screens to open"
          list={nodeScreens.reduce((prev: string[], item) => {
            prev.push(`${item.nodeName} [${item.screen}]`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const nodeWithOpt = nodeScreens.find(
                (nodeScreen) => `${nodeScreen.nodeName} [${nodeScreen.screen}]` === item
              );
              if (nodeWithOpt) {
                if (nodeWithOpt.callback) {
                  nodeWithOpt.callback();
                } else {
                  createSingleTerminalPanel(
                    CmdTypes.SCREEN,
                    nodeWithOpt.providerId,
                    nodeWithOpt.nodeName,
                    nodeWithOpt.screen,
                    nodeWithOpt.external
                  );
                }
              }
            }
            setNodeScreens([]);
          }}
          onCancelCallback={() => {
            setNodeScreens([]);
          }}
        />
      )}
      {nodeLogs.length > 0 && (
        <ListSelectionModal
          title="Select logs to open"
          list={nodeLogs.reduce((prev: string[], item) => {
            prev.push(`${item.node.name}`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const nodeWithOpt = nodeLogs.find((logItem) => `${logItem.node.name}` === item);
              if (nodeWithOpt?.callback) {
                nodeWithOpt.callback();
              }
            }
            setNodeLogs([]);
          }}
          onCancelCallback={() => {
            setNodeLogs([]);
          }}
        />
      )}
      {nodeLoggers.length > 0 && (
        <ListSelectionModal
          title="Select nodes to open logger levels"
          list={nodeLoggers.reduce((prev: string[], item) => {
            prev.push(`${item.node.name}`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const nodeWithOpt = nodeLoggers.find((nodeItem) => `${nodeItem.node.name}` === item);
              if (nodeWithOpt?.callback) {
                nodeWithOpt.callback();
              }
            }
            setNodeLoggers([]);
          }}
          onCancelCallback={() => {
            setNodeLoggers([]);
          }}
        />
      )}
      {nodeMultiLaunches.length > 0 && (
        <MapSelectionModal
          title={"Select launch file"}
          list={nodeMultiLaunches.reduce((prev: MapSelectionItem[], node) => {
            prev.push({ title: node.name, list: Array.from(node.launchInfo.keys()) } as MapSelectionItem);
            return prev;
          }, [])}
          useRadioGroup
          onConfirmCallback={(items) => {
            const useLaunchFiles: { [key: string]: string } = {};
            for (const item of items) {
              for (const launch of item.list) {
                const node = nodeMultiLaunches.find((n) => n.name.includes(item.title));
                if (node) {
                  useLaunchFiles[node.name] = launch;
                }
              }
            }
            startNodesWithLaunchCheck(nodesAwaitModal, false, useLaunchFiles);
            setNodeMultiLaunches([]);
            setNodesAwaitModal([]);
          }}
          onCancelCallback={() => {
            setNodeMultiLaunches([]);
            setNodesAwaitModal([]);
          }}
        />
      )}
      {editNodeWithMultipleLaunchInfos && (
        <MapSelectionModal
          title={"Select launch file"}
          list={[
            {
              title: editNodeWithMultipleLaunchInfos.node.name,
              list: Array.from(editNodeWithMultipleLaunchInfos.node.launchInfo.keys()),
            } as MapSelectionItem,
          ]}
          useRadioGroup
          onConfirmCallback={(items) => {
            for (const item of items) {
              for (const launch of item.list) {
                const launchInfo = editNodeWithMultipleLaunchInfos.node.launchInfo.get(launch);
                navCtx.openEditor(
                  editNodeWithMultipleLaunchInfos.node.providerId,
                  launch,
                  launchInfo?.file_name || "",
                  launchInfo?.file_range as TFileRange,
                  launchInfo?.launch_context_arg || [],
                  launchInfo?.topLevelArgs || [],
                  editNodeWithMultipleLaunchInfos.external
                );
              }
            }
            setEditNodeWithMultipleLaunchInfos(undefined);
          }}
          onCancelCallback={() => {
            setEditNodeWithMultipleLaunchInfos(undefined);
          }}
        />
      )}
      {dynamicReconfigureItems.length > 0 && (
        <MapSelectionModal
          title={"Select dynamic configuration to start"}
          list={dynamicReconfigureItems.reduce((prev: MapSelectionItem[], node) => {
            prev.push({
              title: node.name,
              list: node.dynamicReconfigureServices,
            });
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const node = dynamicReconfigureItems.find((n) => n.name === item.title);
              for (const dri of item.list) {
                startDynamicReconfigure(dri, node?.masteruri || "");
              }
            }
            setDynamicReconfigureItems([]);
          }}
          onCancelCallback={() => {
            setDynamicReconfigureItems([]);
          }}
        />
      )}
      {killProcessQuestion.length > 0 && (
        <ListSelectionModal
          title={`Node process not found. Selected processes to kill for node ${killProcessQuestion[0].node}`}
          selectOnOpen={0}
          list={killProcessQuestion.reduce((prev: string[], item) => {
            prev.push(`${item.pid}: ${item.cmdLine}`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            const promises: Promise<void>[] = [];
            for (const item of items) {
              const listWithOpt = killProcessQuestion.find(
                (listItem) => `${listItem.pid}: ${listItem.cmdLine}` === item
              );
              if (listWithOpt?.callback) {
                promises.push(listWithOpt.callback());
              }
            }
            Promise.all(promises);
            setKillProcessQuestion([]);
          }}
          onCancelCallback={() => {
            setKillProcessQuestion([]);
          }}
        />
      )}
      {shutdownRos && (
        <Dialog
          open={!!shutdownRos}
          onClose={() => {
            setShutdownRos("");
          }}
          onAbort={() => {
            setShutdownRos("");
          }}
          fullWidth
          scroll="paper"
          maxWidth="sm"
          PaperComponent={DraggablePaper}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
            {shutdownRos === "kill ros2" ? "Terminate ROS and terminate all ros2 processes" : "Terminate ROS"}
          </DialogTitle>

          <DialogContent aria-label="list">
            <DialogContentText id="alert-dialog-description">
              {`Terminate ROS on "${selectedProviderItems.map((p) => p.split(":")[0])}"`}
            </DialogContentText>
          </DialogContent>

          <DialogActions>
            <Button
              color="primary"
              onClick={() => {
                setShutdownRos("");
              }}
            >
              Cancel
            </Button>
            <Button
              autoFocus
              color="primary"
              onClick={() => {
                for (const providerId of selectedProviderItems) {
                  shutdownProvider(providerId, shutdownRos === "kill ros2");
                }
                setShutdownRos("");
              }}
            >
              Ok
            </Button>
          </DialogActions>
        </Dialog>
      )}
    </Box>
  );
}
