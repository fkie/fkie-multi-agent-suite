import AddToQueueIcon from "@mui/icons-material/AddToQueue";
import BorderColorIcon from "@mui/icons-material/BorderColor";
import CancelPresentationIcon from "@mui/icons-material/CancelPresentation";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import DeleteSweepIcon from "@mui/icons-material/DeleteSweep";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RefreshIcon from "@mui/icons-material/Refresh";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import SettingsSuggestIcon from "@mui/icons-material/SettingsSuggest";
import StopIcon from "@mui/icons-material/Stop";
import TerminalIcon from "@mui/icons-material/Terminal";
import TuneIcon from "@mui/icons-material/Tune";
import WysiwygIcon from "@mui/icons-material/Wysiwyg";
import {
  Alert,
  Box,
  ButtonGroup,
  Divider,
  FormLabel,
  IconButton,
  LinearProgress,
  Paper,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { useCallback, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import HostTreeView from "@/renderer/components/HostTreeView/HostTreeView";
import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import ListSelectionModal from "@/renderer/components/SelectionModal/ListSelectionModal";
import MapSelectionModal, { MapSelectionItem } from "@/renderer/components/SelectionModal/MapSelectionModal";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import useQueue from "@/renderer/hooks/useQueue";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { Result, RosNode, RosNodeStatus } from "@/renderer/models";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import {
  EVENT_FILTER_NODES,
  EVENT_KILL_NODES,
  EVENT_OPEN_COMPONENT,
  EVENT_SHOW_SCREENS,
  eventOpenComponent,
  TEventId,
  TEventKillNodes,
  TEventShowScreens,
} from "@/renderer/pages/NodeManager/layout/events";
import { CmdType } from "@/renderer/providers";
import { ConnectionState, EventProviderRestartNodes, EventProviderRosNodes } from "@/renderer/providers/events";
import { EVENT_PROVIDER_RESTART_NODES, EVENT_PROVIDER_ROS_NODES } from "@/renderer/providers/eventTypes";
import { TResultClearPath } from "@/renderer/providers/ProviderConnection";
import { findIn } from "@/renderer/utils/index";
import { TFileRange } from "@/types";
import NodeLoggerPanel from "./NodeLoggerPanel";
import ParameterPanel from "./ParameterPanel";

type TProviderNodes = {
  providerId: string;
  nodes: RosNode[];
};

interface TQueueAction {
  action: string;
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

const queueActionMeta = {
  STOP: { successText: "stopped" },
  START: { successText: "started" },
  KILL: { successText: "killed" },
  UNREGISTER: { successText: "unregistered" },
  CLEAR_LOG: { successText: "logs cleared" },
  DYNAMIC_RECONFIGURE: { successText: "dynamic reconfigure started" },
};

export default function HostTreeViewPanel(): JSX.Element {
  // context objects
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();

  // state variables
  const [showButtonsForKeyModifiers, setShowButtonsForKeyModifiers] = useState<boolean>(
    settingsCtx.get("showButtonsForKeyModifiers") as boolean
  );
  const [tooltipDelay, setTooltipDelay] = useState(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [buttonLocation, setButtonLocation] = useState<string>(settingsCtx.get("buttonLocation") as string);

  const [filterText, setFilterText] = useState("");
  const [providerNodes, setProviderNodes] = useState<TProviderNodes[]>([]);
  const [visibleNodes, setVisibleNodes] = useState<RosNode[]>([]);
  const [countFilteredNodes, setCountFilteredNodes] = useState<number>(0);
  const [nodesToStart, setNodesToStart] = useState<RosNode[]>();
  const [progressQueueMain, setProgressQueueMain] = useState<number>(0);
  const queue = useQueue<TQueueAction>(setProgressQueueMain);

  // variables with show dialog actions
  const [rosCleanPurge, setRosCleanPurge] = useState(false);
  const [nodeScreens, setNodeScreens] = useState<TMenuOptionsScreen[]>([]);
  const [nodeParams, setNodeParams] = useState<TMenuOptionsParam[]>([]);
  const [nodeLogs, setNodeLogs] = useState<TMenuOptionsNode[]>([]);
  const [nodeLoggers, setNodeLoggers] = useState<TMenuOptionsNode[]>([]);
  const [nodeMultiLaunches, setNodeMultiLaunches] = useState<RosNode[]>([]);
  const [dynamicReconfigureItems, setDynamicReconfigureItems] = useState<RosNode[]>([]);
  const [nodesAwaitModal, setNodesAwaitModal] = useState<RosNode[]>([]);
  const [editNodeWithMultipleLaunchInfos, setEditNodeWithMultipleLaunchInfos] = useState<TMenuOptionsEditor>();
  const [killProcessQuestion, setKillProcessQuestion] = useState<
    { node: string; pid: number; cmdLine: string; callback?: () => void }[]
  >([]);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setShowButtonsForKeyModifiers(settingsCtx.get("showButtonsForKeyModifiers") as boolean);
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setButtonLocation(settingsCtx.get("buttonLocation") as string);
  }, [settingsCtx.changed]);

  /**
   * Get list of nodes from a list of node.idGlobal
   */
  const getNodesFromIds = useCallback(
    (itemIds: string[]) => {
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

  const getSelectedNodes = useCallback(() => {
    return getNodesFromIds(navCtx.selectedNodes);
  }, [navCtx.selectedNodes, providerNodes]);

  // debounced search callback
  // search in the origin node list and create a new tree
  const onSearch = useCallback(
    (searchTerm: string) => {
      let nodeCount = 0;
      let nodeFilteredCount = 0;
      const newVisibleNodes: RosNode[] = [];
      for (const item of providerNodes) {
        const { nodes } = item;
        nodeCount += nodes.length;
        const filteredNodes = nodes.filter((node) => {
          // filter nodes by user text
          if (searchTerm.length > 0) {
            const isMatch = findIn(searchTerm, [node.name, node.group, node.providerName, node.guid || ""]);
            return isMatch;
          }
          return true;
        });
        nodeFilteredCount += filteredNodes.length;
        // remove nodes which are local in remote hosts
        newVisibleNodes.push(
          ...filteredNodes.filter(
            (node) =>
              node.isLocal || node.launchInfo ||
              rosCtx.localNodes.filter((lNode: TLocalNode) => lNode.node === node.name && lNode.providerId !== node.providerId)
                .length === 0
          )
        );
      }
      setCountFilteredNodes(nodeCount - nodeFilteredCount);
      setVisibleNodes(newVisibleNodes);
    },
    [providerNodes]
  );

  useCustomEventListener(EVENT_PROVIDER_ROS_NODES, (data: EventProviderRosNodes) => {
    // put in our providerNodes list
    const { provider, nodes } = data;
    // TODO: should we remove closed/lost provider infos
    // Update local nodes in RosContext
    rosCtx.updateLocalNodes(
      data.provider.id,
      data.nodes.filter((node) => node.isLocal || node.launchInfo.size > 0).map((node) => node.name)
    );
    setProviderNodes((oldValues) => [
      ...oldValues.filter((item) => {
        return item.providerId !== provider.id;
      }),
      {
        providerId: provider.id,
        nodes: nodes,
      },
    ]);
  });

  useCustomEventListener(EVENT_PROVIDER_RESTART_NODES, (data: EventProviderRestartNodes) => {
    restartNodes(data.nodes, true);
  });

  useCustomEventListener(EVENT_FILTER_NODES, (data: TEventId) => {
    setFilterText(data.id);
  });

  useCustomEventListener(EVENT_KILL_NODES, (data: TEventKillNodes) => {
    queue.update(
      data.nodes.map((node) => {
        return { node, action: "KILL" };
      })
    );
  });

  useCustomEventListener(EVENT_SHOW_SCREENS, (data: TEventShowScreens) => {
    for (const node of data.nodes) {
      for (const screen of node.screens || []) {
        createSingleTerminalPanel(CmdType.SCREEN, node.providerId, node.name, screen, false, false);
      }
    }
  });
  // Register Callbacks ----------------------------------------------------------------------------------

  /**
   * Callback when nodes on the tree are selected by the user
   */
  const handleNodesSelect = useCallback(
    (itemIds: string[]) => {
      const selectedNodes: string[] = [];
      for (const id of itemIds) {
        const n = rosCtx.nodeMap.get(id);
        if (n) {
          selectedNodes.push(id);
        }
      }
      navCtx.setSelectedNodes(selectedNodes, false);
    },
    [navCtx, rosCtx.nodeMap]
  );

  /**
   * Callback when provides on the tree are selected by the user
   */
  function handleProviderSelect(providerIds: string[]): void {
    // set the selected nodes
    const selectedProvidersLocal: string[] = [];
    // update setSelectedNodes based on trees selectedItems
    for (const id of providerIds) {
      // search selected host
      if (rosCtx.getProviderById(id)) {
        selectedProvidersLocal.push(id);
      }
    }
    navCtx.setSelectedProviders(selectedProvidersLocal);
  }

  /**
   * Create and open a new panel with a [NodeLoggerPanel] for a given node
   */
  function createLoggerPanel(node: RosNode): void {
    const id = `node-logger-${node.idGlobal}`;
    const title = node.name;
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        id,
        title,
        <NodeLoggerPanel node={node} />,
        true,
        LAYOUT_TAB_SETS[settingsCtx.get("nodeLoggerOpenLocation") as string],
        new LayoutTabConfig(false, "node-logger")
      )
    );
  }

  /**
   * Start nodes from a list of itemIds
   */
  function createLoggerPanelFromId(itemIds: string[]): void {
    const nodeList = getNodesFromIds(itemIds);
    if (nodeList.length > 0) {
      createLoggerPanel(nodeList[0]);
    }
  }

  /**
   * Create and open a new panel with a [SingleTerminalPanel] for a given node
   */
  async function createSingleTerminalPanel(
    type: CmdType,
    providerId: string,
    nodeName: string,
    screen: string,
    externalKeyModifier: boolean = false,
    openInTerminal: boolean = false
  ): Promise<void> {
    return navCtx.openTerminal(type, providerId, nodeName, screen, "", externalKeyModifier, openInTerminal);
  }

  /**
   * Create and open a new panel with a [createFileEditorPanel] for selected nodes
   */
  function createFileEditorPanel(nodes: RosNode[], external: boolean): void {
    if (nodes.length > 0) {
      // open only for first node
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
        navCtx.openEditor(
          node.providerId,
          rootLaunch,
          launchInfo.file_name || "",
          launchInfo.file_range,
          launchInfo.topLevelArgs,
          external
        );
      }
    }
  }

  /**
   * Create and open a new panel with a [ParameterPanel] for selected nodes
   */
  function createParameterPanel(nodes: RosNode[], providers: string[]): void {
    const openLocation: string = LAYOUT_TAB_SETS[settingsCtx.get("nodeParamOpenLocation") as string];
    const params: { name: string; callback: () => void }[] = [];
    for (const node of nodes) {
      params.push({
        name: node.name,
        callback: () => {
          emitCustomEvent(
            EVENT_OPEN_COMPONENT,
            eventOpenComponent(
              `parameter-node-${node.idGlobal}`,
              `${node.name}`,
              <ParameterPanel nodes={[node]} providers={[]} />,
              true,
              openLocation,
              new LayoutTabConfig(false, "parameter")
            )
          );
        },
      });
    }
    for (const providerId of providers) {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        params.push({
          name: provider.name(),
          callback: () => {
            emitCustomEvent(
              EVENT_OPEN_COMPONENT,
              eventOpenComponent(
                `parameter-provider-${provider}`,
                `${provider.name()}`,
                <ParameterPanel nodes={[]} providers={[providerId]} />,
                true,
                openLocation,
                new LayoutTabConfig(false, "parameter")
              )
            );
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
  }

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

  function getNodeLetManager(node: RosNode, ignoreRunState: boolean, nodes2start: RosNode[]): RosNode | null {
    if (!node) return null;
    const composableParent = node.getLaunchComposableContainer();
    if (composableParent) {
      const provider = rosCtx.getProviderById(node.providerId);
      const nodeNms = provider?.rosNodes.filter((node) => node.name === composableParent) || [];
      const nodeNm = nodeNms.length > 0 ? nodeNms[0] : null;
      // not running or ignoreRunState and in the list with nodes to start
      // and not already in the start queue
      if (
        nodeNm &&
        (nodeNm.status !== RosNodeStatus.RUNNING ||
          (ignoreRunState && nodes2start.filter((item) => item.id === nodeNm.id).length > 0)) &&
        queue.queue &&
        queue.queue.find((elem) => {
          return elem.action === "START" && elem.node?.name === nodeNm.name;
        }) === undefined
      ) {
        return nodeNm;
      }
    }
    return null;
  }

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
    // update nodes with launchFile if they are provided (on multiple launch files for a node)
    let nodeList = nodes.map((node) => {
      node.launchPath = useLaunchFiles[node.name];
      return node;
    });
    // check nodes for associations and extend start node list
    nodeList = updateWithAssociations(nodeList);
    const add2start: (node: RosNode | null) => void = (node) => {
      // add only valid node and if it is not already added
      if (node && node2Start.filter((item) => item.id === node.id).length === 0) {
        node.ignore_timer = ignoreTimer;
        node2Start.push(node);
      }
    };
    for (const node of nodeList) {
      // ignore running and nodes already in the queue
      if (!ignoreRunState && node.status === RosNodeStatus.RUNNING) {
        skippedNodes.set(node.name, "already running");
      } else if (
        queue.queue?.find((elem) => {
          return elem.action === "START" && elem.node?.name === node.name;
        })
      ) {
        skippedNodes.set(node.name, "already in the start queue");
      } else if (node.launchInfo.size > 0) {
        // prepend nodeLet manager to the start list if it is not already added,
        // not running or in the list with started nodes
        const managerNode = getNodeLetManager(node, ignoreRunState, nodes);
        if (managerNode) {
          managerNode.launchPath = node.launchPath;
        }
        add2start(managerNode);
        add2start(node);
        if (node.launchInfo.size > 1 && !node.launchPath) {
          // Multiple launch files available
          withMultiLaunch.push(node);
        }
      } else if (!node.system_node) {
        // no launch files
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

  /**
   * Start nodes in the selected list
   */
  function startSelectedNodes(ignoreTimer: boolean = false): void {
    startNodesWithLaunchCheck(getSelectedNodes(), false, {}, ignoreTimer);
  }

  /**
   * Start nodes from a list of itemIds
   */
  function startNodesFromId(itemIds: string[]): void {
    const nodeList = getNodesFromIds(itemIds);
    startNodesWithLaunchCheck(nodeList);
  }

  /** Stop node from queue and trigger the next one. */
  async function stopNodeQueued(node: RosNode | undefined): Promise<void> {
    if (node !== undefined) {
      logCtx.debug(`stop: ${node.name}`, "");
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        queue.addStatus("STOP", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        if (node.status === RosNodeStatus.RUNNING) {
          // stop node and store result for message
          const resultStopNode = await provider.stopNode(node.id);
          if (!resultStopNode.result) {
            if (queue.queue.length <= 1) {
              const getProcessResult = await provider.findNodeProcess(node.name);
              if (getProcessResult.processes.length > 0) {
                setKillProcessQuestion(
                  getProcessResult.processes.map((p) => {
                    return {
                      node: node.name,
                      pid: p.pid,
                      cmdLine: p.cmdLine,
                      callback: async (): Promise<void> => {
                        await provider.killProcess(p.pid);
                      },
                    };
                  })
                );
              }
            }
            queue.addStatus("STOP", node.name, false, resultStopNode.message);
          } else {
            queue.addStatus("STOP", node.name, true, "stopped");
          }
        } else if ((node.screens || []).length > 0) {
          // terminate screen and store result for message
          const resultTermNode = await provider.screenKillNode(node.id, "SIGTERM");
          if (!resultTermNode.result) {
            queue.addStatus("STOP", node.name, false, resultTermNode.message);
          } else {
            queue.addStatus("STOP", node.name, true, "sent SIGTERM to executable");
          }
        } else {
          queue.addStatus("STOP", node.name, false, "no screen to stop");
        }
      }
    } else {
      queue.addStatus("STOP", "undefined", false, "invalid node");
    }

    return Promise.resolve();
  }

  /**
   * Stops the given nodes.
   */
  function stopNodes(
    nodes: RosNode[],
    onlyWithLaunch?: boolean,
    restart?: boolean,
    ignoreTimer: boolean = false
  ): void {
    const skipped: Record<string, string> = {};
    const nodeList = updateWithAssociations(nodes);

    // Precompute STOP items already in queue (O(1) lookup)
    const stopQueuedNames = new Set(
      (queue.queue ?? []).filter((q) => q.action === "STOP" && q.node?.name).map((q) => q.node?.name)
    );

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

      if (stopQueuedNames.has(node.name)) {
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
      // Enqueue STOP
      queue.update(nodesToStop.map((node) => ({ node, action: "STOP" })));

      // Determine global max kill timeout (instead of per-node timer bug)
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

      // Add KILL step once after max timeout
      if (maxKillTime > 0) {
        setTimeout(() => {
          queue.update(nodesKillTimeout.map((node) => ({ node, action: "KILL" })));
        }, maxKillTime);
      }
    }

    // Restart logic
    if (restart) {
      const restartDelay = maxKillTime > 0 ? maxKillTime + 500 : 0;

      setTimeout(() => {
        startNodesWithLaunchCheck(nodeList, true, {}, ignoreTimer);
      }, restartDelay);
    }
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
  function stopNodesFromId(itemIds: string[]): void {
    const nodeList = getNodesFromIds(itemIds);
    stopNodes(nodeList);
  }

  /**
   * Restart nodes given in the arguments
   */
  function restartNodes(nodeList: RosNode[], onlyWithLaunch: boolean, ignoreTimer: boolean = false): void {
    stopNodes(nodeList, onlyWithLaunch, true, ignoreTimer); // => true, for restart
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
    getSelectedNodes().map(async (node) => {
      // we kill system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      if (
        queue.queue?.find((elem) => {
          return elem.action === "KILL" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2kill.push(node);
      }
    });
    queue.update(
      nodes2kill.map((node) => {
        return { node, action: "KILL" };
      })
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
    getSelectedNodes().map(async (node) => {
      // we unregister system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      // not supported for ROS2
      if (!node.masteruri) return;
      if (
        queue.queue?.find((elem) => {
          return elem.action === "UNREGISTER" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2unregister.push(node);
      }
    });
    queue.update(
      nodes2unregister.map((node) => {
        return { node, action: "UNREGISTER" };
      })
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
      queue.queue?.find((elem) => {
        return elem.action === "DYNAMIC_RECONFIGURE" && elem.service === service && elem.masteruri === masteruri;
      })
    ) {
      // TODO: skippedNodes.set(service, 'already in queue');
    } else {
      queue.update([{ action: "DYNAMIC_RECONFIGURE", service: service, masteruri: masteruri }]);
    }
  }

  /** start dynamic reconfigure in the queue and trigger the next one. */
  async function dynamicReconfigureQueued(service: string | undefined, masteruri: string | undefined): Promise<void> {
    if (service) {
      // store result for message
      const result = await rosCtx.startDynamicReconfigureClient(service, masteruri || "");
      if (!result.result) {
        queue.addStatus("DYNAMIC_RECONFIGURE", service, false, result.message);
      } else {
        queue.addStatus("DYNAMIC_RECONFIGURE", service, true, "dynamic reconfigure started");
      }
    }
    return Promise.resolve();
  }

  /**
   * Remove log files from ROS nodes using [rosclean purge] for a given provider
   */
  async function clearProviderLogs(providers: string[]): Promise<void> {
    if (!providers || providers.length === 0) return;

    try {
      await Promise.all(
        providers.map(async (providerId) => {
          const provider = rosCtx.getProviderById(providerId);
          if (!provider || !provider.isAvailable()) return;

          try {
            const result: Result = await provider.rosCleanPurge();

            if (!result.result) {
              logCtx.error("Could not delete logs", result.message, "logs not deleted");
            } else if (!result.message.includes("Purging ROS node logs")) {
              logCtx.error("Unexpected result during log purge", result.message, "logs not deleted");
            } else {
              logCtx.success("Logs removed successfully", result.message, "logs removed");
            }
          } catch (err) {
            const message = err instanceof Error ? err.message : String(err);
            logCtx.error(`Error purging logs for provider ${providerId}`, message, "logs not deleted");
          }
        })
      );
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
    nodes.map(async (node) => {
      // we unregister system nodes only when they are individually selected
      if (node.system_node && nodes.length > 1) return;
      if (
        queue.queue?.find((elem) => {
          return elem.action === "CLEAR_LOG" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2clear.push(node);
      }
    });
    queue.update(
      nodes2clear.map((node) => {
        return { node, action: "CLEAR_LOG" };
      })
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

  /**
   * Synchronizes providerNodes with the current provider list.
   *
   * Removes all nodes whose providerId is no longer present in
   * rosCtx.providers. This prevents orphaned nodes from remaining
   * in state when providers are removed or replaced.
   */
  const readNodes = useCallback(() => {
    const providerIds = rosCtx.providers.map((p) => p.id);
    setProviderNodes((oldValues) => oldValues.filter((item) => providerIds.includes(item.providerId)));
  }, [rosCtx.providers]);

  // Register useEffect Callbacks ----------------------------------------------------------------------------------

  useEffect(() => {
    // apply filter to nodes if search text was changed by user or nodes are updated by provider
    onSearch(filterText);
  }, [filterText, providerNodes]);

  useEffect(() => {
    readNodes();
  }, [rosCtx.providers]);

  useEffect(() => {
    if (!nodesToStart) return;

    // Enqueue all selected nodes with START action
    queue.update(nodesToStart.map((node) => ({ node, action: "START" })));

    setNodesToStart(undefined);
  }, [nodesToStart]);

  const queueActionHandlers: Record<string, (item: TQueueAction) => Promise<void>> = {
    START: (item) => startNodeQueued(item.node),
    STOP: (item) => stopNodeQueued(item.node),
    UNREGISTER: (item) => unregisterNodeQueued(item.node),
    KILL: (item) => killNodeQueued(item.node),
    CLEAR_LOG: (item) => clearNodeLogQueued(item.node),
    DYNAMIC_RECONFIGURE: (item) => dynamicReconfigureQueued(item.service, item.masteruri),
  };

  async function performQueueMain(index: number): Promise<void> {
    if (index < 0) return;

    const queueItem = queue.get();

    if (queueItem) {
      const handler = queueActionHandlers[queueItem.action];

      if (handler) {
        await handler(queueItem);
      } else {
        console.warn(`Unknown queue action: ${JSON.stringify(queueItem)}`);
      }
      return;
    }

    // ---- Queue finished ----
    for (const action of Object.keys(queueActionMeta)) {
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
  }

  // update queue
  useEffect(() => {
    performQueueMain(queue.currentIndex);
  }, [queue.currentIndex]);

  const createButtonBox = useMemo(() => {
    const selectedNodes = getSelectedNodes();
    return (
      <ButtonGroup orientation="vertical" aria-label="ros node control group">
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Start selected nodes
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift:
                </Typography>
                <Typography fontSize={"inherit"}>ignore start timer</Typography>
              </Stack>
            </div>
          }
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Start"
              onClick={(event) => {
                startSelectedNodes(event.nativeEvent.shiftKey);
              }}
              disabled={selectedNodes.length === 0}
            >
              <PlayArrowIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Stop selected nodes
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift:
                </Typography>
                <Typography fontSize={"inherit"}>kill</Typography>
              </Stack>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Ctrl+Shift:
                </Typography>
                <Typography fontSize={"inherit"}>Unregister ROS1 nodes</Typography>
              </Stack>
            </div>
          }
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Stop"
              onClick={(event) => {
                if (event.nativeEvent.shiftKey) {
                  if (event.nativeEvent.ctrlKey) {
                    unregisterSelectedNodes();
                  } else {
                    killSelectedNodes();
                  }
                } else {
                  stopSelectedNodes();
                }
              }}
              disabled={selectedNodes.length === 0}
            >
              <StopIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Restart selected nodes
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift:
                </Typography>
                <Typography fontSize={"inherit"}>ignore start timer</Typography>
              </Stack>
            </div>
          }
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Restart"
              onClick={(event) => {
                restartSelectedNodes(event.nativeEvent.shiftKey);
              }}
              disabled={selectedNodes.length === 0}
            >
              <RestartAltIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Divider />
        {showButtonsForKeyModifiers && (
          <Stack>
            <Tooltip title="Kill" placement="left" disableInteractive>
              <span>
                <IconButton
                  size="medium"
                  aria-label="Kill"
                  onClick={() => {
                    killSelectedNodes();
                  }}
                  disabled={selectedNodes.length === 0}
                >
                  <CancelPresentationIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip title="Unregister ROS1 nodes" placement="left" disableInteractive>
              <span>
                <IconButton
                  size="medium"
                  aria-label="Unregister"
                  onClick={() => {
                    unregisterSelectedNodes();
                  }}
                  disabled={selectedNodes.filter((node) => (node.masteruri || "").length > 0).length === 0}
                >
                  <DeleteForeverIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Divider />
          </Stack>
        )}
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Edit
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift+click:
                </Typography>
                <Typography fontSize={"inherit"}>alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Edit"
              onClick={(event) => {
                createFileEditorPanel(getSelectedNodes(), event.nativeEvent.shiftKey);
              }}
              disabled={selectedNodes.filter((node) => node.launchInfo.size > 0).length === 0}
            >
              <BorderColorIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title="Parameters"
          placement="left"
          // enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Parameters"
              onClick={() => {
                if (navCtx.selectedProviders?.length > 0) {
                  createParameterPanel([], navCtx.selectedProviders);
                } else {
                  createParameterPanel(getSelectedNodes(), []);
                }
              }}
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
            >
              <TuneIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        {selectedNodes.filter((node) => {
          return node.dynamicReconfigureServices?.length > 0;
        }).length > 0 && (
          <Tooltip title="Dynamic reconfigure for ROS1 nodes" placement="left" disableInteractive>
            <span>
              <IconButton
                size="medium"
                aria-label="dynamic reconfigure"
                onClick={() => {
                  let countDri = 0;
                  const driItems: RosNode[] = [];
                  for (const node of getSelectedNodes()) {
                    countDri += node.dynamicReconfigureServices.length;
                    driItems.push(node);
                  }
                  if (countDri > 2) {
                    setDynamicReconfigureItems(driItems);
                  } else {
                    for (const node of driItems) {
                      for (const dri of node.dynamicReconfigureServices) {
                        startDynamicReconfigure(dri, node.masteruri || "");
                      }
                    }
                  }
                }}
                // disabled={
                //   selectedNodes.filter((node) => {
                //     return node.dynamicReconfigureServices?.length > 0;
                //   }).length === 0
                // }
              >
                <SettingsSuggestIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
        )}
        <Divider />
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Open screen of the node
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift+click:
                </Typography>
                <Typography fontSize={"inherit"}>alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Screen"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                if (navCtx.selectedProviders?.length > 0) {
                  const screens: TMenuOptionsScreen[] = []; // {node : string, screen: string, callback: () => void, external: boolean}
                  for (const providerId of navCtx.selectedProviders || []) {
                    const prov = rosCtx.getProviderById(providerId);
                    for (const screenMap of prov?.screens || []) {
                      for (const screen of screenMap.screens || []) {
                        screens.push({
                          nodeName: screenMap.name,
                          providerId: prov?.id || "",
                          screen: screen,
                          external: event.nativeEvent.shiftKey,
                        });
                      }
                    }
                    setNodeScreens(screens);
                    if (screens.length === 0) {
                      logCtx.info("no screens found", "", "no screens found");
                    }
                  }
                } else {
                  const screens: TMenuOptionsScreen[] = []; // {node : string, screen: string, callback: () => void, external: boolean}
                  for (const node of getSelectedNodes()) {
                    if (node.screens && node.screens.length === 1) {
                      // 1 screen available
                      screens.push({
                        nodeName: node.name,
                        providerId: node.providerId,
                        screen: node.screens[0],
                        callback: () => {
                          if (node.screens) {
                            createSingleTerminalPanel(
                              CmdType.SCREEN,
                              node.providerId,
                              node.name,
                              node.screens[0],
                              event.nativeEvent.shiftKey,
                              event.nativeEvent.ctrlKey
                            );
                          }
                        },
                      });
                    } else if (node.screens && node.screens.length > 1) {
                      // Multiple screens available
                      node.screens.map((screen) => {
                        screens.push({
                          nodeName: node.name,
                          providerId: node.providerId,
                          screen: screen,
                          callback: () => {
                            createSingleTerminalPanel(
                              CmdType.SCREEN,
                              node.providerId,
                              node.name,
                              screen,
                              event.nativeEvent.shiftKey,
                              event.nativeEvent.ctrlKey
                            );
                          },
                        });
                      });
                    } else {
                      // no screens, try to find by node name instead
                      screens.push({
                        nodeName: node.name,
                        providerId: node.providerId,
                        screen: "autodetect",
                        callback: () => {
                          createSingleTerminalPanel(
                            CmdType.SCREEN,
                            node.providerId,
                            node.name,
                            "",
                            event.nativeEvent.shiftKey,
                            event.nativeEvent.ctrlKey
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
                          CmdType.SCREEN,
                          item.providerId,
                          item.nodeName,
                          item.screen,
                          item.external
                        );
                      }
                    }
                  }
                }
              }}
            >
              {navCtx.selectedProviders?.length > 0 ? (
                <AddToQueueIcon fontSize="inherit" />
              ) : (
                <DvrIcon fontSize="inherit" />
              )}
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Open log of the node
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight={"bold"} fontSize={"inherit"}>
                  Shift+click:
                </Typography>
                <Typography fontSize={"inherit"}>alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Log"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                const logs: TMenuOptionsNode[] = []; // {node : string, callback: () => void}
                for (const node of getSelectedNodes()) {
                  logs.push({
                    node: node,
                    callback: () => {
                      createSingleTerminalPanel(
                        CmdType.LOG,
                        node.providerId,
                        node.name,
                        "",
                        event.nativeEvent.shiftKey,
                        event.nativeEvent.ctrlKey
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
              }}
            >
              <WysiwygIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip title="Change log level" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Log Level"
              disabled={selectedNodes.length === 0}
              onClick={() => {
                const loggers: TMenuOptionsNode[] = []; // {node : string, callback: () => void}
                for (const node of getSelectedNodes()) {
                  loggers.push({
                    node: node,
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
              }}
            >
              <SettingsInputCompositeOutlinedIcon fontSize="inherit" sx={{ rotate: "90deg" }} />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={navCtx.selectedProviders?.length > 0 ? "ros clean purge" : "Clear Logs"}
          placement="left"
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Clear Logs"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={() => {
                if (navCtx.selectedProviders?.length > 0) {
                  setRosCleanPurge(true);
                } else {
                  clearLogs(getSelectedNodes());
                }
              }}
            >
              <DeleteSweepIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        {navCtx.selectedProviders?.length > 0 && <Divider />}
        {navCtx.selectedProviders?.length > 0 && (
          <Tooltip
            title="Open Terminal on selected host (external terminal with shift+click)"
            placement="left"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <span>
              <IconButton
                size="medium"
                aria-label="Open Terminal on selected host"
                disabled={navCtx.selectedProviders?.length === 0}
                onClick={(event) => {
                  // open a new terminal for each selected provider
                  for (const providerId of navCtx.selectedProviders) {
                    createSingleTerminalPanel(
                      CmdType.TERMINAL,
                      providerId,
                      "",
                      "",
                      event.nativeEvent.shiftKey,
                      event.nativeEvent.ctrlKey
                    );
                  }
                }}
              >
                <TerminalIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
        )}
      </ButtonGroup>
    );
  }, [navCtx.selectedNodes, navCtx.selectedProviders, providerNodes, tooltipDelay]);

  return (
    <Box
      // ref={panelRef}
      width="100%"
      height="100%"
      overflow="auto"
      sx={{ backgroundColor: backgroundColor }}
    >
      <Stack spacing={0.5} direction="column" width="100%" height="100%">
        {queue.currentIndex < 0 && (
          <Stack direction="row" spacing={0.5} alignItems="center">
            {buttonLocation === BUTTON_LOCATIONS.LEFT && (
              <Tooltip
                title="Reload node list"
                placement="left"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
                disableInteractive
              >
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
              <Tooltip
                title="Reload node list"
                placement="left"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
                disableInteractive
              >
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
          {buttonLocation === BUTTON_LOCATIONS.LEFT && <Box height="100%">{createButtonBox}</Box>}
          <HostTreeView
            // providerNodeTree={providerNodeTree}
            visibleNodes={visibleNodes}
            isFiltered={filterText.length > 0}
            onNodeSelect={handleNodesSelect}
            onProviderSelect={handleProviderSelect}
            showLoggers={createLoggerPanelFromId}
            startNodes={startNodesFromId}
            stopNodes={stopNodesFromId}
          />
          {buttonLocation === BUTTON_LOCATIONS.RIGHT && <Box height="100%">{createButtonBox}</Box>}
        </Stack>
      </Stack>

      {rosCleanPurge && (
        <ConfirmModal
          title="Question"
          message="Confirm to remove all ros log files."
          onConfirmCallback={() => {
            setRosCleanPurge(false);
            clearProviderLogs(navCtx.selectedProviders);
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
                    CmdType.SCREEN,
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
              const nodeWithOpt = nodeLogs.find((nodeItem) => `${nodeItem.node.name}` === item);
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
            const useLaunchFiles = {};
            for (const item of items) {
              for (const launch of item.list) {
                const node = nodeMultiLaunches.find((n) => n.name.includes(item.title));
                if (node) {
                  useLaunchFiles[node.name] = launch;
                }
              }
            }
            // setNodesToStart(nodesAwaitModal);
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
              const node = dynamicReconfigureItems.find((node) => node.name === item.title);
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
          title={`Kill selected processes for node ${killProcessQuestion[0].node}`}
          selectOnOpen={0}
          list={killProcessQuestion.reduce((prev: string[], item) => {
            prev.push(`${item.pid}: ${item.cmdLine}`);
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            for (const item of items) {
              const listWithOpt = killProcessQuestion.find(
                (listItem) => `${listItem.pid}: ${listItem.cmdLine}` === item
              );
              if (listWithOpt?.callback) {
                listWithOpt.callback();
              }
            }
            setKillProcessQuestion([]);
          }}
          onCancelCallback={() => {
            setKillProcessQuestion([]);
          }}
        />
      )}
    </Box>
  );
}
