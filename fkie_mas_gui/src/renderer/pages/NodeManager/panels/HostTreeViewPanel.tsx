import AddToQueueIcon from "@mui/icons-material/AddToQueue";
import BorderColorIcon from "@mui/icons-material/BorderColor";
import CancelPresentationIcon from "@mui/icons-material/CancelPresentation";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import DeleteSweepIcon from "@mui/icons-material/DeleteSweep";
import DvrIcon from "@mui/icons-material/Dvr";
import LocalPlayOutlinedIcon from "@mui/icons-material/LocalPlayOutlined";
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
  AlertTitle,
  Box,
  ButtonGroup,
  Divider,
  FormLabel,
  IconButton,
  LinearProgress,
  Paper,
  Stack,
  ToggleButton,
  Tooltip,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import HostTreeView from "@/renderer/components/HostTreeView/HostTreeView";
import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import ListSelectionModal from "@/renderer/components/SelectionModal/ListSelectionModal";
import MapSelectionModal, { MapSelectionItem } from "@/renderer/components/SelectionModal/MapSelectionModal";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { NavigationContext } from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useQueue from "@/renderer/hooks/useQueue";
import { Result, RosNode, RosNodeStatus } from "@/renderer/models";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import {
  EVENT_FILTER_NODES,
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
  TEventId,
} from "@/renderer/pages/NodeManager/layout/events";
import { CmdType } from "@/renderer/providers";
import { EventProviderRestartNodes, EventProviderRosNodes } from "@/renderer/providers/events";
import { EVENT_PROVIDER_RESTART_NODES, EVENT_PROVIDER_ROS_NODES } from "@/renderer/providers/eventTypes";
import { TResultClearPath } from "@/renderer/providers/ProviderConnection";
import { findIn } from "@/renderer/utils/index";
import { TFileRange, TLaunchArg } from "@/types";
import NodeLoggerPanel from "./NodeLoggerPanel";
import ParameterPanel from "./ParameterPanel";

type TProviderNodes = {
  providerId: string;
  nodes: RosNode[];
};

type TQueueAction = {
  action: string;
  node?: RosNode;
  service?: string;
  masteruri?: string;
};

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

export default function HostTreeViewPanel(): JSX.Element {
  // context objects
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);

  // state variables
  const [showRemoteNodes, setShowRemoteNodes] = useState<boolean>(settingsCtx.get("showRemoteNodes") as boolean);
  const [showButtonsForKeyModifiers, setShowButtonsForKeyModifiers] = useState<boolean>(
    settingsCtx.get("showButtonsForKeyModifiers") as boolean
  );
  const [tooltipDelay, setTooltipDelay] = useState(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  const [filterText, setFilterText] = useState("");
  const [providerNodes, setProviderNodes] = useState<TProviderNodes[]>([]);
  const [visibleNodes, setVisibleNodes] = useState<RosNode[]>([]);
  const [nodesToStart, setNodesToStart] = useState<RosNode[]>();
  const [progressQueueMain, setProgressQueueMain] = useState<number>(0);
  const {
    update: updateQueueMain,
    clear: clearQueueMain,
    get: getQueueMain,
    queue: queueItemsQueueMain,
    currentIndex: indexQueueMain,
    success: successQueueMain,
    failed: failedQueueMain,
    addStatus: addStatusQueueMain,
  } = useQueue<TQueueAction>(setProgressQueueMain);

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

  useEffect(() => {
    setShowRemoteNodes(settingsCtx.get("showRemoteNodes") as boolean);
    setShowButtonsForKeyModifiers(settingsCtx.get("showButtonsForKeyModifiers") as boolean);
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx, settingsCtx.changed]);

  /**
   * Get list of nodes from a list of node.idGlobal
   */
  const getNodesFromIds = useCallback(
    (itemIds: string[]) => {
      const nodeList: RosNode[] = [];
      itemIds.forEach((item) => {
        const node = rosCtx.nodeMap.get(item);
        if (node) {
          nodeList.push(node);
        }
      });
      return nodeList;
    },
    [rosCtx.nodeMap]
  );

  const getSelectedNodes = useCallback(() => {
    return getNodesFromIds(navCtx.selectedNodes);
  }, [navCtx.selectedNodes]);

  // debounced search callback
  // search in the origin node list and create a new tree
  const onSearch = useDebounceCallback((searchTerm: string) => {
    const newVisibleNodes: RosNode[] = [];
    providerNodes.forEach((item) => {
      const { nodes } = item;
      nodes.forEach((node) => {
        // filter nodes by user text
        if (searchTerm.length > 0) {
          const isMatch = findIn(searchTerm, [node.name, node.group, node.providerName, node.guid || ""]);
          if (!isMatch) return;
        }
        newVisibleNodes.push(node);
      });
    });
    setVisibleNodes(newVisibleNodes);
  }, 300);

  useCustomEventListener(
    EVENT_PROVIDER_ROS_NODES,
    (data: EventProviderRosNodes) => {
      // put in our providerNodes list
      const { provider, nodes } = data;
      // TODO: should we remove closed/lost provider infos
      setProviderNodes((oldValues) => [
        ...oldValues.filter((item) => {
          return item.providerId !== provider.id;
        }),
        {
          providerId: provider.id,
          nodes: nodes,
        },
      ]);
    },
    [setProviderNodes, settingsCtx]
  );

  useCustomEventListener(EVENT_PROVIDER_RESTART_NODES, (data: EventProviderRestartNodes) => {
    restartNodes(data.nodes, true);
  });

  useCustomEventListener(EVENT_FILTER_NODES, (data: TEventId) => {
    setFilterText(data.id);
  });

  // Register Callbacks ----------------------------------------------------------------------------------

  /**
   * Callback when nodes on the tree are selected by the user
   */
  const handleNodesSelect = useCallback(
    (itemIds: string[]) => {
      const selectedNoes: string[] = [];
      itemIds.forEach((id) => {
        const n = rosCtx.nodeMap.get(id);
        if (n) {
          selectedNoes.push(id);
        }
      });
      navCtx.setSelectedNodes(selectedNoes);
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
    providerIds.forEach((id: string) => {
      // search selected host
      if (rosCtx.getProviderById(id)) {
        selectedProvidersLocal.push(id);
      }
    });
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
        logCtx.error(`Could not find launch file for node: [${node.name}]`, `Node Info: ${JSON.stringify(node)}`);
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
          launchInfo.launch_context_arg || [],
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
    nodes.forEach((node) => {
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
    });
    providers.forEach((providerId) => {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        params.push({
          name: provider.name(),
          callback: () => {
            emitCustomEvent(
              EVENT_OPEN_COMPONENT,
              eventOpenComponent(
                `parameter-provider-${provider}`,
                `${provider}`,
                <ParameterPanel nodes={[]} providers={[providerId]} />,
                true,
                openLocation,
                new LayoutTabConfig(false, "parameter")
              )
            );
          },
        });
      }
    });
    if (params.length >= 3) {
      setNodeParams(params);
    } else {
      params.forEach((item) => {
        item.callback();
      });
    }
  }

  async function startNodeQueued(node: RosNode | undefined): Promise<void> {
    // let node = getQueueMain();
    if (node !== undefined) {
      const provider = rosCtx.getProviderById(node.providerId);
      logCtx.debug(`start: ${node.name}`, "");
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("START", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        // store result to inform the user after queue is finished
        const resultStartNode = await provider.startNode(node);
        if (!resultStartNode.success) {
          addStatusQueueMain("START", node.name, false, `${resultStartNode.details}`);
        } else {
          addStatusQueueMain("START", node.name, true, "started");
        }
      }
    }
  }

  function updateWithAssociations(nodes: RosNode[], depth?: number): RosNode[] {
    if (depth === undefined) depth = 0;
    if (depth > 10) return [...nodes];
    const newNodeList: RosNode[] = [];
    nodes.forEach((node) => {
      if (node.launchInfo.size > 0) {
        const associations: string[] =
          node.launchInfo.size === 1
            ? node.launchInfo.values().next().value?.associations || []
            : node.launchInfo.get(node.launchPath)?.associations || [];
        if (associations) {
          const provider = rosCtx.getProviderById(node.providerId);
          if (provider) {
            associations.forEach((asNodeName: string) => {
              const asNodes = provider.rosNodes.filter((n) => n.name === asNodeName);
              const asNodesRec = updateWithAssociations(asNodes, depth + 1);
              asNodesRec.forEach((asNode) => {
                if (!newNodeList.find((n) => n.name === asNode.name)) {
                  newNodeList.push(asNode);
                }
              });
            });
          }
        }
      }
      if (!newNodeList.find((n) => n.name === node.name)) {
        newNodeList.push(node);
      }
    });
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
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
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
    useLaunchFiles: { [key: string]: string } = {}
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
        node2Start.push(node);
      }
    };
    nodeList.forEach((node) => {
      // ignore running and nodes already in the queue
      if (!ignoreRunState && node.status === RosNodeStatus.RUNNING) {
        skippedNodes.set(node.name, "already running");
      } else if (
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
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
    });
    if (skippedNodes.size > 0) {
      logCtx.debug(`Skipped ${skippedNodes.size} nodes`, JSON.stringify(Object.fromEntries(skippedNodes)));
    }
    if (withNoLaunch.length > 0) {
      withNoLaunch.forEach((nodeName) => {
        skippedNodes.set(nodeName, "no launch file");
      });
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
  function startSelectedNodes(): void {
    startNodesWithLaunchCheck(getSelectedNodes());
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
        addStatusQueueMain("STOP", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        if (node.status === RosNodeStatus.RUNNING) {
          // stop node and store result for message
          const resultStopNode = await provider.stopNode(node.id);
          if (!resultStopNode.result) {
            addStatusQueueMain("STOP", node.name, false, resultStopNode.message);
          } else {
            addStatusQueueMain("STOP", node.name, true, "stopped");
          }
        } else if ((node.screens || []).length > 0) {
          // terminate screen and store result for message
          const resultTermNode = await provider.screenKillNode(node.id, "SIGTERM");
          if (!resultTermNode.result) {
            addStatusQueueMain("STOP", node.name, false, resultTermNode.message);
          } else {
            addStatusQueueMain("STOP", node.name, true, "sent SIGTERM to executable");
          }
        }
      }
    }
    return Promise.resolve();
  }

  /**
   * Stop nodes given in the arguments
   */
  function stopNodes(nodes: RosNode[], onlyWithLaunch?: boolean, restart?: boolean): void {
    const nodes2stop: RosNode[] = [];
    const skippedNodes: Map<string, string> = new Map();
    const nodeList = updateWithAssociations(nodes);
    nodeList.forEach((node) => {
      // we stop system nodes only when they are individually selected
      if (node.system_node && nodeList.length > 1) {
        // skip system nodes
        skippedNodes.set(node.name, "system node");
      } else if (onlyWithLaunch && node.launchInfo.size === 0) {
        skippedNodes.set(node.name, "stop only with launch files");
      } else if (
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "STOP" && elem.node?.name === node.name;
        })
      ) {
        skippedNodes.set(node.name, "already in queue");
      } else if (node.status === RosNodeStatus.RUNNING || (node.screens || []).length > 0) {
        // const isRunning = node.status === RosNodeStatus.RUNNING;
        // const isRunning = true;
        // if (!skip) {
        nodes2stop.push(node);
      } else {
        skippedNodes.set(node.name, "not running");
      }
    });
    if (skippedNodes.size > 0) {
      logCtx.debug(`Skipped ${skippedNodes.size} nodes`, JSON.stringify(Object.fromEntries(skippedNodes)));
    }
    updateQueueMain(
      nodes2stop.map((node) => {
        return { node, action: "STOP" };
      })
    );
    let maxKillTime = -1;
    // add kill on stop commands
    nodes2stop.forEach((node) => {
      if (node.pid) {
        let killTime: number = -1;
        node.launchInfo.forEach((launchInfo) => {
          launchInfo.parameters?.forEach((param) => {
            // TODO: search for kill parameter in ros2
            if (param.name.endsWith("/nm/kill_on_stop")) {
              if (killTime === -1)
                killTime = typeof param.value === "string" ? parseInt(param.value) : (param.value as number);
            }
          });
        });

        if (killTime > -1) {
          if (maxKillTime < killTime) {
            maxKillTime = killTime;
          }
          setTimeout(() => {
            updateQueueMain([{ node, action: "KILL" }]);
          }, killTime);
        }
      }
    });
    if (restart) {
      if (maxKillTime > -1) {
        // wait until all timers are expired before start nodes if kill timer was used while stop nodes
        setTimeout(() => {
          startNodesWithLaunchCheck(nodeList, true);
        }, maxKillTime + 500);
      } else {
        startNodesWithLaunchCheck(nodeList, true);
      }
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
  function restartNodes(nodeList: RosNode[], onlyWithLaunch: boolean): void {
    stopNodes(nodeList, onlyWithLaunch, true); // => true, for restart
  }

  /**
   * Restart nodes in the selected list
   */
  function restartSelectedNodes(): void {
    restartNodes(getSelectedNodes(), true);
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
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "KILL" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2kill.push(node);
      }
    });
    updateQueueMain(
      nodes2kill.map((node) => {
        return { node, action: "KILL" };
      })
    );
  }

  /** Kill node in the queue and trigger the next one. */
  async function killNodeQueued(node: RosNode | undefined): Promise<void> {
    if (node !== undefined) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("KILL", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        // store result for message
        const resultKillNode = await provider.screenKillNode(node.name);
        if (!resultKillNode.result) {
          addStatusQueueMain("KILL", node.name, false, resultKillNode.message);
        } else {
          addStatusQueueMain("KILL", node.name, true, "killed");
        }
      }
    }
    return Promise.resolve();
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
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "UNREGISTER" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2unregister.push(node);
      }
    });
    updateQueueMain(
      nodes2unregister.map((node) => {
        return { node, action: "UNREGISTER" };
      })
    );
  }

  /** Unregister node in the queue and trigger the next one. */
  async function unregisterNodeQueued(node: RosNode | undefined): Promise<void> {
    if (node) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("UNREGISTER", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        // store result for message
        const resultUnregisterNode = await provider.unregisterNode(node.id);
        if (!resultUnregisterNode.result) {
          addStatusQueueMain("UNREGISTER", node.name, false, resultUnregisterNode.message);
        } else {
          addStatusQueueMain("UNREGISTER", node.name, true, "unregistered");
        }
      }
    }
    return Promise.resolve();
  }

  /**
   * Start dynamic reconfigure GUI for selected nodes
   */
  function startDynamicReconfigure(service: string, masteruri: string): void {
    if (
      queueItemsQueueMain &&
      queueItemsQueueMain.find((elem) => {
        return elem.action === "DYNAMIC_RECONFIGURE" && elem.service === service && elem.masteruri === masteruri;
      })
    ) {
      // TODO: skippedNodes.set(service, 'already in queue');
    } else {
      updateQueueMain([{ action: "DYNAMIC_RECONFIGURE", service: service, masteruri: masteruri }]);
    }
  }

  /** start dynamic reconfigure in the queue and trigger the next one. */
  async function dynamicReconfigureQueued(service: string | undefined, masteruri: string | undefined): Promise<void> {
    if (service) {
      // store result for message
      const result = await rosCtx.startDynamicReconfigureClient(service, masteruri || "");
      if (!result.result) {
        addStatusQueueMain("DYNAMIC_RECONFIGURE", service, false, result.message);
      } else {
        addStatusQueueMain("DYNAMIC_RECONFIGURE", service, true, `dynamic reconfigure started`);
      }
    }
    return Promise.resolve();
  }

  /**
   * Remove log files from ROS nodes using [rosclean purge] for a given provider
   */
  async function clearProviderLogs(providers: string[]): Promise<void> {
    // purge logs from host
    if (providers) {
      Promise.all(
        providers.map(async (providerId: string) => {
          const provider = rosCtx.getProviderById(providerId);
          if (!provider || !provider.isAvailable()) return;

          const result: Result = await provider.rosCleanPurge();

          if (result.result && result.message.indexOf("Purging ROS node logs.") === -1) {
            // should not happen, probably error
            logCtx.error("Could not delete logs", result.message);
          } else {
            logCtx.success("Logs removed successfully", "");
          }
        })
      ).catch((error) => {
        logCtx.error(`Could not clear logs for providers`, error);
      });
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
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "CLEAR_LOG" && elem.node?.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2clear.push(node);
      }
    });
    updateQueueMain(
      nodes2clear.map((node) => {
        return { node, action: "CLEAR_LOG" };
      })
    );
  }

  /** Delete log file for node in the queue and trigger the next one. */
  async function clearNodeLogQueued(node: RosNode | undefined): Promise<void> {
    if (node !== undefined) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("CLEAR_LOG", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        const result: TResultClearPath[] = await provider.clearLogPaths([node.name]);
        if (result.length === 0) {
          logCtx.error(`Could not delete log files for node: [${node.name}]`, JSON.stringify(result));
          addStatusQueueMain("CLEAR_LOG", node.name, false, JSON.stringify(result));
        } else if (!result[0].result) {
          logCtx.error(`Could not delete log files for node: [${node.name}]`, result[0].message);
          addStatusQueueMain("CLEAR_LOG", node.name, false, result[0].message);
        } else {
          addStatusQueueMain("CLEAR_LOG", node.name, true, `Removed log files for node: [${node.name}]`);
        }
      }
    }
    return Promise.resolve();
  }

  function refreshAllProvider(forceRefresh: boolean): void {
    rosCtx.providersConnected.forEach((p) => {
      p.updateRosNodes({}, forceRefresh);
      p.updateTimeDiff();
    });
  }

  // Register useEffect Callbacks ----------------------------------------------------------------------------------

  useEffect(() => {
    refreshAllProvider(false);
  }, [rosCtx.providersConnected]);

  useEffect(() => {
    // apply filter to nodes if search text was changed by user or nodes are updated by provider
    onSearch(filterText);
  }, [filterText, providerNodes]);

  useEffect(() => {
    // remove provider from our list if provider was removed in rosCtx
    setProviderNodes((prev) => [
      ...prev.filter((item) => rosCtx.providers.filter((prov) => prov.id === item.providerId).length > 0),
    ]);
  }, [rosCtx.providers]);

  useEffect(() => {
    if (nodesToStart) {
      // put all selected nodes to the start queue
      updateQueueMain(
        nodesToStart.map((node) => {
          return { node, action: "START" };
        })
      );
      setNodesToStart(undefined);
    }
  }, [nodesToStart]);

  async function performQueueMain(index: number): Promise<void> {
    if (index < 0) return Promise.resolve();
    // get current item from queue.
    // The index will be increased after the task was executed (by the callback).
    const queueItem = getQueueMain();
    if (queueItem) {
      if (queueItem.action === "START") {
        await startNodeQueued(queueItem.node);
      } else if (queueItem.action === "STOP") {
        await stopNodeQueued(queueItem.node);
      } else if (queueItem.action === "UNREGISTER") {
        await unregisterNodeQueued(queueItem.node);
      } else if (queueItem.action === "KILL") {
        await killNodeQueued(queueItem.node);
      } else if (queueItem.action === "CLEAR_LOG") {
        await clearNodeLogQueued(queueItem.node);
      } else if (queueItem.action === "DYNAMIC_RECONFIGURE") {
        await dynamicReconfigureQueued(queueItem.service, queueItem.masteruri);
      } else {
        console.log(`unknown item in the queue: ${JSON.stringify(queueItem)}`);
      }
    } else {
      // queue is finished, print failed results
      ["STOP", "START", "KILL", "UNREGISTER", "CLEAR_LOG", "DYNAMIC_RECONFIGURE"].forEach((action) => {
        const failed = failedQueueMain(action);
        if (failed.length > 0) {
          const infoDict = {};
          failed.forEach((item) => {
            infoDict[item.itemName] = item.message;
          });
          logCtx.warn(`Failed to ${action.toLocaleLowerCase()} ${failed.length} nodes`, JSON.stringify(infoDict), true);
        }
      });
      // queue is finished, print success results
      [
        ["STOP", "stopped"],
        ["START", "started"],
        ["KILL", "killed"],
        ["UNREGISTER", "unregistered"],
        ["CLEAR_LOG", "logs cleared"],
        ["DYNAMIC_RECONFIGURE", "dynamic reconfigure started"],
      ].forEach((action) => {
        const success = successQueueMain(action[0]);
        if (success.length > 0) {
          const infoDict = {};
          success.forEach((item) => {
            infoDict[item.itemName] = item.message;
          });
          logCtx.success(`${success.length} nodes ${action[1]}`, JSON.stringify(infoDict), true);
        }
      });
      // clear queue and results
      clearQueueMain();
      return Promise.resolve();
    }
  }

  // update queue
  useEffect(() => {
    performQueueMain(indexQueueMain);
  }, [indexQueueMain]);

  function showRemoteOnAllProvider(state: boolean): void {
    rosCtx.providersConnected.forEach((p) => {
      p.showRemoteNodes = state;
    });
  }

  useEffect(() => {
    showRemoteOnAllProvider(showRemoteNodes);
    refreshAllProvider(false);
  }, [showRemoteNodes]);

  const createButtonBox = useMemo(() => {
    const selectedNodes = getSelectedNodes();
    return (
      <ButtonGroup orientation="vertical" aria-label="ros node control group">
        <Tooltip
          title="Start"
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Start"
              onClick={() => {
                startSelectedNodes();
              }}
              disabled={selectedNodes.length === 0}
            >
              <PlayArrowIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title="Stop; Shift: kill; Ctrl+Shift: Unregister ROS1 nodes"
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
          title="Restart"
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Restart"
              onClick={() => {
                restartSelectedNodes();
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
                  disabled={selectedNodes.filter((node) => node.masteruri?.length > 0).length === 0}
                >
                  <DeleteForeverIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Divider />
          </Stack>
        )}
        <Tooltip
          title="Edit (shift+click for alternative open location)"
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
                  getSelectedNodes().forEach((node) => {
                    countDri += node.dynamicReconfigureServices.length;
                    driItems.push(node);
                  });
                  if (countDri > 2) {
                    setDynamicReconfigureItems(driItems);
                  } else {
                    driItems.forEach((node) => {
                      node.dynamicReconfigureServices.forEach((dri) => {
                        startDynamicReconfigure(dri, node.masteruri);
                      });
                    });
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
        <Tooltip title="Screen (shift+click for alternative open location)" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Screen"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                if (navCtx.selectedProviders?.length > 0) {
                  const screens: TMenuOptionsScreen[] = []; // {node : string, screen: string, callback: () => void, external: boolean}
                  navCtx.selectedProviders?.forEach((providerId) => {
                    const prov = rosCtx.getProviderById(providerId);
                    prov?.screens.forEach((screenMap) => {
                      screenMap.screens?.forEach((screen) => {
                        screens.push({
                          nodeName: screenMap.name,
                          providerId: prov?.id,
                          screen: screen,
                          external: event.nativeEvent.shiftKey,
                        });
                      });
                    });
                    setNodeScreens(screens);
                  });
                } else {
                  const screens: TMenuOptionsScreen[] = []; // {node : string, screen: string, callback: () => void, external: boolean}
                  getSelectedNodes().forEach((node) => {
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
                  });
                  if (screens.length >= 3) {
                    setNodeScreens(screens);
                  } else {
                    screens.forEach((item) => {
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
                    });
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
        <Tooltip title="Log (shift+click for alternative open location)" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Log"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                const logs: TMenuOptionsNode[] = []; // {node : string, callback: () => void}
                getSelectedNodes().forEach((node) => {
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
                });
                if (logs.length >= 3) {
                  setNodeLogs(logs);
                } else {
                  logs.forEach((item) => {
                    item.callback();
                  });
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
                getSelectedNodes().forEach((node) => {
                  loggers.push({
                    node: node,
                    callback: () => {
                      createLoggerPanel(node);
                    },
                  });
                });
                if (loggers.length >= 3) {
                  setNodeLoggers(loggers);
                } else {
                  loggers.forEach((item) => {
                    item.callback();
                  });
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
                  navCtx.selectedProviders.forEach((providerId) => {
                    createSingleTerminalPanel(
                      CmdType.TERMINAL,
                      providerId,
                      "",
                      "",
                      event.nativeEvent.shiftKey,
                      event.nativeEvent.ctrlKey
                    );
                  });
                }}
              >
                <TerminalIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
        )}
      </ButtonGroup>
    );
  }, [navCtx.selectedNodes, navCtx.selectedProviders]);

  return (
    <Box
      // ref={panelRef}
      width="100%"
      height="100%"
      overflow="auto"
      sx={{ backgroundColor: backgroundColor }}
    >
      <Stack spacing={0.5} direction="column" width="100%" height="100%">
        {indexQueueMain < 0 && (
          <Stack direction="row" spacing={0.5} alignItems="center">
            <Tooltip
              title="Each host shows all nodes visible to it"
              placement="left"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <ToggleButton
                size="small"
                value="showRemoteNodes"
                selected={showRemoteNodes}
                onChange={() => setShowRemoteNodes(!showRemoteNodes)}
              >
                <LocalPlayOutlinedIcon sx={{ fontSize: "inherit" }} />
              </ToggleButton>
            </Tooltip>
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
            <SearchBar
              key={"search-bar-host"}
              onSearch={(value) => {
                setFilterText(value);
              }}
              placeholder="Search nodes (OR: <space>, AND: +, NOT: !)"
              defaultValue={filterText}
              fullWidth
            />
          </Stack>
        )}
        {indexQueueMain >= 0 && (
          <Paper elevation={2}>
            <Stack alignItems="center" justifyItems="center" direction="row" spacing={0.5} sx={{ marginRight: 2 }}>
              <LinearProgress sx={{ width: "100%" }} variant="determinate" value={progressQueueMain} />
              <FormLabel>
                {indexQueueMain}/{queueItemsQueueMain.length}
              </FormLabel>
              <IconButton
                onClick={() => {
                  clearQueueMain();
                }}
                size="small"
              >
                <StopIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Stack>
          </Paper>
        )}
        <Stack direction="row" height="100%" overflow="auto">
          <Box height="100%">
            {/* <Paper elevation={2} sx={{ border: 0 }} height="100%"> */}
            {createButtonBox}
            {/* </Paper> */}
          </Box>
          <Box width="100%" height="100%" overflow="auto">
            {(!rosCtx.providersConnected || rosCtx.providersConnected.length === 0) && (
              <Alert severity="info">
                <AlertTitle>No providers available</AlertTitle>
                Please connect to a ROS provider
              </Alert>
            )}
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
          </Box>
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
            items.forEach((item) => {
              const nodeWithOpt = nodeParams.find((nodeItem) => `${nodeItem.name}` === item);
              if (nodeWithOpt && nodeWithOpt.callback) {
                nodeWithOpt.callback();
              }
            });
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
            items.forEach((item) => {
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
            });
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
            items.forEach((item) => {
              const nodeWithOpt = nodeLogs.find((logItem) => `${logItem.node.name}` === item);
              if (nodeWithOpt && nodeWithOpt.callback) {
                nodeWithOpt.callback();
              }
            });
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
            items.forEach((item) => {
              const nodeWithOpt = nodeLogs.find((nodeItem) => `${nodeItem.node.name}` === item);
              if (nodeWithOpt && nodeWithOpt.callback) {
                nodeWithOpt.callback();
              }
            });
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
            items.forEach((item) => {
              item.list.forEach((launch) => {
                const node = nodeMultiLaunches.find((n) => n.name.includes(item.title));
                if (node) {
                  useLaunchFiles[node.name] = launch;
                }
              });
            });
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
            items.forEach((item) => {
              item.list.forEach((launch) => {
                const launchInfo = editNodeWithMultipleLaunchInfos.node.launchInfo.get(launch);
                navCtx.openEditor(
                  editNodeWithMultipleLaunchInfos.node.providerId,
                  launch,
                  launchInfo?.file_name || "",
                  launchInfo?.file_range as TFileRange,
                  launchInfo?.launch_context_arg as TLaunchArg[],
                  editNodeWithMultipleLaunchInfos.external
                );
              });
            });
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
            items.forEach((item) => {
              const node = dynamicReconfigureItems.find((node) => node.name === item.title);
              item.list.forEach((dri) => {
                startDynamicReconfigure(dri, node?.masteruri || "");
              });
            });
            setDynamicReconfigureItems([]);
          }}
          onCancelCallback={() => {
            setDynamicReconfigureItems([]);
          }}
        />
      )}
    </Box>
  );
}
