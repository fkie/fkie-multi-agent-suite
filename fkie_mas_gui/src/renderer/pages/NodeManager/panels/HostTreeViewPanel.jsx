import { useCallback, useContext, useEffect, useMemo, useState } from "react";
// mui imports
import AddToQueueIcon from "@mui/icons-material/AddToQueue";
import BorderColorIcon from "@mui/icons-material/BorderColor";
import CancelPresentationIcon from "@mui/icons-material/CancelPresentation";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import DeleteSweepIcon from "@mui/icons-material/DeleteSweep";
import DvrIcon from "@mui/icons-material/Dvr";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
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
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { ConfirmModal, HostTreeView, MapSelectionModal, SearchBar } from "../../../components";
import { LoggingContext } from "../../../context/LoggingContext";
import { NavigationContext } from "../../../context/NavigationContext";
import { RosContext } from "../../../context/RosContext";
import { SSHContext } from "../../../context/SSHContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useQueue from "../../../hooks/useQueue";
import { RosNode, RosNodeStatus, getBaseName } from "../../../models";
import { CmdType } from "../../../providers";
import { EVENT_PROVIDER_ROS_NODES } from "../../../providers/eventTypes";
import {
  EVENT_EDITOR_SELECT_RANGE,
  EVENT_OPEN_COMPONENT,
  eventEditorSelectRange,
  eventOpenComponent,
} from "../../../utils/events";
import { findIn } from "../../../utils/index";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import FileEditorPanel from "./FileEditorPanel";
import NodeLoggerPanel from "./NodeLoggerPanel";
import ParameterPanel from "./ParameterPanel";
import SingleTerminalPanel from "./SingleTerminalPanel";

// TODO: Add settings for this
// const IGNORED_NODES = ['rostopic_'];
const IGNORED_NODES = [];

function HostTreeViewPanel() {
  // context objects
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const SSHCtx = useContext(SSHContext);
  const navCtx = useContext(NavigationContext);

  // state variables
  const [showRemoteNodes, setShowRemoteNodes] = useState(settingsCtx.get("showRemoteNodes"));
  const [filterText, setFilterText] = useState("");
  // providerNodes: list of {providerId: string, nodes: RosNode[]}
  const [providerNodes, setProviderNodes] = useState([]);
  // updated and filtered node tree
  // providerNodeTree: list of {providerId: string, nodeTree: object}
  const [providerNodeTree, setProviderNodeTree] = useState([]);
  const [rosCleanPurge, setRosCleanPurge] = useState(false);
  const [nodeScreens, setNodeScreens] = useState(null);
  const [nodeMultiLaunches, setNodeMultiLaunches] = useState(null);
  const [nodesAwaitModal, setNodesAwaitModal] = useState(null);
  const [nodesToStart, setNodesToStart] = useState(null);
  const [progressQueueMain, setProgressQueueMain] = useState(null);
  const {
    update: updateQueueMain,
    clear: clearQueueMain,
    next: nextQueueMain,
    get: getQueueMain,
    queueItems: queueItemsQueueMain,
    size: sizeQueueMain,
    index: indexQueueMain,
    success: successQueueMain,
    failed: failedQueueMain,
    addStatus: addStatusQueueMain,
  } = useQueue(setProgressQueueMain);
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  /**
   * Get list of nodes from a list of node.idGlobal
   */
  const getNodesFromIds = useCallback(
    (itemIds) => {
      const nodeList = [];
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
  }, [getNodesFromIds, navCtx.selectedNodes]);

  const nameWithoutNamespace = (node) => {
    return node.namespace && node.namespace !== "/" ? node.name.replace(node.namespace, "") : node.name;
  };

  // debounced search callback
  // search in the origin node list and create a new tree
  const onSearch = useDebounceCallback((searchTerm) => {
    const newProvidersTree = [];
    providerNodes.forEach((item) => {
      const { providerId, nodes } = item;
      // generate node tree structure based on node list
      // reference: https://stackoverflow.com/questions/57344694/create-a-tree-from-a-list-of-strings-containing-paths-of-files-javascript
      const nodeTree = [];
      const level = { nodeTree };
      // ...and keep a list of the tree nodes
      const nodeTreeList = [];
      const nodeItemMap = new Map();
      nodes.forEach((node) => {
        nodeItemMap.set(node.idGlobal, node);
        // filter nodes by user text
        if (searchTerm.length > 0) {
          const isMatch = findIn(searchTerm, [node.name, node.group, node.providerName]);
          if (!isMatch) return;
        }

        const nodePath = `${node.group}/${node.idGlobal}`;
        nodePath.split("/").reduce((r, name, i, a) => {
          if (!r[name]) {
            r[name] = { nodeTree: [] };

            // Meaning of [name]:
            //    In case of a node: corresponds to the uniqueId
            //    In case of group: corresponds to group name
            if (nodeItemMap.has(name)) {
              // create a node
              const treePath = `${a.slice(0, -1).join("/")}#${nameWithoutNamespace(node)}`;
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node,
              });
              nodeTreeList.push(`${providerId}#${treePath}`);
            } else {
              // create a (sub)group

              const treePath = name ? a.slice(0, i + 1).join("/") : "";
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node: null,
              });
              nodeTreeList.push(treePath ? `${providerId}#${treePath}` : providerId);
            }
          }
          return r[name];
        }, level);
      });
      const p = rosCtx.getProviderById(providerId);
      // providerNodeTree[providerId] = nodeTree[0];
      newProvidersTree.push({
        providerId,
        providerName: p?.name(),
        nodeTree: nodeTree[0],
      });
    });
    setProviderNodeTree(newProvidersTree);
  }, 300);

  useCustomEventListener(
    EVENT_PROVIDER_ROS_NODES,
    (data) => {
      const { provider, nodes } = data;
      // create new node list with updated group labels
      const newNodes = [];
      nodes.forEach((node) => {
        node.group = "";
        // If the node is flagged as system node, set a group with the highest priority
        if (node.system_node) {
          node.group += settingsCtx.get("namespaceSystemNodes");
        }
        // If node has namespace, add it to the group with the second highest priority
        if (node.namespace && node.namespace !== "/") {
          node.group += `${node.namespace}`;
        }
        // group using parameters
        settingsCtx.get("groupParameters")?.forEach((parameter) => {
          if (node.parameters && node.parameters.has(parameter)) {
            const parameterValue = `${node.parameters.get(parameter)}`;
            // TODO: Add parameter name to group?
            // const parameterName = parameter.replaceAll('/', ' ');
            // node.group += `/${parameterName}: ${parameterValue}`;

            node.group += `/${parameterValue}`;
          }
        });
        newNodes.push(node);
      });
      // update state
      // TODO: should we remove closed/lost provider infos
      setProviderNodes((oldValues) => [
        ...oldValues.filter((item) => {
          return item.providerId !== provider.id;
        }),
        {
          providerId: provider.id,
          nodes: newNodes,
        },
      ]);
      // setSelectedTreeItems((prevValues) => [...prevValues]);
    },
    [setProviderNodes]
  );

  // Register Callbacks ----------------------------------------------------------------------------------

  /**
   * Callback when nodes on the tree are selected by the user
   */
  const handleNodesSelect = useCallback(
    (itemIds) => {
      const selectedNoes = [];
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
  const handleProviderSelect = useCallback(
    (providerIds) => {
      // set the selected nodes
      const selectedProvidersLocal = [];
      // update setSelectedNodes based on trees selectedItems
      providerIds.forEach((id) => {
        // search selected host
        if (rosCtx.getProviderById(id)) {
          selectedProvidersLocal.push(id);
        }
      });
      navCtx.setSelectedProviders(selectedProvidersLocal);
    },
    [navCtx, rosCtx]
  );

  /**
   * Create and open a new panel with a [NodeLoggerPanel] for a given node
   */
  const createLoggerPanel = useCallback(async (node) => {
    const id = `node-logger-${node.idGlobal}`;
    const title = node.name;
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        id,
        title,
        <NodeLoggerPanel node={node} />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(false, "node-logger", {})
      )
    );
  }, []);

  /**
   * Start nodes from a list of itemIds
   */
  const createLoggerPanelFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      if (nodeList.length > 0) {
        createLoggerPanel(nodeList[0]);
      }
    },
    [getNodesFromIds, createLoggerPanel]
  );

  /**
   * Create and open a new panel with a [SingleTerminalPanel] for a given node
   */
  const createSingleTerminalPanel = useCallback(
    async (type, node, screen, external = false) => {
      if (external && window.CommandExecutor) {
        // create a terminal command
        const provider = rosCtx.getProviderById(node.providerId);
        const terminalCmd = await provider.cmdForType(type, node.name, "", screen, "");
        // open screen in a new terminal
        try {
          const result = await window.CommandExecutor?.execTerminal(
            provider.isLocalHost ? null : SSHCtx.getCredentialHost(provider.host()),
            `"${type.toLocaleUpperCase()} ${node.name}@${provider.host()}"`,
            terminalCmd.cmd
          );
          if (!result.result) {
            logCtx.error(`Can't open external terminal on ${provider.host()}`, result.message, true);
          }
        } catch (error) {
          logCtx.error(`Can't open external terminal on ${provider.host()}`, error, true);
        }
      } else {
        const id = `${type}-${screen}-${node.name}@${node.providerName}`;
        const title = node.name ? `${node.name}@${node.providerName}` : node.providerName;
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            id,
            title,
            <SingleTerminalPanel id={id} type={type} providerId={node.providerId} node={node} screen={screen} />,
            true,
            LAYOUT_TAB_SETS.BORDER_BOTTOM,
            new LayoutTabConfig(true, type, {
              type,
              providerId: node.providerId,
              nodeName: node.name,
              screen,
            })
          )
        );
      }
    },
    [rosCtx, SSHCtx, logCtx]
  );

  /**
   * Create and open a new panel with a [createFileEditorPanel] for selected nodes
   */
  const createFileEditorPanel = (nodes, external) => {
    const openIds = [];
    nodes.forEach(async (node) => {
      if (!node.launchInfo) {
        logCtx.error(`Could not find launch file for node: [${node.name}]`, `Node Info: ${JSON.stringify(node)}`);
        return;
      }
      const rootLaunch = [...node.launchPaths][0];
      if (node.launchPaths.size > 1) {
        // TODO: select
      }
      const provider = rosCtx.getProviderById(node.providerId);
      const id = `editor-${provider.connection.host}-${provider.connection.port}-${rootLaunch}`
      if (external && provider && window.electronAPI) {
        // open in new window
        window.electronAPI.openEditor(
          id,
          provider.connection.host,
          provider.connection.port,
          rootLaunch,
          node.launchInfo.file_name,
          node.launchInfo.file_range
        );
        return;
      }
      // open in a tab
      const launchName = getBaseName(rootLaunch);
      const hasExtEditor = await window.electronAPI?.hasEditor(id);
      if (hasExtEditor) {
        // inform external window about new selected range
        window.electronAPI?.emitEditorFileRange(
          id,
          node.launchInfo.file_name,
          node.launchInfo.file_range
        );
      } else if (!openIds.includes(id)) {
        // inform already open tab about new node selection
        emitCustomEvent(
          EVENT_EDITOR_SELECT_RANGE,
          eventEditorSelectRange(id, node.launchInfo.file_name, node.launchInfo.file_range)
        );
        // open new editor in a tab. Checks for existing tabs are performed in NodeManager
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            id,
            launchName,
            <FileEditorPanel
              tabId={id}
              providerId={node.providerId}
              fileRange={node.launchInfo.file_range}
              currentFilePath={node.launchInfo.file_name}
              rootFilePath={rootLaunch}
            />,
            true,
            LAYOUT_TAB_SETS[settingsCtx.get("editorOpenLocation")],
            new LayoutTabConfig(true, "editor", null, {
              id: id,
              host: provider?.connection.host,
              port: provider?.connection.port,
              rootLaunch: rootLaunch,
              path: node.launchInfo.file_name,
              fileRange: node.launchInfo.file_range,
            })
          )
        );
        openIds.push(id);
      }
    });
  };

  /**
   * Create and open a new panel with a [ParameterPanel] for selected nodes
   */
  const createParameterPanel = useCallback((nodes, providers) => {
    nodes?.forEach((node) => {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `parameter-node-${node.idGlobal}`,
          `${node.name}`,
          <ParameterPanel nodes={[node]} providers={null} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, "parameter")
        )
      );
    });
    providers?.forEach((provider) => {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `parameter-provider-${provider}`,
          `${provider}`,
          <ParameterPanel nodes={null} providers={[provider]} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, "parameter")
        )
      );
    });
  }, []);

  const startNodeQueued = async (node) => {
    // let node = getQueueMain();
    if (node !== null) {
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
  };

  const updateWithAssociations = useCallback(
    (nodes, depth = 0) => {
      if (depth > 10) return [nodes];
      const newNodeList = [];
      nodes.forEach((node) => {
        if (node.associations.length > 0) {
          const provider = rosCtx.getProviderById(node.providerId);
          if (provider) {
            node.associations.forEach((asNodeName) => {
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
        if (!newNodeList.find((n) => n.name === node.name)) {
          newNodeList.push(node);
        }
      });
      return newNodeList;
    },
    [rosCtx]
  );

  const startNodesWithLaunchCheck = useCallback(
    (nodes, ignoreRunState = false) => {
      const withMultiLaunch = [];
      const node2Start = [];
      const withNoLaunch = [];
      const skippedNodes = new Map();
      // check nodes for associations and extend start node list
      const nodeList = updateWithAssociations(nodes);
      nodeList.forEach((node) => {
        // ignore running and nodes already in the queue
        if (!ignoreRunState && node.status === RosNodeStatus.RUNNING) {
          skippedNodes.set(node.name, "already running");
        } else if (
          queueItemsQueueMain &&
          queueItemsQueueMain.find((elem) => {
            return elem.action === "START" && elem.node.name === node.name;
          })
        ) {
          skippedNodes.set(node.name, "already in the start queue");
        } else if (node.launchPaths.size === 1) {
          node2Start.push(node);
        } else if (node.launchPaths.size > 1) {
          node2Start.push(node);
          // Multiple launch files available
          withMultiLaunch.push(node);
        } else if (!node.system_node) {
          // no launch files
          withNoLaunch.push(node.name);
        }
      });
      if (skippedNodes.size > 0) {
        logCtx.debug(`Skipped ${skippedNodes.size} nodes`, Object.fromEntries(skippedNodes));
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
    },
    [queueItemsQueueMain, logCtx, updateWithAssociations]
  );

  /**
   * Start nodes in the selected list
   */
  const startSelectedNodes = useCallback(() => {
    startNodesWithLaunchCheck(getSelectedNodes());
  }, [getSelectedNodes, startNodesWithLaunchCheck]);

  /**
   * Start nodes from a list of itemIds
   */
  const startNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      startNodesWithLaunchCheck(nodeList);
    },
    [getNodesFromIds, startNodesWithLaunchCheck]
  );

  /** Stop node from queue and trigger the next one. */
  const stopNodeQueued = async (node) => {
    if (node !== null) {
      logCtx.debug(`stop: ${node.name}`, "");
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("STOP", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        // store result for message
        const resultStopNode = await provider.stopNode(node.id);
        if (!resultStopNode.result) {
          addStatusQueueMain("STOP", node.name, false, resultStopNode.message);
        } else {
          addStatusQueueMain("STOP", node.name, true, "stopped");
        }
      }
    }
  };

  /**
   * Stop nodes given in the arguments
   */
  const stopNodes = useCallback(
    (nodes, onlyWithLaunch, restart) => {
      const nodes2stop = [];
      const skippedNodes = new Map();
      const nodeList = updateWithAssociations(nodes);
      nodeList.forEach((node) => {
        // we stop system nodes only when they are individually selected
        if (node.system_node && nodeList.length > 1) {
          // skip system nodes
          skippedNodes.set(node.name, "system node");
        } else if (onlyWithLaunch && node.launchPaths.length === 0) {
          skippedNodes.set(node.name, "stop only with launch files");
        } else if (
          queueItemsQueueMain &&
          queueItemsQueueMain.find((elem) => {
            return elem.action === "STOP" && elem.node.name === node.name;
          })
        ) {
          skippedNodes.set(node.name, "already in queue");
        } else if (node.status === RosNodeStatus.RUNNING) {
          // const isRunning = node.status === RosNodeStatus.RUNNING;
          // const isRunning = true;
          // if (!skip) {
          nodes2stop.push(node);
        } else {
          skippedNodes.set(node.name, "not running");
        }
      });
      if (skippedNodes.size > 0) {
        logCtx.debug(`Skipped ${skippedNodes.size} nodes`, Object.fromEntries(skippedNodes));
      }
      updateQueueMain(
        nodes2stop.map((node) => {
          return { node, action: "STOP" };
        })
      );
      if (restart) {
        startNodesWithLaunchCheck(nodeList, true);
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [queueItemsQueueMain, startNodesWithLaunchCheck]
  );

  /**
   * Stop nodes in the selected list
   */
  const stopSelectedNodes = useCallback(() => {
    stopNodes(getSelectedNodes());
  }, [getSelectedNodes, stopNodes]);

  /**
   * Stop nodes from a list of itemIds
   */
  const stopNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      stopNodes(nodeList);
    },
    [getNodesFromIds, stopNodes]
  );

  /**
   * Restart nodes given in the arguments
   */
  const restartNodes = useCallback(
    (nodeList, onlyWithLaunch) => {
      stopNodes(nodeList, onlyWithLaunch, true); // => true, for restart
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [stopNodes]
  );

  /**
   * Restart nodes in the selected list
   */
  const restartSelectedNodes = useCallback(() => {
    restartNodes(getSelectedNodes());
  }, [getSelectedNodes, restartNodes]);

  /**
   * Restart nodes from a list of itemIds
   */
  const restartNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      restartNodes(nodeList);
    },
    [getNodesFromIds, restartNodes]
  );

  /**
   * Kill selected nodes using provider
   */
  const killSelectedNodes = useCallback(() => {
    const nodes2kill = [];
    getSelectedNodes().map(async (node) => {
      // we kill system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      if (
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "KILL" && elem.node.name === node.name;
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
  }, [getSelectedNodes, navCtx.selectedNodes.length, queueItemsQueueMain, updateQueueMain]);

  /** Kill node in the queue and trigger the next one. */
  const killNodeQueued = async (node) => {
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("KILL", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        // store result for message
        const resultKillNode = await provider.screenKillNode(node.id);
        if (!resultKillNode.result) {
          addStatusQueueMain("KILL", node.name, false, resultKillNode.message);
        } else {
          addStatusQueueMain("UNREGISTER", node.name, true, "killed");
        }
      }
    }
  };

  /**
   * Unregister selected nodes using provider
   */
  const unregisterSelectedNodes = useCallback(() => {
    const nodes2unregister = [];
    getSelectedNodes().map(async (node) => {
      // we unregister system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      // not supported for ROS2
      if (!node.masteruri) return;
      if (
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "UNREGISTER" && elem.node.name === node.name;
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
  }, [getSelectedNodes, navCtx.selectedNodes.length, queueItemsQueueMain, updateQueueMain]);

  /** Unregister node in the queue and trigger the next one. */
  const unregisterNodeQueued = async (node) => {
    if (node !== null) {
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
  };

  /**
   * Check given node for dynamic reconfigure capabilities
   */
  const isDynamicReconfigureNode = (node) => {
    if (!node.masteruri) return false;
    if (
      Array.from(node.services.keys()).filter((service) => {
        return service.endsWith("/set_parameters");
      }).length === 0
    ) {
      // no services for dynamic reconfigure found
      return false;
    }
    return true;
  };

  /**
   * Start dynamic reconfigure GUI for selected nodes
   */
  const startDynamicReconfigure = useCallback(() => {
    const nodes2configure = [];
    getSelectedNodes().map(async (node) => {
      if (!isDynamicReconfigureNode(node)) {
        return;
      }
      if (
        queueItemsQueueMain &&
        queueItemsQueueMain.find((elem) => {
          return elem.action === "DYNAMIC_RECONFIGURE" && elem.node.name === node.name;
        })
      ) {
        // TODO: skippedNodes.set(node.name, 'already in queue');
      } else {
        nodes2configure.push(node);
      }
    });
    updateQueueMain(
      nodes2configure.map((node) => {
        return { node, action: "DYNAMIC_RECONFIGURE" };
      })
    );
  }, [getSelectedNodes, navCtx.selectedNodes.length, queueItemsQueueMain, updateQueueMain]);

  /** start dynamic reconfigure in the queue and trigger the next one. */
  const dynamicReconfigureQueued = async (node) => {
    if (node !== null) {
      // store result for message
      const result = await rosCtx.startDynamicReconfigureClient(node);
      if (!result.result) {
        addStatusQueueMain("DYNAMIC_RECONFIGURE", node.name, false, result.message);
      } else {
        addStatusQueueMain("DYNAMIC_RECONFIGURE", node.name, true, `dynamic reconfigure started`);
      }
    }
  };

  /**
   * Remove log files from ROS nodes using [rosclean purge] for a given provider
   */
  const clearProviderLogs = useCallback(
    async (providers) => {
      // purge logs from host
      if (providers) {
        Promise.all(
          providers.map(async (providerId) => {
            const provider = rosCtx.getProviderById(providerId);
            if (!provider || !provider.isAvailable()) return;

            const result = await provider.rosCleanPurge();

            if (result.length > 0 && result.indexOf("Purging ROS node logs.") === -1) {
              // should not happen, probably error
              logCtx.error("Could not delete logs", result);
            } else {
              logCtx.success("Logs removed successfully", "");
            }
          })
        ).catch((error) => {
          logCtx.error(`Could not clear logs for providers`, error);
        });
      }
    },
    [rosCtx, logCtx]
  );

  /**
   * Delete log file of given nodes.
   */
  const clearLogs = useCallback(
    (nodes) => {
      const nodes2clear = [];
      nodes.map(async (node) => {
        // we unregister system nodes only when they are individually selected
        if (node.system_node && nodes.length > 1) return;
        if (
          queueItemsQueueMain &&
          queueItemsQueueMain.find((elem) => {
            return elem.action === "CLEAR_LOG" && elem.node.name === node.name;
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
    },
    [queueItemsQueueMain, updateQueueMain]
  );

  /** Delete log file for node in the queue and trigger the next one. */
  const clearNodeLogQueued = async (node) => {
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain("CLEAR_LOG", node.name, false, `Provider ${node.providerName} not available`);
      } else {
        const result = await provider.clearLogPaths([node.name]);
        if (result.length === 0) {
          logCtx.error(`Could not delete log files for node: [${node.name}]`, result);
          addStatusQueueMain("CLEAR_LOG", node.name, false, result);
        } else if (!result[0].result) {
          logCtx.error(`Could not delete log files for node: [${node.name}]`, result[0].message);
          addStatusQueueMain("CLEAR_LOG", node.name, false, result[0].message);
        } else {
          addStatusQueueMain("CLEAR_LOG", node.name, true, `Removed log files for node: [${node.name}]`);
        }
      }
    }
  };

  const refreshAllProvider = useCallback(() => {
    rosCtx.providersConnected.forEach((p) => {
      if (p.isReady()) {
        p.updateRosNodes();
        p.updateTimeDiff();
      }
    });
  }, [rosCtx.providersConnected]);

  // Register useEffect Callbacks ----------------------------------------------------------------------------------

  useEffect(() => {
    refreshAllProvider();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.providersConnected]);

  useEffect(() => {
    // filter nodes by user text
    onSearch(filterText);
  }, [filterText, onSearch, providerNodes]);

  useEffect(() => {
    // filter nodes by user text
    let removed = false;
    const newNodeMap = [];
    providerNodes.forEach((item) => {
      if (rosCtx.providers.filter((prov) => prov.id === item.providerId).length > 0) {
        newNodeMap.push(item);
      } else {
        removed = true;
      }
    });
    if (removed) {
      setProviderNodes(newNodeMap);
    }
  }, [rosCtx.providers, providerNodes]);

  // useEffect(() => {
  //   // each time selected items are changed (user select, nodes or launch changes)
  //   // set the selected nodes
  //   navCtx.setSelectedNodes(selectedTreeItems);

  //   // clear selected provider
  //   navCtx.setSelectedProviders([]);
  //   const selectedProvidersLocal = [];
  //   // update setSelectedNodes based on trees selectedItems
  //   selectedTreeItems.forEach((item) => {
  //     // search selected host
  //     if (rosCtx.getProviderById(item)) {
  //       selectedProvidersLocal.push(item);
  //     }
  //   });
  //   navCtx.setSelectedProviders(selectedProvidersLocal);
  //   // eslint-disable-next-line react-hooks/exhaustive-deps
  // }, [selectedTreeItems, getNodesFromIds, rosCtx]);

  useEffect(() => {
    if (nodesToStart) {
      // put all selected nodes to the start queue
      updateQueueMain(
        nodesToStart.map((node) => {
          return { node, action: "START" };
        })
      );
      setNodesToStart(null);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [nodesToStart]);

  // update queue
  useEffect(() => {
    if (sizeQueueMain === 0) return;

    if (indexQueueMain < 0) {
      // default index, we should start queue
      nextQueueMain();
    } else {
      // get current item from queue.
      // The index will be increased after the task was executed (by the callback).
      const queueItem = getQueueMain();
      if (queueItem) {
        if (queueItem.action === "START") {
          startNodeQueued(queueItem.node);
        } else if (queueItem.action === "STOP") {
          stopNodeQueued(queueItem.node);
        } else if (queueItem.action === "UNREGISTER") {
          unregisterNodeQueued(queueItem.node);
        } else if (queueItem.action === "KILL") {
          killNodeQueued(queueItem.node);
        } else if (queueItem.action === "CLEAR_LOG") {
          clearNodeLogQueued(queueItem.node);
        } else if (queueItem.action === "DYNAMIC_RECONFIGURE") {
          dynamicReconfigureQueued(queueItem.node);
        }
      } else {
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
            logCtx.success(`${success.length} nodes ${action[1]}`, infoDict, true);
          }
        });
        // queue is finished, print failed results
        ["STOP", "START", "KILL", "UNREGISTER", "CLEAR_LOG", "DYNAMIC_RECONFIGURE"].forEach((action) => {
          const failed = failedQueueMain(action);
          if (failed.length > 0) {
            const infoDict = {};
            failed.forEach((item) => {
              infoDict[item.itemName] = item.message;
            });
            logCtx.warn(`Failed to ${action.toLocaleLowerCase()} ${failed.length} nodes`, infoDict, true);
          }
        });
        // clear queue and results
        clearQueueMain();
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [indexQueueMain, failedQueueMain, successQueueMain]);

  const showRemoteOnAllProvider = useCallback(
    (state) => {
      rosCtx.providersConnected.forEach((p) => {
        p.showRemoteNodes = state;
      });
    },
    [rosCtx.providersConnected]
  );

  useEffect(() => {
    showRemoteOnAllProvider(showRemoteNodes);
    refreshAllProvider();
    // eslint-disable-next-line react-hooks/exhaustive-deps
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
          title="Stop"
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Stop"
              onClick={() => {
                stopSelectedNodes();
              }}
              disabled={selectedNodes.length === 0}
            >
              <StopIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Divider />
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
        <Divider />
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
        <Tooltip title="Dynamic reconfigure for ROS1 nodes" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="dynamic reconfigure"
              onClick={() => {
                startDynamicReconfigure();
              }}
              disabled={
                selectedNodes.filter((node) => {
                  return isDynamicReconfigureNode(node);
                }).length === 0
              }
            >
              <SettingsSuggestIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>

        <Divider />
        <Tooltip
          title="Edit (new window with shift+click)"
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
              disabled={selectedNodes.filter((node) => node.launchPaths.size > 0).length === 0}
            >
              <BorderColorIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title="Parameters"
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Parameters"
              onClick={() => {
                if (navCtx.selectedProviders?.length > 0) {
                  createParameterPanel(null, navCtx.selectedProviders);
                } else {
                  createParameterPanel(getSelectedNodes(), null);
                }
              }}
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
            >
              <TuneIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Divider />
        <Tooltip title="Screen (external terminal with shift+click)" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Screen"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                if (navCtx.selectedProviders?.length > 0) {
                  navCtx.selectedProviders?.forEach((providerId) => {
                    const prov = rosCtx.getProviderById(providerId);
                    const emptyNode = new RosNode();
                    emptyNode.name = prov?.name();
                    emptyNode.providerId = providerId;
                    emptyNode.providerName = prov?.name();
                    emptyNode.screens = [];
                    prov?.screens.forEach((screen) => {
                      emptyNode.screens = [...emptyNode.screens, ...screen.screens];
                    });
                    const sl = {
                      node: emptyNode,
                      external: event.nativeEvent.shiftKey,
                    };
                    setNodeScreens((prevNodes) => (prevNodes ? [...prevNodes, sl] : [sl]));
                  });
                } else {
                  getSelectedNodes().forEach((node) => {
                    if (node.screens.length === 1) {
                      // 1 screen available
                      node.screens.forEach((screen) => {
                        createSingleTerminalPanel(CmdType.SCREEN, node, screen, event.nativeEvent.shiftKey);
                      });
                    } else if (node.screens.length > 1) {
                      // Multiple screens available
                      setNodeScreens((prevNodes) =>
                        prevNodes
                          ? [
                              ...prevNodes,
                              {
                                node,
                                external: event.nativeEvent.shiftKey,
                              },
                            ]
                          : [
                              {
                                node,
                                external: event.nativeEvent.shiftKey,
                              },
                            ]
                      );
                    } else {
                      // no screens, try to find by node name instead
                      createSingleTerminalPanel(CmdType.SCREEN, node, undefined, event.nativeEvent.shiftKey);
                    }
                  });
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
        <Tooltip title="Change log level" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Log Level"
              disabled={selectedNodes.length === 0}
              onClick={(event) => {
                getSelectedNodes().forEach((node) => {
                  createLoggerPanel(node);
                });
              }}
            >
              <SettingsInputCompositeOutlinedIcon fontSize="inherit" sx={{ rotate: "90deg" }} />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip title="Log (external terminal with shift+click)" placement="left" disableInteractive>
          <span>
            <IconButton
              size="medium"
              aria-label="Log"
              disabled={selectedNodes.length === 0 && navCtx.selectedProviders?.length === 0}
              onClick={(event) => {
                getSelectedNodes().forEach((node) => {
                  createSingleTerminalPanel(CmdType.LOG, node, undefined, event.nativeEvent.shiftKey);
                });
              }}
            >
              <WysiwygIcon fontSize="inherit" />
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
                  clearLogs(getSelectedNodes(), null);
                }
              }}
            >
              <DeleteSweepIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Divider />
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
                  const prov = rosCtx.getProviderById(providerId);
                  const emptyNode = new RosNode();
                  emptyNode.name = "";
                  emptyNode.providerId = providerId;
                  emptyNode.providerName = prov?.name();
                  createSingleTerminalPanel(CmdType.TERMINAL, emptyNode, "", event.nativeEvent.shiftKey);
                });
              }}
            >
              <TerminalIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
      </ButtonGroup>
    );
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [navCtx.selectedNodes, navCtx.selectedProviders]);

  return (
    <Box
      // ref={panelRef}
      width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get("backgroundColor")}
    >
      <Stack spacing={0.5} direction="column" width="100%" height="100%">
        {sizeQueueMain === 0 && (
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
                  refreshAllProvider();
                }}
              >
                <RefreshIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
            <SearchBar
              onSearch={(value) => {
                setFilterText(value);
              }}
              placeholder="Search nodes (<space> for OR, + for AND)"
              defaultValue={filterText}
              fullWidth
            />
          </Stack>
        )}
        {sizeQueueMain > 0 && (
          <Paper elevation={2}>
            <Stack alignItems="center" justifyItems="center" direction="row" spacing={0.5} sx={{ marginRight: 2 }}>
              <LinearProgress sx={{ width: "100%" }} variant="determinate" value={progressQueueMain} />
              <FormLabel>
                {indexQueueMain}/{sizeQueueMain}
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
              providerNodeTree={providerNodeTree}
              onNodeSelect={handleNodesSelect}
              onProviderSelect={handleProviderSelect}
              showLoggers={createLoggerPanelFromId}
              startNodes={startNodesFromId}
              stopNodes={stopNodesFromId}
              restartNodes={restartNodesFromId}
              createSingleTerminalPanel={createSingleTerminalPanel}
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

      {nodeScreens && (
        <MapSelectionModal
          list={nodeScreens.reduce((prev, item) => {
            prev.push({
              title: item.node.name,
              list: item.node.screens,
              external: item.external,
            });
            return prev;
          }, [])}
          onConfirmCallback={(items) => {
            items.forEach((item) => {
              item.list.forEach((screen) => {
                const nodeWithOpt = nodeScreens.find((nodeMultiple) => nodeMultiple.node.screens.includes(screen));
                createSingleTerminalPanel(CmdType.SCREEN, nodeWithOpt.node, screen, nodeWithOpt.external);
              });
            });
            setNodeScreens(null);
          }}
          onCancelCallback={() => {
            setNodeScreens(null);
          }}
        />
      )}
      {nodeMultiLaunches && (
        <MapSelectionModal
          list={nodeMultiLaunches.reduce((prev, node) => {
            prev.push({ title: node.name, list: [...node.launchPaths] });
            return prev;
          }, [])}
          useRadioGroup
          onConfirmCallback={(items) => {
            items.forEach((item) => {
              item.list.forEach((launch) => {
                const node = nodesAwaitModal.find((n) => n.name.includes(item.title));
                node.launchPath = launch;
              });
            });
            setNodesToStart(nodesAwaitModal);
            setNodeMultiLaunches(null);
            setNodesAwaitModal(null);
          }}
          onCancelCallback={() => {
            setNodeMultiLaunches(null);
            setNodesAwaitModal(null);
          }}
        />
      )}
    </Box>
  );
}

HostTreeViewPanel.propTypes = {};

export default HostTreeViewPanel;
