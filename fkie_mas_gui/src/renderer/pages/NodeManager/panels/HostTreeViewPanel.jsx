import { useCallback, useContext, useEffect, useState } from 'react';

// mui imports
import BorderColorIcon from '@mui/icons-material/BorderColor';
import CancelPresentationIcon from '@mui/icons-material/CancelPresentation';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import DeleteSweepIcon from '@mui/icons-material/DeleteSweep';
import DynamicFeedOutlinedIcon from '@mui/icons-material/DynamicFeedOutlined';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import RefreshIcon from '@mui/icons-material/Refresh';
import RestartAltIcon from '@mui/icons-material/RestartAlt';
import StopIcon from '@mui/icons-material/Stop';
import SubjectIcon from '@mui/icons-material/Subject';
import TerminalIcon from '@mui/icons-material/Terminal';
import TuneIcon from '@mui/icons-material/Tune';
import WysiwygIcon from '@mui/icons-material/Wysiwyg';
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
  Tooltip,
} from '@mui/material';
import { useDebounceCallback } from '@react-hook/debounce';
import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';
import {
  ConfirmModal,
  HostTreeView,
  MapSelectionModal,
  SearchBar,
} from '../../../components';
import { LoggingContext } from '../../../context/LoggingContext';
import { NavigationContext } from '../../../context/NavigationContext';
import { RosContext } from '../../../context/RosContext';
import { SSHContext } from '../../../context/SSHContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { RosNode, RosNodeStatus, getFileName } from '../../../models';
import { CmdType } from '../../../providers';
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from '../layout';

import useQueue from '../../../hooks/useQueue';
import { EVENT_PROVIDER_ROS_NODES } from '../../../providers/events';
import {
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
} from '../../../utils/events';
import FileEditorPanel from './FileEditorPanel';
import ParameterPanel from './ParameterPanel';
import SingleTerminalPanel from './SingleTerminalPanel';

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
  const [filterText, setFilterText] = useState('');
  // providerNodes: list of {providerId: string, nodes: RosNode[]}
  const [providerNodes, setProviderNodes] = useState([]);
  // updated and filtered node tree
  // providerNodeTree: list of {providerId: string, nodeTree: object}
  const [providerNodeTree, setProviderNodeTree] = useState([]);
  // launchContentList: list of {providerId: string, launches: LaunchContent[]}
  const [launchContentList, setLaunchContentList] = useState([]);
  // nodeMap: Map<string, RosNode>
  const [nodeMap, setNodeMap] = useState(new Map());
  // items selected by user in the tree
  const [selectedTreeItems, setSelectedTreeItems] = useState([]);
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
  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  /**
   * Get list of nodes from a list of TreeItem IDs
   */
  const getNodesFromIds = useCallback(
    (itemIds) => {
      const nodeList = [];
      itemIds.forEach((item) => {
        // Make sure we have a nodeMap object and the item is not the provider (root)
        if (nodeMap && item.indexOf('#') > -1) {
          // search individual nodes
          if (nodeMap.has(item)) {
            const node = nodeMap.get(item);
            if (!nodeList.some((n) => n.id === node.id)) {
              nodeList.push(node);
            }
          }
          // search group of nodes
          else {
            nodeMap.forEach((node, key) => {
              if (
                !nodeList.some((n) => n.id === node.id) &&
                key.startsWith(item)
              ) {
                nodeList.push(node);
              }
            });
          }
        }
      });

      return nodeList;
    },
    [nodeMap],
  );

  const nameWithoutNamespace = (node) => {
    return node.namespace && node.namespace !== '/'
      ? node.name.replace(node.namespace, '')
      : node.name;
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
        if (filterText.length > 0) {
          const nodeNameFilter = `${node.name} ${node.group}`;
          let nodeExcluded = false;

          if (filterText.indexOf(',') >= 0) {
            // User wrote multiple filters separated by commas
            nodeExcluded = false;
            filterText.split(',').forEach((t) => {
              const filterPart = t.trim();
              if (
                filterPart.length > 0 &&
                nodeNameFilter.indexOf(filterPart) === -1
              ) {
                nodeExcluded = true;
              }
            });
          } else if (nodeNameFilter.indexOf(filterText) === -1) {
            // User wrote single filter
            nodeExcluded = true;
          }

          if (nodeExcluded) return;
        }

        const nodePath = `${node.group}/${node.idGlobal}`;
        nodePath.split('/').reduce((r, name, i, a) => {
          if (!r[name]) {
            r[name] = { nodeTree: [] };

            // Meaning of [name]:
            //    In case of a node: corresponds to the uniqueId
            //    In case of group: corresponds to group name
            if (nodeItemMap.has(name)) {
              // create a node
              const treePath = `${a
                .slice(0, -1)
                .join('/')}#${nameWithoutNamespace(node)}`;
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node,
              });
              nodeTreeList.push(`${providerId}#${treePath}`);
            } else {
              // create a (sub)group

              const treePath = name ? a.slice(0, i + 1).join('/') : '';
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node: null,
              });
              nodeTreeList.push(
                treePath ? `${providerId}#${treePath}` : providerId,
              );
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
        node.group = '';
        // If the node is flagged as system node, set a group with the highest priority
        if (node.system_node) {
          node.group += settingsCtx.get('namespaceSystemNodes');
        }
        // If node has namespace, add it to the group with the second highest priority
        if (node.namespace && node.namespace !== '/') {
          node.group += `${node.namespace}`;
        }
        // group using parameters
        settingsCtx.get('groupParameters')?.forEach((parameter) => {
          if (node.parameters && node.parameters.has(parameter)) {
            const parameterValue = `${node.parameters.get(parameter)}`;
            // TODO: Add parameter name to group?
            // const parameterName = parameter.replaceAll('/', ' ');
            // node.group += `/${parameterName}: ${parameterValue}`;

            node.group += `/${parameterValue}`;
          }
        });

        // update the nodeMap
        nodeMap.set(
          `${provider.id}#${node.group}#${nameWithoutNamespace(node)}#${
            node.id
          }`,
          node,
        );
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
      setSelectedTreeItems((prevValues) => [...prevValues]);
    },
    [nodeMap, setProviderNodes, setSelectedTreeItems],
  );

  // Register Callbacks ----------------------------------------------------------------------------------

  /**
   * Callback when nodes on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (itemIds) => {
      setSelectedTreeItems(itemIds);
      // inform details panel tab about selected nodes by user
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, 'default', {}, false),
      );
    },
    [setSelectedTreeItems],
  );

  /**
   * Create and open a new panel with a [SingleTerminalPanel] for a given node
   */
  const createSingleTerminalPanel = useCallback(
    async (type, node, screen, external = false) => {
      if (external && window.CommandExecutor) {
        // create a terminal command
        const provider = rosCtx.getProviderById(node.providerId);
        const terminalCmd = await provider.cmdForType(
          type,
          node?.name,
          '',
          screen,
          '',
        );
        // open screen in a new terminal
        try {
          const result = await window.CommandExecutor?.execTerminal(
            provider.isLocalHost
              ? null
              : SSHCtx.getCredentialHost(provider.host()),
            `"${type.toLocaleUpperCase()} ${node.name}@${provider.host()}"`,
            terminalCmd.cmd,
          );
          if (!result.result) {
            logCtx.error(
              `Can't open external terminal for ${node.name}`,
              result.message,
              true,
            );
          }
        } catch (error) {
          logCtx.error(
            `Can't open external terminal for ${node.name}`,
            error,
            true,
          );
        }
      } else {
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            `${type}-${screen}-${node.name}@${node.providerName}`,
            `${node.name}@${node.providerName}`,
            <SingleTerminalPanel
              type={type}
              providerId={node.providerId}
              node={node}
              screen={screen}
            />,
            false,
            true,
            LAYOUT_TAB_SETS.BORDER_BOTTOM,
            new LayoutTabConfig(true, type, {
              type,
              providerId: node.providerId,
              nodeName: node.name,
              screen,
            }),
          ),
        );
      }
    },
    [rosCtx, SSHCtx],
  );

  /**
   * Create and open a new panel with a [createFileEditorPanel] for selected nodes
   */
  const createFileEditorPanel = (nodes) => {
    const openIds = [];
    nodes.forEach((node) => {
      if (!node.launchInfo) {
        logCtx.error(
          `Could not find launch file for node: [${node.name}]`,
          `Node Info: ${JSON.stringify(node)}`,
        );
        return;
      }
      const rootLaunch = [...node.launchPaths][0];
      if (node.launchPaths.size > 1) {
        // TODO: select
      }
      const launchName = getFileName(node.launchInfo.file_name);
      const provider = rosCtx.getProviderById(node.providerId);
      const packages = provider?.packages?.filter((rosPackage) => {
        return node.launchInfo.file_name.startsWith(
          rosPackage.path.endsWith('/')
            ? rosPackage.path
            : `${rosPackage.path}/`,
        );
      });

      const packageName = packages?.length > 0 ? `${packages[0]?.name}` : '';
      const id = `editor-${node.providerId}-${rootLaunch}`;
      if (!openIds.includes(id)) {
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            id,
            `${launchName} [${packageName}]`,
            <FileEditorPanel
              providerId={node.providerId}
              fileRange={node.launchInfo.file_range}
              currentFilePath={node.launchInfo.file_name}
              rootFilePath={rootLaunch}
            />,
            false,
            true,
            LAYOUT_TAB_SETS.BORDER_TOP,
            new LayoutTabConfig(false, 'editor'),
          ),
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
          `parameter-node-${node.id}`,
          `${node.name}`,
          <ParameterPanel nodes={[node]} providers={null} />,
          false,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, 'parameter'),
        ),
      );
    });
    providers?.forEach((provider) => {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `parameter-provider-${provider}`,
          `${provider}`,
          <ParameterPanel nodes={null} providers={[provider]} />,
          false,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, 'parameter'),
        ),
      );
    });
  }, []);

  const startNodeQueued = async (node) => {
    // let node = getQueueMain();
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      logCtx.debug(`start: ${node.name}`, '');
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain(
          'START',
          node.name,
          false,
          `Provider ${node.providerName} not available`,
        );
      } else {
        // store result to inform the user after queue is finished
        const resultStartNode = await provider.startNode(node);
        if (!resultStartNode.success) {
          addStatusQueueMain(
            'START',
            node.name,
            false,
            `${resultStartNode.details}`,
          );
        } else {
          addStatusQueueMain('START', node.name, true, 'started');
        }
      }
      // trigger next item in the queue
      nextQueueMain();
    }
  };

  const startNodesWithLaunchCheck = useCallback(
    (nodes, ignoreRunState = false) => {
      const withMultiLaunch = [];
      const node2Start = [];
      const withNoLaunch = [];
      const skippedNodes = new Map();
      nodes.forEach((node) => {
        // ignore running and nodes already in the queue
        if (!ignoreRunState && node.status === RosNodeStatus.RUNNING) {
          skippedNodes.set(node.name, 'already running');
        } else if (
          queueItemsQueueMain &&
          queueItemsQueueMain.find(
            (elem) => elem.action === 'START' && elem.name === node.name,
          )
        ) {
          skippedNodes.set(node.name, 'in the start queue');
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
        logCtx.debug(
          `Skipped ${skippedNodes.size} nodes`,
          Object.fromEntries(skippedNodes),
        );
      }
      if (withNoLaunch.length > 0) {
        skippedNodes.concat(withNoLaunch);
        logCtx.error(
          `No launch file for ${withNoLaunch.length} nodes found`,
          node2Start,
        );
      }
      if (withMultiLaunch.length > 0) {
        // let the user select the launch files.
        setNodesAwaitModal(node2Start);
        setNodeMultiLaunches(withMultiLaunch);
      } else if (node2Start.length > 0) {
        setNodesToStart(node2Start);
      }
    },
    [queueItemsQueueMain, logCtx],
  );

  /**
   * Start nodes in the selected list
   */
  const startSelectedNodes = useCallback(() => {
    startNodesWithLaunchCheck(navCtx.selectedNodes);
  }, [navCtx.selectedNodes, startNodesWithLaunchCheck]);

  /**
   * Start nodes from a list of itemIds
   */
  const startNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      startNodesWithLaunchCheck(nodeList);
    },
    [getNodesFromIds, startNodesWithLaunchCheck],
  );

  /** Stop node from queue and trigger the next one. */
  const stopNodeQueued = async (node) => {
    if (node !== null) {
      logCtx.debug(`stop: ${node.name}`, '');
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain(
          'STOP',
          node.name,
          false,
          `Provider ${node.providerName} not available`,
        );
      } else {
        // store result for message
        const resultStopNode = await provider.stopNode(node.id);
        if (!resultStopNode.result) {
          addStatusQueueMain('STOP', node.name, false, resultStopNode.message);
        } else {
          addStatusQueueMain('STOP', node.name, true, 'stopped');
        }
      }
      // trigger next item in the queue
      nextQueueMain();
    }
  };

  /**
   * Stop nodes given in the arguments
   */
  const stopNodes = useCallback(
    (nodeList, onlyWithLaunch, restart) => {
      const nodes2stop = [];
      const skippedNodes = new Map();
      nodeList.forEach((node) => {
        // we stop system nodes only when they are individually selected
        if (node.system_node && nodeList.length > 1) {
          // skip system nodes
          skippedNodes.set(node.name, 'system node');
        } else if (onlyWithLaunch && node.launchPaths.length === 0) {
          skippedNodes.set(node.name, 'stop only with launch files');
        } else if (node.status === RosNodeStatus.RUNNING) {
          // const isRunning = node.status === RosNodeStatus.RUNNING;
          // const isRunning = true;
          // if (!skip) {
          nodes2stop.push(node);
        } else {
          skippedNodes.set(node.name, 'not running');
        }
      });
      if (skippedNodes.size > 0) {
        logCtx.debug(
          `Skipped ${skippedNodes.size} nodes`,
          Object.fromEntries(skippedNodes),
        );
      }
      updateQueueMain(
        nodes2stop.map((node) => {
          return { node, action: 'STOP' };
        }),
      );
      if (restart) {
        startNodesWithLaunchCheck(nodeList, true);
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx],
  );

  /**
   * Stop nodes in the selected list
   */
  const stopSelectedNodes = useCallback(() => {
    stopNodes(navCtx.selectedNodes);
  }, [navCtx.selectedNodes, stopNodes]);

  /**
   * Stop nodes from a list of itemIds
   */
  const stopNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      stopNodes(nodeList);
    },
    [getNodesFromIds, stopNodes],
  );

  /**
   * Restart nodes given in the arguments
   */
  const restartNodes = useCallback(
    (nodeList, onlyWithLaunch) => {
      stopNodes(nodeList, onlyWithLaunch, true); // => true, for restart
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx],
  );

  /**
   * Restart nodes in the selected list
   */
  const restartSelectedNodes = useCallback(() => {
    restartNodes(navCtx.selectedNodes);
  }, [navCtx.selectedNodes, restartNodes]);

  /**
   * Restart nodes from a list of itemIds
   */
  const restartNodesFromId = useCallback(
    (itemIds) => {
      const nodeList = getNodesFromIds(itemIds);
      restartNodes(nodeList);
    },
    [getNodesFromIds, restartNodes],
  );

  /**
   * Kill selected nodes using provider
   */
  const killSelectedNodes = useCallback(() => {
    const nodes2kill = [];
    navCtx.selectedNodes.map(async (node) => {
      // we kill system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      nodes2kill.push(node);
    });
    updateQueueMain(
      nodes2kill.map((node) => {
        return { node, action: 'KILL' };
      }),
    );
  }, [navCtx.selectedNodes, updateQueueMain]);

  /** Kill node in the queue and trigger the next one. */
  const killNodeQueued = async (node) => {
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain(
          'KILL',
          node.name,
          false,
          `Provider ${node.providerName} not available`,
        );
      } else {
        // store result for message
        const resultKillNode = await provider.screenKillNode(node.id);
        if (!resultKillNode.result) {
          addStatusQueueMain('KILL', node.name, false, resultKillNode.message);
        } else {
          addStatusQueueMain('UNREGISTER', node.name, true, 'killed');
        }
      }
      // trigger next item in the queue
      nextQueueMain();
    }
  };

  /**
   * Unregister selected nodes using provider
   */
  const unregisterSelectedNodes = useCallback(() => {
    const nodes2unregister = [];
    navCtx.selectedNodes.map(async (node) => {
      // we unregister system nodes only when they are individually selected
      if (node.system_node && navCtx.selectedNodes.length > 1) return;
      nodes2unregister.push(node);
    });
    updateQueueMain(
      nodes2unregister.map((node) => {
        return { node, action: 'UNREGISTER' };
      }),
    );
  }, [navCtx.selectedNodes, updateQueueMain]);

  /** Unregister node in the queue and trigger the next one. */
  const unregisterNodeQueued = async (node) => {
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain(
          'UNREGISTER',
          node.name,
          false,
          `Provider ${node.providerName} not available`,
        );
      } else {
        // store result for message
        const resultUnregisterNode = await provider.unregisterNode(node.id);
        if (!resultUnregisterNode.result) {
          addStatusQueueMain(
            'UNREGISTER',
            node.name,
            false,
            resultUnregisterNode.message,
          );
        } else {
          addStatusQueueMain('UNREGISTER', node.name, true, 'unregistered');
        }
      }
      // trigger next item in the queue
      nextQueueMain();
    }
  };

  /**
   * Start all ROS nodes belonging to a given provider
   */
  const startProviderNodes = useCallback(
    async (providerId) => {
      if (!providerId || providerId.length === 0) return;
      if (!rosCtx.mapProviderRosNodes.has(providerId)) return;
      // After killing provider, node list still shows  old nodes
      startNodesWithLaunchCheck(rosCtx.mapProviderRosNodes.get(providerId));
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.mapProviderRosNodes, rosCtx],
  );

  /**
   * Stop all ROS nodes belonging to a given provider
   */
  const stopProviderNodes = useCallback(
    (providerId) => {
      if (!providerId || providerId.length === 0) return;
      if (!rosCtx.mapProviderRosNodes.has(providerId)) return;
      stopNodes(rosCtx.mapProviderRosNodes.get(providerId), true);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.mapProviderRosNodes, rosCtx, stopNodes],
  );

  /**
   * Restart all ROS nodes belonging to a given provider
   */
  const restartProviderNodes = useCallback(
    (providerId) => {
      if (!providerId || providerId.length === 0) return;
      if (!rosCtx.mapProviderRosNodes.has(providerId)) return;
      restartNodes(rosCtx.mapProviderRosNodes.get(providerId), true);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.mapProviderRosNodes, rosCtx],
  );

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

            if (
              result.length > 0 &&
              result.indexOf('Purging ROS node logs.') === -1
            ) {
              // should not happen, probably error
              logCtx.error('Could not delete logs', result);
            } else {
              logCtx.success('Logs removed successfully', '');
            }
          }),
        ).catch((error) => {
          logCtx.error(`Could not clear logs for providers`, error);
        });
      }
    },
    [rosCtx, logCtx],
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
        nodes2clear.push(node);
      });
      updateQueueMain(
        nodes2clear.map((node) => {
          return { node, action: 'CLEAR_LOG' };
        }),
      );
    },
    [updateQueueMain],
  );

  /** Delete log file for node in the queue and trigger the next one. */
  const clearNodeLogQueued = async (node) => {
    if (node !== null) {
      const provider = rosCtx.getProviderById(node.providerId);
      if (!provider || !provider.isAvailable()) {
        addStatusQueueMain(
          'CLEAR_LOG',
          node.name,
          false,
          `Provider ${node.providerName} not available`,
        );
      } else {
        const result = await provider.clearLogPaths([node.name]);
        if (result.length === 0) {
          logCtx.error(
            `Could not delete log files for node: [${node.name}]`,
            result,
          );
          addStatusQueueMain('CLEAR_LOG', node.name, false, result);
        } else if (!result[0].result) {
          logCtx.error(
            `Could not delete log files for node: [${node.name}]`,
            result[0].message,
          );
          addStatusQueueMain('CLEAR_LOG', node.name, false, result[0].message);
        } else {
          addStatusQueueMain(
            'CLEAR_LOG',
            node.name,
            true,
            `Removed log files for node: [${node.name}]`,
          );
        }
      }
      // trigger next item in the queue
      nextQueueMain();
    }
  };

  const refreshAllProvider = useCallback(() => {
    rosCtx.providersConnected.forEach((p) => {
      p.updateRosNodes();
      p.updateTimeDiff();
    });
  }, [rosCtx.providersConnected]);

  // Register useEffect Callbacks ----------------------------------------------------------------------------------

  useEffect(() => {
    refreshAllProvider();
  }, [refreshAllProvider]);

  useEffect(() => {
    // filter nodes by user text
    onSearch(filterText);
  }, [filterText, onSearch, providerNodes]);

  useEffect(() => {
    // filter nodes by user text
    let removed = false;
    const newNodeMap = [];
    providerNodes.forEach((item) => {
      if (
        rosCtx.providers.filter((prov) => prov.id === item.providerId).length >
        0
      ) {
        newNodeMap.push(item);
      } else {
        removed = true;
      }
    });
    if (removed) {
      setProviderNodes(newNodeMap);
    }
  }, [rosCtx.providers, providerNodes]);

  useEffect(() => {
    // each time selected items are changed (user select, nodes or launch changes)
    // get list of nodes
    const sn = getNodesFromIds(selectedTreeItems);
    // set the selected nodes
    navCtx.setSelectedNodes(sn);

    // clear selected provider
    navCtx.setSelectedProviders([]);
    const selectedProvidersLocal = [];
    // update setSelectedNodes based on trees selectedItems
    selectedTreeItems.forEach((item) => {
      // search selected host
      if (rosCtx.getProviderById(item)) {
        selectedProvidersLocal.push(item);
      }
    });
    navCtx.setSelectedProviders(selectedProvidersLocal);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedTreeItems, getNodesFromIds, rosCtx]);

  useEffect(() => {
    if (nodesToStart) {
      // put all selected nodes to the start queue
      updateQueueMain(
        nodesToStart.map((node) => {
          return { node, action: 'START' };
        }),
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
        if (queueItem.action === 'START') {
          startNodeQueued(queueItem.node);
        } else if (queueItem.action === 'STOP') {
          stopNodeQueued(queueItem.node);
        } else if (queueItem.action === 'UNREGISTER') {
          unregisterNodeQueued(queueItem.node);
        } else if (queueItem.action === 'KILL') {
          killNodeQueued(queueItem.node);
        } else if (queueItem.action === 'CLEAR_LOG') {
          clearNodeLogQueued(queueItem.node);
        }
      } else {
        // queue is finished, print success results
        [
          ['STOP', 'stopped'],
          ['START', 'started'],
          ['KILL', 'killed'],
          ['UNREGISTER', 'unregistered'],
          ['CLEAR_LOG', 'logs cleared'],
        ].forEach((action) => {
          const success = successQueueMain(action[0]);
          if (success.length > 0) {
            const infoDict = {};
            success.forEach((item) => {
              infoDict[item.itemName] = item.message;
            });
            logCtx.success(
              `${success.length} nodes ${action[1]}`,
              infoDict,
              true,
            );
          }
        });
        // queue is finished, print failed results
        ['STOP', 'START', 'KILL', 'UNREGISTER', 'CLEAR_LOG'].forEach(
          (action) => {
            const failed = failedQueueMain(action);
            if (failed.length > 0) {
              const infoDict = {};
              failed.forEach((item) => {
                infoDict[item.itemName] = item.message;
              });
              logCtx.warn(
                `Failed to ${action.toLocaleLowerCase()} ${
                  failed.length
                } nodes`,
                infoDict,
                true,
              );
            }
          },
        );
        // clear queue and results
        clearQueueMain();
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sizeQueueMain, indexQueueMain]);

  return (
    <Box
      // ref={panelRef}
      width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      <Stack spacing={0.5} direction="column" width="100%" height="100%">
        {sizeQueueMain === 0 && (
          <Stack direction="row" spacing={1}>
            <SearchBar
              onSearch={(value) => {
                setFilterText(value);
              }}
              placeholder="Search nodes (comma separated)"
              defaultValue={filterText}
              fullWidth
            />
            <Tooltip
              title="Reload node list"
              placement="left"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
              <IconButton
                size="small"
                onClick={() => {
                  refreshAllProvider();
                }}
              >
                <RefreshIcon sx={{ fontSize: 18 }} />
              </IconButton>
            </Tooltip>
          </Stack>
        )}
        {sizeQueueMain > 0 && (
          <Paper elevation={2}>
            <Stack
              alignItems="center"
              justifyItems="center"
              direction="row"
              spacing={0.5}
              sx={{ marginRight: 2 }}
            >
              <LinearProgress
                sx={{ width: '100%' }}
                variant="determinate"
                value={progressQueueMain}
              />
              <FormLabel>
                {indexQueueMain}/{sizeQueueMain}
              </FormLabel>
              <IconButton
                onClick={() => {
                  clearQueueMain();
                }}
                size="small"
              >
                <StopIcon sx={{ fontSize: 24 }} />
              </IconButton>
            </Stack>
          </Paper>
        )}
        {(!rosCtx.providersConnected ||
          rosCtx.providersConnected.length === 0) && (
          <Alert severity="info">
            <AlertTitle>No providers available</AlertTitle>
            Please connect to a ROS provider
          </Alert>
        )}

        <Stack direction="row" height="100%" overflow="auto">
          <Box width="100%" height="100%" overflow="auto">
            <HostTreeView
              providerNodeTree={providerNodeTree}
              launchContentList={launchContentList}
              onNodeSelect={handleSelect}
              startNodes={startNodesFromId}
              stopNodes={stopNodesFromId}
              restartNodes={restartNodesFromId}
              startProviderNodes={startProviderNodes}
              stopProviderNodes={stopProviderNodes}
              restartProviderNodes={restartProviderNodes}
            />
          </Box>
          <Box>
            <Paper elevation={2} sx={{ border: 0 }}>
              {navCtx.selectedProviders &&
                navCtx.selectedProviders.length > 0 && (
                  <ButtonGroup
                    orientation="vertical"
                    aria-label="provider control group"
                  >
                    <Tooltip
                      title="Start All"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Start All"
                        onClick={() => {
                          navCtx.selectedProviders.map(async (provider) => {
                            try {
                              await startProviderNodes(provider);
                            } catch (error) {
                              logCtx.error(
                                `Could not start nodes for provider: ${provider.name()} `,
                              );
                            }
                          });
                        }}
                      >
                        <PlayArrowIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip
                      title="Stop All"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Stop All"
                        onClick={() => {
                          navCtx.selectedProviders.forEach((provider) => {
                            stopProviderNodes(provider);
                          });
                        }}
                      >
                        <StopIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                    <Divider />
                    <Tooltip
                      title="Restart All"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Restart All"
                        onClick={() => {
                          navCtx.selectedProviders.forEach((provider) => {
                            restartProviderNodes(provider);
                          });
                        }}
                      >
                        <RestartAltIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  </ButtonGroup>
                )}
              {navCtx.selectedProviders &&
                navCtx.selectedProviders.length > 0 && <Divider />}
              {navCtx.selectedProviders &&
                navCtx.selectedProviders.length > 0 && (
                  <ButtonGroup
                    // showLabels
                    // sx={{
                    //   backgroundColor: settingsCtx.get('backgroundColor'),
                    // }}
                    orientation="vertical"
                    aria-label="ros node control group"
                  >
                    <Tooltip
                      title="Open Terminal (external terminal with shift+click)"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Open Terminal"
                        onClick={(event) => {
                          // open a new terminal for each selected provider
                          navCtx.selectedProviders.forEach((providerId) => {
                            const prov = rosCtx.getProviderById(providerId);
                            const emptyNode = new RosNode();
                            emptyNode.name = 'terminal';
                            emptyNode.providerId = providerId;
                            emptyNode.providerName = prov?.name();
                            createSingleTerminalPanel(
                              CmdType.TERMINAL,
                              emptyNode,
                              '',
                              event.nativeEvent.shiftKey,
                            );
                          });
                        }}
                      >
                        <TerminalIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip
                      title="Select screens (external terminal with shift+click)"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Select screens"
                        onClick={(event) => {
                          navCtx.selectedProviders.forEach((providerId) => {
                            const prov = rosCtx.getProviderById(providerId);
                            const emptyNode = new RosNode();
                            emptyNode.name = prov?.name();
                            emptyNode.providerId = providerId;
                            emptyNode.providerName = prov?.name();
                            emptyNode.screens = [];
                            prov?.screens.forEach((screen) => {
                              emptyNode.screens = [
                                ...emptyNode.screens,
                                ...screen.screens,
                              ];
                            });
                            const sl = {
                              node: emptyNode,
                              external: event.nativeEvent.shiftKey,
                            };
                            setNodeScreens((prevNodes) =>
                              prevNodes ? [...prevNodes, sl] : [sl],
                            );
                          });
                        }}
                      >
                        <DynamicFeedOutlinedIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip
                      title="Parameters"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Parameters"
                        onClick={() => {
                          createParameterPanel(null, navCtx.selectedProviders);
                        }}
                      >
                        <TuneIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>

                    <Tooltip
                      title="ros clean purge"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="ros clean purge"
                        onClick={() => {
                          setRosCleanPurge(true);
                        }}
                      >
                        <DeleteSweepIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  </ButtonGroup>
                )}
              {navCtx.selectedNodes && navCtx.selectedNodes.length > 0 && (
                <ButtonGroup
                  orientation="vertical"
                  aria-label="ros node control group"
                >
                  <Tooltip
                    title="Start"
                    placement="left"
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <IconButton
                      size="medium"
                      aria-label="Start"
                      onClick={() => {
                        startSelectedNodes();
                      }}
                    >
                      <PlayArrowIcon fontSize="inherit" />
                    </IconButton>
                  </Tooltip>
                  <Tooltip
                    title="Stop"
                    placement="left"
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <IconButton
                      size="medium"
                      aria-label="Stop"
                      onClick={() => {
                        stopSelectedNodes();
                      }}
                    >
                      <StopIcon fontSize="inherit" />
                    </IconButton>
                  </Tooltip>
                  <Divider />
                  <Tooltip
                    title="Restart"
                    placement="left"
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <IconButton
                      size="medium"
                      aria-label="Restart"
                      onClick={() => {
                        restartSelectedNodes();
                      }}
                    >
                      <RestartAltIcon fontSize="inherit" />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Kill" placement="left">
                    <IconButton
                      size="medium"
                      aria-label="Kill"
                      onClick={() => {
                        killSelectedNodes();
                      }}
                    >
                      <CancelPresentationIcon fontSize="inherit" />
                    </IconButton>
                  </Tooltip>
                  <Tooltip title="Unregister" placement="left">
                    <IconButton
                      size="medium"
                      aria-label="Unregister"
                      onClick={() => {
                        unregisterSelectedNodes();
                      }}
                    >
                      <DeleteForeverIcon fontSize="inherit" />
                    </IconButton>
                  </Tooltip>
                </ButtonGroup>
              )}
              {navCtx.selectedNodes && navCtx.selectedNodes.length > 0 && (
                <Divider />
              )}
              {navCtx.selectedNodes && navCtx.selectedNodes.length > 0 && (
                <ButtonGroup
                  // showLabels
                  // sx={{
                  //   backgroundColor: settingsCtx.get('backgroundColor'),
                  // }}
                  orientation="vertical"
                  aria-label="ros node control group"
                >
                  {navCtx.selectedProviders.length === 0 && (
                    <Tooltip
                      title="Edit"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Edit"
                        onClick={() => {
                          createFileEditorPanel(navCtx.selectedNodes);
                        }}
                      >
                        <BorderColorIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  )}

                  {navCtx.selectedProviders.length === 0 && (
                    <Tooltip
                      title="Parameters"
                      placement="left"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        size="medium"
                        aria-label="Parameters"
                        onClick={() => {
                          createParameterPanel(navCtx.selectedNodes, null);
                        }}
                      >
                        <TuneIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  )}
                  <Divider />
                  {navCtx.selectedProviders.length === 0 && (
                    <Tooltip
                      title="Screen (external terminal with shift+click)"
                      placement="left"
                    >
                      <IconButton
                        size="medium"
                        aria-label="Screen"
                        onClick={(event) => {
                          navCtx.selectedNodes.forEach((node) => {
                            if (node.screens.length === 1) {
                              // 1 screen available
                              node.screens.forEach((screen) => {
                                createSingleTerminalPanel(
                                  CmdType.SCREEN,
                                  node,
                                  screen,
                                  event.nativeEvent.shiftKey,
                                );
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
                                    ],
                              );
                            } else {
                              // no screens, try to find by node name instead
                              createSingleTerminalPanel(
                                CmdType.SCREEN,
                                node,
                                undefined,
                                event.nativeEvent.shiftKey,
                              );
                            }
                          });
                        }}
                      >
                        <WysiwygIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  )}

                  {navCtx.selectedProviders.length === 0 && (
                    <Tooltip
                      title="Log (external terminal with shift+click)"
                      placement="left"
                    >
                      <IconButton
                        size="medium"
                        aria-label="Log"
                        onClick={(event) => {
                          navCtx.selectedNodes.forEach((node) => {
                            createSingleTerminalPanel(
                              CmdType.LOG,
                              node,
                              undefined,
                              event.nativeEvent.shiftKey,
                            );
                          });
                        }}
                      >
                        <SubjectIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  )}

                  {navCtx.selectedProviders.length === 0 && (
                    <Tooltip title="Clear Logs" placement="left">
                      <IconButton
                        size="medium"
                        aria-label="Clear Logs"
                        onClick={() => {
                          clearLogs(navCtx.selectedNodes, null);
                        }}
                      >
                        <DeleteSweepIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip>
                  )}
                </ButtonGroup>
              )}
            </Paper>
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
                const nodeWithOpt = nodeScreens.find((nodeMultiple) =>
                  nodeMultiple.node.screens.includes(screen),
                );
                createSingleTerminalPanel(
                  CmdType.SCREEN,
                  nodeWithOpt.node,
                  screen,
                  nodeWithOpt.external,
                );
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
                const node = nodesAwaitModal.find((n) =>
                  n.name.includes(item.title),
                );
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
