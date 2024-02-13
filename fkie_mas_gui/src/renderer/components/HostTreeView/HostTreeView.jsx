import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useMemo, useState } from 'react';

import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import ChangeCircleOutlinedIcon from '@mui/icons-material/ChangeCircleOutlined';
import HideSourceIcon from '@mui/icons-material/HideSource';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import { blue, green, grey, red } from '@mui/material/colors';
import { TreeView } from '@mui/x-tree-view';
import { emitCustomEvent } from 'react-custom-events';
import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import { SettingsContext } from '../../context/SettingsContext';
import { CmdType } from '../../providers';

import SingleTerminalPanel from '../../pages/NodeManager/panels/SingleTerminalPanel';
import { generateUniqueId } from '../../utils';
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from '../../utils/events';
import { colorFromHostname } from '../UI/Colors';
import HostTreeViewItem from './HostTreeViewItem';
import {
  getGroupIcon,
  getGroupIconColor,
  getNodeIcon,
  getNodeIconColor,
  namespaceSystemNodes,
} from './HostTreeViewUtils';
import LaunchFileList from './LaunchFileList';

import {
  LaunchFile,
  ProviderLaunchConfiguration,
  RosNodeStatus,
  getFileName,
} from '../../models';

const compareTreeItems = (a, b) => {
  // place system groups are at the end
  const aSystem = a.treePath.includes('{');
  const bSystem = b.treePath.includes('{');
  if (aSystem && !bSystem) {
    return 1;
  }
  if (!aSystem && bSystem) {
    return -1;
  }
  return a.treePath.localeCompare(b.treePath);
};

const compareTreeProvider = (a, b) => {
  return a.providerName?.localeCompare(b.providerName);
};

function HostTreeView({
  providerNodeTree,
  onNodeSelect,
  onProviderSelect,
  startNodes,
  stopNodes,
  restartNodes,
}) {
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const [expanded, setExpanded] = useState([]);
  const [selectedItems, setSelectedItems] = useState([]);
  // keyNodeList: {key: string, idGlobal: string}[]
  const keyNodeList = useMemo(() => {
    return [];
  });

  /**
   * Callback when items on the tree are expanded/retracted
   */
  const handleToggle = useCallback((event, nodeIds) => {
    setExpanded(nodeIds);
  }, []);

  /**
   * Callback when items on the tree are double clicked
   */
  const handleDoubleClick = useCallback(
    (label, id) => {
      if (!expanded || !id) return;

      // check if providers exists on expanded items
      if (expanded.length === 0) {
        setExpanded([...providerNodeTree.map((item) => item.providerId), id]);
        return;
      }

      // check if items are already expanded, and if yes, remove to collapse
      const alreadyExpanded = expanded.find((item) => item === id);
      if (alreadyExpanded) {
        setExpanded((prev) => [...prev.filter((item) => item !== id)]);
        return;
      }

      // finally, just add item to expanded array
      setExpanded((prev) => [...prev, id]);
    },
    [expanded, providerNodeTree],
  );

  /**
   * Function to get all the IDs belonging to a list of parent IDs
   */
  const getParentAndChildrenIds = useCallback(
    (parentIds) => {
      let allIds = parentIds;
      parentIds.forEach((id) => {
        const parsedId = id.split('#');
        // a group (with children) must have 2 substrings separated by #
        if (parsedId.length === 2) {
          // get the list of node IDs of the provider
          const nodeList = rosCtx.mapProviderRosNodes.get(parsedId[0]);
          // get the children IDs
          const childrenIds = nodeList?.filter((node) =>
            node.id.startsWith(id),
          );
          if (childrenIds) {
            allIds = [...allIds, ...childrenIds];
          }
        }
      });
      // remove multiple copies of a selected item
      return [...new Set(allIds)];
    },
    [rosCtx.mapProviderRosNodes],
  );

  /**
   * Get nodes for selected ids
   */
  const getNodeIdsFromTreeIds = useCallback(
    (itemIds) => {
      let nodeList = [];
      itemIds.forEach((item) => {
        nodeList = [
          ...nodeList,
          ...keyNodeList
            .filter((entry) => {
              return entry.key.startsWith(item);
            })
            .map((entry) => {
              return entry.idGlobal;
            }),
        ];
      });
      return nodeList;
    },
    [keyNodeList],
  );

  /**
   * Get providers for selected ids
   */
  const getProvidersFromIds = useCallback((itemIds) => {
    const provList = [];
    itemIds.forEach((item) => {
      if (!item.includes('#')) {
        provList.push(item);
      }
    });
    return provList;
  }, []);

  /**
   * Callback when items on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (event, itemId) => {
      // update selected state
      setSelectedItems((prevSelected) => {
        // start with the clicked items
        let selectedIds = itemId;
        // in the case of multiple selection (CTRL or SHIFT modifiers):
        if (selectedIds.length > 1) {
          // if a group was previously selected but not anymore, deselect all its children
          prevSelected.forEach((prevId) => {
            const prevParsedId = prevId.split('#');
            if (prevParsedId.length === 2 && !selectedIds.includes(prevId)) {
              selectedIds = selectedIds.filter((e) => !e.startsWith(prevId));
            }
          });
          // remove group selection if a node in it was deselected
          prevSelected.forEach((prevId) => {
            if (!selectedIds.some((id) => id === prevId)) {
              selectedIds.forEach((id) => {
                if (prevId.startsWith(id)) {
                  selectedIds = selectedIds.filter((e) => e !== id);
                }
              });
            }
          });
        }
        // add child items for selected groups
        return getParentAndChildrenIds(selectedIds);
      });
    },
    [getParentAndChildrenIds],
  );

  // /**
  //  * effect to remove selected items that are no longer in the providerNodeList
  //  * TODO: CHECK THIS FUNCTION
  //  */
  // useEffect(() => {
  //   setSelectedItems((prevItems) => [
  //     ...getParentAndChildrenIds(
  //       prevItems.filter((item) => {
  //         const itemProvider = item.split('#')[0];
  //         let present = false;
  //         console.log(`GET itemProvider for ${JSON.stringify(item)}`);
  //         if (rosCtx.mapProviderRosNodes.get(itemProvider)) {
  //           rosCtx.mapProviderRosNodes.get(itemProvider).forEach((nodeList) => {
  //             console.log(`GET nodeList for ${JSON.stringify(nodeList)}`);
  //             if (nodeList.id.includes(item)) {
  //               present = true;
  //               console.log(
  //                 `GET PRESENT ${JSON.stringify(nodeList.id)} === item: ${item}`
  //               );
  //             }
  //           });
  //         }
  //         return present;
  //       })
  //     ),
  //   ]);
  // }, [setSelectedItems, getParentAndChildrenIds, rosCtx.mapProviderRosNodes]);

  /**
   * effect to update parent selected nodes when the tree selection changes
   */
  useEffect(() => {
    if (onNodeSelect) {
      onNodeSelect(getNodeIdsFromTreeIds(selectedItems));
    }
    if (onProviderSelect) {
      onProviderSelect(getProvidersFromIds(selectedItems));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedItems]);

  /**
   * Callback when the start floating button of a HostTreeViewItem is clicked
   */
  const onStartClick = useCallback(
    (nodeId) => {
      if (selectedItems.includes(nodeId)) {
        startNodes(getNodeIdsFromTreeIds(selectedItems));
      } else {
        startNodes(getNodeIdsFromTreeIds([nodeId]));
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [selectedItems, keyNodeList],
  );

  /**
   * Callback when the stop floating button of a HostTreeViewItem is clicked
   */
  const onStopClick = useCallback(
    (nodeId) => {
      if (selectedItems.includes(nodeId)) {
        stopNodes(getNodeIdsFromTreeIds(selectedItems));
      } else {
        stopNodes(getNodeIdsFromTreeIds([nodeId]));
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [selectedItems, keyNodeList],
  );

  /**
   * Callback when the restart floating button of a HostTreeViewItem is clicked
   */
  const onRestartClick = useCallback(
    (nodeId) => {
      if (selectedItems.includes(nodeId)) {
        restartNodes(getNodeIdsFromTreeIds(selectedItems));
      } else {
        restartNodes(getNodeIdsFromTreeIds([nodeId]));
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [selectedItems, keyNodeList],
  );

  /**
   * Callback when the event of removing a launch file is triggered
   */
  const onRemoveLaunch = useCallback(
    async (providerId, path, masteruri) => {
      const provider = rosCtx.getProviderById(providerId);
      if (!provider || !provider.launchUnloadFile) return;

      const request = new LaunchFile(path, masteruri, provider.host);
      const resultLaunchUnloadFile = await provider.launchUnloadFile(request);

      if (resultLaunchUnloadFile) {
        // trigger node's update (will force a reload using useEffect hook)
        // rosCtx.updateNodeList(provider.id);
        // rosCtx.updateLaunchList(provider.id);

        // parse remove result output
        if (resultLaunchUnloadFile.status.code === 'OK') {
          logCtx.success(
            `Launch file [${getFileName(path)}] removed`,
            `Path: ${path}`,
          );
        } else if (resultLaunchUnloadFile.status.code === 'FILE_NOT_FOUND') {
          logCtx.error(
            'Could not remove launch file',
            `File not found: ${path}`,
          );
        } else {
          logCtx.error(
            'Could not remove launch file',
            `Error: ${resultLaunchUnloadFile.status.msg}`,
          );
        }
      } else {
        logCtx.error(
          'Invalid reply from [launchUnloadFile]',
          `This is probably a bug, please report it as issue.`,
        );
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [logCtx, rosCtx.providers],
  );

  /**
   * Callback when the event of reloading a launch file is triggered
   */
  const onReloadLaunch = useCallback(
    async (providerId, path, masteruri) => {
      await rosCtx.reloadLaunchFile(providerId, path);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.reloadLaunchFile],
  );

  /**
   * Select all nodes on the tree, that belongs to a given launch file and provider
   */
  const selectNodesFromLaunch = useCallback((providerId, launch) => {
    // console.log('selectNodesFromLaunch', providerId, launch);
  }, []);

  /**
   * Create and open a new panel with a [SingleTerminalPanel] for a given node
   */
  const createSingleTerminalCmdPanel = useCallback(
    (providerId, cmd) => {
      const provider = rosCtx.getProviderById(providerId);
      const id = `cmd-${generateUniqueId()}`;
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          `${cmd}@${provider?.name()}`,
          <SingleTerminalPanel
            id={id}
            type={CmdType.CMD}
            providerId={providerId}
            cmd={cmd}
          />,
          true,
          '',
        ),
      );
    },
    [rosCtx],
  );

  /**
   * Get provider tags
   */
  const getProviderTags = useCallback((provider) => {
    const tags = [];
    if (!provider?.daemon) {
      tags.push({ text: 'No Daemon', color: 'red' });
    }
    if (!provider?.discovery) {
      tags.push({ text: 'No Discovery', color: 'red' });
    }
    return tags;
  }, []);

  /**
   * Check if provider has master sync on
   */
  const getMasterSyncNode = useCallback(
    (providerId) => {
      const foundSyncNode = rosCtx.mapProviderRosNodes
        .get(providerId)
        ?.find((node) => {
          return node.id.includes(`/mas_sync`);
        });
      return foundSyncNode;
    },
    [rosCtx.mapProviderRosNodes],
  );

  const toggleMasterSync = useCallback(
    (provider) => {
      const syncNode = getMasterSyncNode(provider.id);
      if (syncNode) {
        stopNodes([syncNode.idGlobal]);
      } else {
        const lc = new ProviderLaunchConfiguration(
          provider.crossbar.host,
          provider.rosVersion,
        );
        lc.sync.enable = true;
        rosCtx.startConfig(lc);
      }
    },
    [getMasterSyncNode, rosCtx, stopNodes],
  );

  /** Returns count of nodes for given group */
  const getNodesCount = useCallback((children) => {
    let result = 0;
    let itemsCount = 0;
    children.forEach((treeItem) => {
      if (treeItem.children && treeItem.children.length > 0) {
        result += getNodesCount(treeItem.children);
      } else {
        itemsCount += 1;
      }
    });
    return result + itemsCount;
  }, []);

  /**
   * Create elements of the tree view component
   */
  const buildHostTreeViewItem = useCallback(
    (providerId, treeItem) => {
      if (!treeItem) {
        console.error('Invalid item ', providerId, treeItem);
        return <div key={`${providerId}#${generateUniqueId()}`} />;
      }
      const { children, treePath, node } = treeItem;
      const nodeId = `${providerId}#${treePath}`;
      if (node && children && children.length === 0) {
        // no children means that item is a RosNode
        let label = node.name.replace(node.namespace, '');
        label = label[0] === '/' ? label.slice(1) : label;
        // console.log(`ADD:  node.idGlobal`);
        keyNodeList.push({
          key: `${nodeId}#${node.id}`,
          idGlobal: node.idGlobal,
        });
        return (
          <HostTreeViewItem
            key={`${nodeId}#${node.id}`}
            nodeId={`${nodeId}#${node.id}`}
            labelText={label}
            labelIcon={getNodeIcon(node.status)}
            iconColor={getNodeIconColor(node, settingsCtx.get('useDarkMode'))}
            color={blue[700]}
            bgColor={blue[200]}
            paddingLeft={0.0}
            showMultipleScreen={
              node.status === RosNodeStatus.RUNNING && node.screens?.length > 1
            }
            showNoScreen={
              node.status === RosNodeStatus.RUNNING && node.screens?.length < 1
            }
            showGhostScreen={
              node.status !== RosNodeStatus.RUNNING && node.screens?.length > 0
            }
            onStartClick={
              node.status !== RosNodeStatus.RUNNING ? onStartClick : null
            }
            onStopClick={
              node.status !== RosNodeStatus.INACTIVE ? onStopClick : null
            }
            onRestartClick={node.launchInfo ? onRestartClick : null}
            startTooltipText={
              selectedItems.includes(nodeId)
                ? 'Start selected nodes'
                : 'Start this node'
            }
            stopTooltipText={
              selectedItems.includes(nodeId)
                ? 'Stop selected nodes'
                : 'Stop this node'
            }
            restartTooltipText={
              selectedItems.includes(nodeId)
                ? 'Restart selected nodes'
                : 'Restart this node'
            }
            tags={node.tags}
          />
        );
      }
      // valid children means that item is a group
      const groupName = treePath.split('/').pop();
      return (
        <HostTreeViewItem
          key={nodeId}
          nodeId={nodeId}
          labelText={groupName}
          labelIcon={node ? getNodeIcon(node.status) : getGroupIcon(children)}
          iconColor={
            node
              ? getNodeIconColor(node, settingsCtx.get('useDarkMode'))
              : getGroupIconColor(children, settingsCtx.get('useDarkMode'))
          }
          color={blue[700]}
          bgColor={blue[200]}
          paddingLeft={0.5}
          showMultipleScreen={
            node ? node.screens && node.screens.length > 1 : false
          }
          showNoScreen={node ? node.screens && node.screens.length < 1 : false}
          onDoubleClick={handleDoubleClick}
          onStartClick={
            onStartClick
            // (node && node.status !== RosNodeStatus.RUNNING) ||
            // getGroupStatus(children) !== GroupStatus.ALL_RUNNING
            //   ? onStartClick
            //   : null
          }
          onStopClick={
            onStopClick
            // (node && node.status !== RosNodeStatus.INACTIVE) ||
            // getGroupStatus(children) !== GroupStatus.ALL_INACTIVE
            //   ? onStopClick
            //   : null
          }
          onRestartClick={
            (node &&
              node.launchInfo &&
              node.status !== RosNodeStatus.INACTIVE) ||
            groupName !== namespaceSystemNodes.replace('/', '')
              ? onRestartClick
              : null
          }
          startTooltipText={
            selectedItems.includes(nodeId)
              ? 'Start selected nodes'
              : 'Start this group'
          }
          stopTooltipText={
            selectedItems.includes(nodeId)
              ? 'Stop selected nodes'
              : 'Stop this group'
          }
          restartTooltipText={
            selectedItems.includes(nodeId)
              ? 'Restart selected nodes'
              : 'Restart this group'
          }
          countChildren={node ? 0 : getNodesCount(children)}
        >
          {children.sort(compareTreeItems).map((tItem) => {
            return buildHostTreeViewItem(providerId, tItem);
          })}
        </HostTreeViewItem>
      );
    },
    [
      settingsCtx,
      handleDoubleClick,
      onStartClick,
      onStopClick,
      onRestartClick,
      selectedItems,
      getNodesCount,
    ],
  );

  const getHostStyle = (provider) => {
    if (settingsCtx.get('colorizeHosts')) {
      // borderLeft: `3px dashed`,
      // borderColor: colorFromHostname(provider.name()),
      return {
        borderLeftStyle: 'outset',
        borderLeftColor: colorFromHostname(provider.name()),
        borderLeftWidth: '0.6em',
      };
    }
    return {};
  };

  /**
   * Memoize the generation of the tree to improve render performance
   * The idea is to prevent rerendering when scrolling/focusing the component
   */
  const generateTree = useMemo(() => {
    return (
      <TreeView
        aria-label="node list"
        defaultCollapseIcon={<ArrowDropDownIcon />}
        defaultExpandIcon={<ArrowRightIcon />}
        multiSelect
        // use either the expanded state or the key of the node tree (expand the first layer)
        expanded={
          expanded.length > 0
            ? expanded
            : providerNodeTree?.map((item) => item.providerId)
        }
        // sx={{ height: '100%' }}
        selected={selectedItems}
        onNodeToggle={handleToggle}
        onNodeSelect={handleSelect}
      >
        {providerNodeTree?.sort(compareTreeProvider).map((item) => {
          const { providerId, nodeTree } = item;
          let providerIsAvailable = false;
          let providerIsReady = false;
          const p = rosCtx.getProviderById(providerId);
          if (p && p.isAvailable()) {
            providerIsAvailable = true;
          }
          if (p && p.isReady()) {
            providerIsReady = true;
          }
          if (!p) {
            return '';
          }
          // loop through available hosts
          return (
            <HostTreeViewItem
              key={providerId}
              nodeId={providerId}
              sx={getHostStyle(p)}
              buttonIcon={
                p?.rosState.ros_version === '1'
                  ? ChangeCircleOutlinedIcon
                  : null
              }
              buttonIconColor={
                getMasterSyncNode(providerId) ? green[500] : grey[700]
              }
              buttonIconText="Toggle Master Sync"
              onButtonIconClick={() => {
                toggleMasterSync(p);
              }}
              timeSyncActive={
                Math.abs(p.timeDiff) > settingsCtx.get('timeDiffThreshold')
              }
              timeSyncText={`Time not in sync for approx. ${(
                p.timeDiff / 1000
              ).toFixed(3)} s`}
              onTimeSync={(cmd) => {
                createSingleTerminalCmdPanel(p.id, cmd);
              }}
              labelText={p.name()}
              labelIcon={
                providerIsAvailable
                  ? PrecisionManufacturingIcon
                  : HideSourceIcon
              }
              iconColor={providerIsAvailable ? blue[700] : red[700]}
              color={providerIsAvailable ? blue[700] : red[700]}
              bgColor={providerIsAvailable ? blue[200] : red[200]}
              paddingLeft={0.5}
              provider={p || null}
              onDoubleClick={handleDoubleClick}
              onStartClick={onStartClick}
              onStopClick={onStopClick}
              onRestartClick={onRestartClick}
              startTooltipText="Start all nodes"
              stopTooltipText="Stop all nodes"
              restartTooltipText="Restart all nodes"
              tags={providerIsAvailable ? getProviderTags(p) : []}
            >
              {/* Show launch files if host is available (have children) */}
              {providerIsAvailable && (
                <LaunchFileList
                  onMouseOver={(event) => {
                    event.stopPropagation();
                  }}
                  providerId={providerId}
                  launchContentList={p.launchFiles}
                  selectNodesFromLaunch={selectNodesFromLaunch}
                  onRemoveLaunch={onRemoveLaunch}
                  onReloadLaunch={onReloadLaunch}
                />
              )}

              {nodeTree &&
                nodeTree.children.sort(compareTreeItems).map((sortItem) => {
                  return buildHostTreeViewItem(providerId, sortItem);
                })}
            </HostTreeViewItem>
          );
        })}

        {/* this box creates an empty space at the end, to prevent items to be covered by app bar */}
        {/* <Box sx={{ height: 130, width: '100%' }} /> */}
      </TreeView>
    );
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    expanded,
    providerNodeTree,
    selectedItems,
    handleToggle,
    handleSelect,
    rosCtx,
    getMasterSyncNode,
    settingsCtx,
    handleDoubleClick,
    getProviderTags,
    selectNodesFromLaunch,
    onRemoveLaunch,
    onReloadLaunch,
    toggleMasterSync,
    createSingleTerminalCmdPanel,
    buildHostTreeViewItem,
  ]);

  return generateTree;
}

HostTreeView.defaultProps = {
  providerNodeTree: [],
  onNodeSelect: () => {},
  onProviderSelect: () => {},
  startNodes: () => {},
  stopNodes: () => {},
  restartNodes: () => {},
};

HostTreeView.propTypes = {
  providerNodeTree: PropTypes.array,
  onNodeSelect: PropTypes.func,
  onProviderSelect: PropTypes.func,
  startNodes: PropTypes.func,
  stopNodes: PropTypes.func,
  restartNodes: PropTypes.func,
};

export default HostTreeView;
