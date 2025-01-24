import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { getFileName, LaunchContent, LaunchFile, RosNode } from "@/renderer/models";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { CmdType, Provider } from "@/renderer/providers";
import { generateUniqueId, nodeNameWithoutNamespace, removeDDSuid } from "@/renderer/utils";
import GroupItem, { GroupIcon, NodesCount } from "./GroupItem";
import HostItem from "./HostItem";
import LaunchFileList from "./LaunchFileList";
import NodeItem from "./NodeItem";
import { KeyTreeItem, NodeTree, NodeTreeItem } from "./types";
// import { useTreeViewApiRef } from "@mui/x-tree-view";

function compareTreeItems(a: NodeTreeItem, b: NodeTreeItem): number {
  // place system groups are at the end
  const aSystem = a.treePath.includes("{SYSTEM}");
  const bSystem = b.treePath.includes("{SYSTEM}");
  if (aSystem && !bSystem) {
    return 1;
  }
  if (!aSystem && bSystem) {
    return -1;
  }
  return a.treePath.localeCompare(b.treePath);
}

function compareTreeProvider(a: NodeTreeItem, b: NodeTreeItem): number {
  if (a.providerName && b.providerName) {
    return a.providerName?.localeCompare(b.providerName);
  } else if (a.providerName) {
    return -1;
  } else if (b.providerName) {
    return 1;
  }
  return 0;
}

type HostTreeViewProps = {
  visibleNodes: RosNode[];
  isFiltered: boolean;
  onNodeSelect: (itemIds: string[]) => void; // id of the items in rosCtx.nodeMap
  onProviderSelect: (providerIds: string[]) => void; // id of the providers in rosCtx
  startNodes: (itemIds: string[]) => void; // id of the items in rosCtx.nodeMap
  stopNodes: (itemIds: string[]) => void; // id of the items in rosCtx.nodeMap
  showLoggers: (itemIds: string[]) => void; // id of the items in rosCtx.nodeMap
};

const HostTreeView = forwardRef<HTMLDivElement, HostTreeViewProps>(function HostTreeView(props, ref) {
  const {
    visibleNodes,
    isFiltered = false,
    onNodeSelect = (): void => {},
    onProviderSelect = (): void => {},
    startNodes = (): void => {},
    stopNodes = (): void => {},
    showLoggers = (): void => {},
  } = props;
  // const apiRef = useTreeViewApiRef();
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  // providerNodeTree: list of {providerId: string, nodeTree: object}
  const [providerNodeTree, setProviderNodeTree] = useState<NodeTreeItem[]>([]);
  const [expanded, setExpanded] = useState<string[]>([]);
  const [selectedItems, setSelectedItems] = useState<string[]>([]);
  const [keyNodeList, setKeyNodeList] = useState<KeyTreeItem[]>([]);
  const [avoidGroupWithOneItem, setAvoidGroupWithOneItem] = useState<string>(
    settingsCtx.get("avoidGroupWithOneItem") as string
  );

  useEffect(() => {
    setAvoidGroupWithOneItem(settingsCtx.get("avoidGroupWithOneItem") as string);
  }, [settingsCtx.changed]);

  function createTreeFromNodes(nodes: RosNode[]): void {
    const namespaceSystemNodes: string = settingsCtx.get("namespaceSystemNodes") as string;
    const expandedGroups: string[] = [];
    const nodeItemMap = new Map();
    // generate node tree structure based on node list
    // reference: https://stackoverflow.com/questions/57344694/create-a-tree-from-a-list-of-strings-containing-paths-of-files-javascript
    // ...and keep a list of the tree nodes
    const nodeTree: NodeTreeItem[] = [];
    const level: NodeTree = { nodeTree };
    nodes.forEach((node: RosNode) => {
      let nodePath = node.providerId || "";
      if (node.system_node && namespaceSystemNodes) {
        // for system nodes, e.g.: /{SYSTEM}/ns
        nodePath += namespaceSystemNodes;
        if (node.namespace != "/") {
          nodePath += node.namespace;
        }
      } else {
        // for nodes, e.g.: /robot_ns/{CAP_GROUP}/sub_ns
        let groupNamespace = "";
        let groupName = "";
        let nodeRestNamespace = node.namespace !== "/" ? node.namespace : "";
        if (node.capabilityGroup.namespace) {
          groupNamespace = `${node.capabilityGroup.namespace}`;
          nodeRestNamespace = node.namespace.replace(groupNamespace, "");
        }
        if (node.capabilityGroup.name) {
          groupName = `/${node.capabilityGroup.name}`;
        }
        nodePath += `${groupNamespace}${groupName}${nodeRestNamespace}`;
      }
      nodePath += "/" + node.idGlobal;
      if (!nodeItemMap.has(node.idGlobal)) {
        nodeItemMap.set(node.idGlobal, node);
        nodePath.split("/").reduce((r: NodeTree, name, idx, a) => {
          if (!r[name]) {
            r[name] = { nodeTree: [] };

            // Meaning of [name]:
            //    In case of a node: corresponds to the uniqueId
            //    In case of group: corresponds to group name
            if (nodeItemMap.has(name)) {
              // create a node
              const treePath = `${a.slice(0, -1).join("/")}#${nodeNameWithoutNamespace(node)}-${node.guid}`;
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node,
                name: nodeNameWithoutNamespace(node),
                providerId: undefined,
                providerName: undefined,
              });
            } else {
              // create a (sub)group
              const treePath = name ? a.slice(0, idx + 1).join("/") : "";
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node: null,
                providerId: idx === 0 ? node.providerId : undefined,
                providerName: idx === 0 ? node.providerName : undefined,
                name: idx === 0 ? node.providerName || "" : a.slice(idx, idx + 1).join("/"), // top level is the provider
              });
              expandedGroups.push(treePath);
            }
          }
          return r[name];
        }, level);
      }
    });
    setProviderNodeTree((prevTree) => {
      if (prevTree.length === 0) {
        // use either the expanded state or the key of the node tree (expand the first layer)
        // only at first load
        setExpanded(nodeTree?.map((item) => item.providerId as string));
      }
      return nodeTree;
    });
    if (isFiltered) {
      setExpanded(expandedGroups);
    }
  }

  useEffect(() => {
    createTreeFromNodes(visibleNodes);
  }, [visibleNodes, avoidGroupWithOneItem]);

  /**
   * Callback when items on the tree are expanded/retracted
   */
  function handleToggle(_event: React.SyntheticEvent, nodeIds: string[]): void {
    setExpanded(nodeIds);
  }

  /**
   * Callback when items on the tree are double clicked
   */
  const handleDoubleClick = useCallback(
    function (_event: React.MouseEvent, id: string): void {
      if (!expanded || !id) {
        return;
      }

      // check if providers exists on expanded items
      if (expanded.length === 0) {
        setExpanded([...providerNodeTree.map((item) => item.providerId as string), id]);
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
    [expanded, providerNodeTree]
  );

  /**
   * Callback when items on the tree are double clicked
   */
  const handleDoubleClickOnNode = useCallback(
    function (event: React.MouseEvent, id: string): void {
      const nodeIds = getNodeIdsFromTreeIds([id]);
      nodeIds.map((nodeId) => {
        const node = rosCtx.nodeMap.get(nodeId);
        if (node) {
          if (node.pid > 0 || (node.screens || []).length > 0) {
            if (event.nativeEvent.ctrlKey && !node.system_node) {
              // stop node
              stopNodes([node.idGlobal]);
            } else {
              node.screens?.forEach((screen) => {
                navCtx.openTerminal(
                  CmdType.SCREEN,
                  node.providerId as string,
                  node.name,
                  screen,
                  "",
                  event.nativeEvent.shiftKey,
                  event.nativeEvent.ctrlKey
                );
              });
            }
          } else {
            if (event.nativeEvent.ctrlKey && !node.system_node) {
              // stop node
              startNodes([node.idGlobal]);
            } else {
              navCtx.openTerminal(
                CmdType.LOG,
                node.providerId as string,
                node.name,
                "",
                "",
                event.nativeEvent.shiftKey,
                event.nativeEvent.ctrlKey
              );
            }
          }
        }
      });
    },
    [rosCtx.nodeMap]
  );

  const handleClickOnLoggers = useCallback(
    function (_event: React.MouseEvent, id: string): void {
      if (showLoggers) {
        const nodeIds = getNodeIdsFromTreeIds([id]);
        showLoggers(nodeIds);
      }
    },
    [rosCtx.nodeMap]
  );

  /**
   * Function to get all the IDs belonging to a list of parent IDs
   */
  const getParentAndChildrenIds = useCallback(
    function (parentIds: string[]): string[] {
      let allIds = parentIds;
      parentIds.forEach((id) => {
        const parsedId = id.split("#");
        // a group (with children) must have 1 substring
        if (parsedId.length === 1) {
          // get the children IDs
          const childrenIds = keyNodeList.filter((node) => node.key.startsWith(id)).map((node) => node.key);
          if (childrenIds) {
            allIds = [...allIds, ...childrenIds];
          }
        }
      });
      // remove multiple copies of a selected item
      return [...new Set(allIds)];
    },
    [keyNodeList]
  );

  /**
   * Get nodes for selected ids
   */
  const getNodeIdsFromTreeIds = useCallback(
    function (itemIds: string[]): string[] {
      let nodeList: string[] = [];
      itemIds.forEach((item) => {
        nodeList = [
          ...nodeList,
          ...keyNodeList
            .filter((entry) => {
              if (entry.key.includes("#")) {
                return entry.key === item;
              }
              return entry.key.startsWith(item);
            })
            .map((entry) => {
              return entry.idGlobal ? entry.idGlobal : "";
            }),
        ];
      });
      // filter duplicate entries
      return [...new Set(nodeList)];
    },
    [keyNodeList]
  );

  /**
   * Get providers for selected ids
   */
  function getProvidersFromIds(itemIds: string[]): string[] {
    const provList: string[] = [];
    itemIds.forEach((item) => {
      if (!item.includes("#") && !item.includes("/")) {
        provList.push(item);
      }
    });
    return provList;
  }

  /**
   * Callback when items on the tree are selected by the user
   */
  const handleSelect = useCallback(
    function (_event: React.SyntheticEvent, itemIds: string[]): void {
      // update selected state
      setSelectedItems((prevSelected) => {
        // start with the clicked items, preserving the previous order
        let selectedIds = prevSelected.filter((prevId) => itemIds.includes(prevId));
        selectedIds = [...new Set([...selectedIds, ...itemIds])];
        // in the case of multiple selection (CTRL or SHIFT modifiers):
        if (selectedIds.length > 1) {
          // if a group was previously selected but not anymore, deselect all its children
          prevSelected.forEach((prevId) => {
            const prevParsedId = prevId.split("#");
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
      // inform details panel tab about selected nodes by user
      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
    },
    [getParentAndChildrenIds]
  );

  function keyToNodeName(key: string): { isValidNode: boolean; provider: string; node_name: string } {
    if (!key) return { isValidNode: false, provider: "", node_name: "" };
    const [prefix, node] = key.split("#", 2);
    if (!node) return { isValidNode: false, provider: "", node_name: "" };
    // the namespace begins with /
    // we split to get the provider id
    const splitted = prefix.split("/");
    const provider = splitted[0];
    splitted.shift();
    // the rest of the array is namespace
    // remove groups e.g. {SYSTEM} from namespace
    const namespace: string[] = splitted.filter((item) => !(item.startsWith("{") && item.endsWith("}")));
    // add empty at the beginning to create "/" while join
    namespace.unshift("");
    // add node name
    namespace.push(node);
    return { isValidNode: name !== undefined, provider: provider, node_name: removeDDSuid(namespace.join("/")) };
  }

  /**
   * synchronize selected items and available nodes (important in ROS2)
   * since the running node has an DDS id at the end of the node name separated by '-'
   */
  const updateSelectedNodeIds = useCallback(
    function (): void {
      setSelectedItems((prevItems) => [
        ...getParentAndChildrenIds(
          prevItems.map((selItem) => {
            const selItemSplitted = keyToNodeName(selItem);
            if (selItemSplitted.isValidNode) {
              const newKey = keyNodeList.filter((keyItem) => {
                const keyItemSplitted = keyToNodeName(keyItem.key);
                if (keyItemSplitted.isValidNode) {
                  return (
                    selItemSplitted.provider === keyItemSplitted.provider &&
                    selItemSplitted.node_name === keyItemSplitted.node_name
                  );
                }
                return false;
              });
              if (newKey.length > 0) {
                return newKey[0].key;
              }
            }
            return selItem;
          })
        ),
      ]);
    },
    [getParentAndChildrenIds, rosCtx.mapProviderRosNodes]
  );

  /**
   * synchronize selected items and available nodes (important in ROS2)
   * since the running node has an DDS id at the end of the node name separated by '-'
   */
  useEffect(() => {
    updateSelectedNodeIds();
  }, [
    // update only if providerNodeTree was changed
    providerNodeTree,
  ]);

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
  }, [selectedItems]);

  /**
   * Callback when the event of removing a launch file is triggered
   */
  const onRemoveLaunch = useCallback(
    async function (providerId: string, path: string, masteruri: string): Promise<void> {
      const provider: Provider | undefined = rosCtx.getProviderById(providerId, true);
      if (!provider || !provider.launchUnloadFile) return;

      const request = new LaunchFile(path, masteruri, provider.host());
      const resultLaunchUnloadFile = await provider.launchUnloadFile(request);

      if (resultLaunchUnloadFile) {
        // trigger node's update (will force a reload using useEffect hook)
        // rosCtx.updateNodeList(provider.id);
        // rosCtx.updateLaunchList(provider.id);

        // parse remove result output
        if (resultLaunchUnloadFile.status.code === "OK") {
          logCtx.success(`Launch file [${getFileName(path)}] removed`, `Path: ${path}`);
        } else if (resultLaunchUnloadFile.status.code === "FILE_NOT_FOUND") {
          logCtx.error("Could not remove launch file", `File not found: ${path}`);
        } else {
          logCtx.error("Could not remove launch file", `Error: ${resultLaunchUnloadFile.status.msg}`);
        }
      } else {
        logCtx.error("Invalid reply from [launchUnloadFile]", `This is probably a bug, please report it as issue.`);
      }
    },
    [logCtx, rosCtx.providers]
  );

  /**
   * Callback when the event of reloading a launch file is triggered
   */
  const onReloadLaunch = useCallback(
    async function (providerId: string, path: string, masteruri: string): Promise<void> {
      console.debug(`unused masteruri: ${masteruri}`);
      await rosCtx.reloadLaunchFile(providerId, path);
    },
    [rosCtx.reloadLaunchFile]
  );

  /**
   * Select all nodes on the tree, that belongs to a given launch file and provider
   */
  const selectNodesFromLaunch = useCallback(
    function (providerId: string, launch: LaunchContent): void {
      let treeNodes: string[] = [];
      // launch file contains the names of the nodes
      // find ros nodes with this name
      const providerNodes = rosCtx.mapProviderRosNodes.get(providerId);
      providerNodes?.forEach((treeNode) => {
        const nodes = launch.nodes?.filter((lNode) => {
          return lNode.node_name === treeNode.name;
        });
        if ((nodes || []).length > 0) {
          treeNodes = [...treeNodes, treeNode.idGlobal];
        }
      });
      // get the tree ids for the nodes ids
      const newSelItems = keyNodeList
        .filter((kNode) => {
          return kNode.idGlobal && treeNodes.includes(kNode.idGlobal);
        })
        .map((kNode) => kNode.key);
      setSelectedItems(newSelItems);
    },
    [keyNodeList, rosCtx.mapProviderRosNodes]
  );

  /**
   * Create elements of the tree view component
   */
  const buildHostTreeViewItem = useCallback(
    function (providerId: string, treeItem: NodeTreeItem, newKeyNodeList: KeyTreeItem[]): JSX.Element {
      if (!treeItem) {
        console.error("Invalid item ", providerId, treeItem);
        return <div key={`${providerId}#${generateUniqueId()}`} />;
      }
      let { children, treePath, node, name } = treeItem;
      let namespacePart = "";
      while (avoidGroupWithOneItem && children && children.length === 1) {
        const child = children[0];
        children = child.children;
        treePath = child.treePath;
        node = child.node;
        namespacePart = `${namespacePart}${name}/`;
        name = `${name}/${child.name}`;
      }
      const itemId = treePath;
      if (node && children && children.length === 0) {
        // no children means that item is a RosNode
        newKeyNodeList.push({
          key: itemId,
          idGlobal: node.idGlobal,
        });
        return (
          <NodeItem
            // add all relevant infos to key to update the TreeItem visualization
            key={`${itemId}-${JSON.stringify(node.screens)}`}
            itemId={itemId}
            node={node}
            namespacePart={namespacePart}
            onDoubleClick={(event: React.MouseEvent, itemId: string) => handleDoubleClickOnNode(event, itemId)}
            onShowLoggersClick={(event: React.MouseEvent, itemId: string) => handleClickOnLoggers(event, itemId)}
          />
        );
      }
      // valid children means that item is a group
      const groupName = name; // treePath.split("/").pop();
      newKeyNodeList.push({ key: itemId, idGlobal: undefined });
      return (
        <GroupItem
          key={itemId}
          itemId={itemId}
          groupName={groupName}
          icon={GroupIcon(children, settingsCtx.get("useDarkMode") as boolean)}
          countChildren={NodesCount(children)}
          onDoubleClick={(event: React.MouseEvent, id: string) => {
            handleDoubleClick(event, id);
          }}
        >
          {children.sort(compareTreeItems).map((tItem) => {
            return buildHostTreeViewItem(providerId, tItem, newKeyNodeList);
          })}
        </GroupItem>
      );
    },
    [
      // do not include keyNodeList
      settingsCtx,
      handleDoubleClick,
      selectedItems,
      // NodesCount,
    ]
  );

  /**
   * Memoize the generation of the tree to improve render performance
   * The idea is to prevent rerendering when scrolling/focusing the component
   */
  const generateTree = useMemo(() => {
    const newKeyNodeList: { key: string; idGlobal: string }[] = [];
    const tree = (
      <SimpleTreeView
        // apiRef={apiRef}
        ref={ref as LegacyRef<HTMLUListElement>}
        aria-label="node list"
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        multiSelect
        expandedItems={expanded}
        // sx={{ height: '100%' }}
        selectedItems={selectedItems}
        onExpandedItemsChange={(event: React.SyntheticEvent, itemIds: string[]) => handleToggle(event, itemIds)}
        onSelectedItemsChange={(event: React.SyntheticEvent, itemIds: string[]) => handleSelect(event, itemIds)}
        expansionTrigger={"iconContainer"}
      >
        {providerNodeTree?.sort(compareTreeProvider).map((item) => {
          let providerIsAvailable = false;
          const p = rosCtx.getProviderById(item.providerId as string, true);
          if (p && p.isAvailable()) {
            providerIsAvailable = true;
          }
          if (!p) {
            return "";
          }
          // loop through available hosts
          return (
            <HostItem
              key={p.id}
              provider={p}
              stopNodes={(idGlobalNodes: string[]) => {
                stopNodes(idGlobalNodes);
              }}
              onDoubleClick={(event: React.MouseEvent, id: string) => {
                handleDoubleClick(event, id);
              }}
            >
              {/* Show launch files if host is available (have children) */}
              {providerIsAvailable && (
                <LaunchFileList
                  onMouseOver={(event: React.MouseEvent) => {
                    event.stopPropagation();
                  }}
                  providerId={item.providerId as string}
                  launchContentList={p.launchFiles}
                  selectNodesFromLaunch={(providerId: string, launch: LaunchContent) =>
                    selectNodesFromLaunch(providerId, launch)
                  }
                  onRemoveLaunch={(providerId: string, path: string, masteruri: string) =>
                    onRemoveLaunch(providerId, path, masteruri)
                  }
                  onReloadLaunch={(providerId: string, path: string, masteruri: string) =>
                    onReloadLaunch(providerId, path, masteruri)
                  }
                />
              )}

              {item.children.sort(compareTreeItems).map((sortItem) => {
                return buildHostTreeViewItem(item.providerId as string, sortItem, newKeyNodeList);
              })}
            </HostItem>
          );
        })}
      </SimpleTreeView>
    );
    setKeyNodeList(newKeyNodeList);
    return tree;
  }, [
    expanded,
    providerNodeTree,
    selectedItems,
    rosCtx,
    settingsCtx.changed,
    // handleToggle, <= causes too many re-renders
    // handleSelect, <= causes too many re-renders
    // getMasterSyncNode,     <= causes too many re-renders
    // handleDoubleClick,     <= causes too many re-renders
    // getProviderTags,       <= causes too many re-renders
    // selectNodesFromLaunch, <= causes too many re-renders
    // onRemoveLaunch,        <= causes too many re-renders
    // onReloadLaunch,        <= causes too many re-renders
    // toggleMasterSync,      <= causes too many re-renders
    // createSingleTerminalCmdPanel,  <= causes too many re-renders
    // buildHostTreeViewItem, <= causes too many re-renders
    // setKeyNodeList,        <= causes too many re-renders
  ]);

  return generateTree;
});

export default HostTreeView;
