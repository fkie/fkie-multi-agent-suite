import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useCallback, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { getFileName, LaunchContent, LaunchFile, RosNode } from "@/renderer/models";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { CmdType, Provider } from "@/renderer/providers";
import { generateUniqueId, idFromDDSLocations, nodeNameWithoutNamespace, removeDDSuid } from "@/renderer/utils";
import { Alert, AlertTitle, Box } from "@mui/material";
import GroupItem, { GroupIcon, NodesCount } from "./GroupItem";
import HostItem from "./HostItem";
import LaunchFileList from "./LaunchFileList";
import NodeItem from "./NodeItem";
import { KeyTreeItem, NodeTree, NodeTreeItem } from "./types";
// import { useTreeViewApiRef } from "@mui/x-tree-view";

function compareTreeItems(a: NodeTreeItem, b: NodeTreeItem): number {
  // place system groups are at the end
  const aSystem = a.treePath.match(/({SYSTEM})|({SPAM})/);
  const bSystem = b.treePath.match(/({SYSTEM})|({SPAM})/);
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
  }
  if (a.providerName) {
    return -1;
  }
  if (b.providerName) {
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

export default function HostTreeView(props: HostTreeViewProps): JSX.Element {
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
  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);
  const [openScreenByDefault, setOpenScreenByDefault] = useState<string>(
    settingsCtx.get("openScreenByDefault") as string
  );
  const [spamNodesRegExp, setSpamNodesRegExp] = useState<RegExp | undefined>(getSpamNodesRegExp());

  function getSpamNodesRegExp(): RegExp | undefined {
    const spamNodes = (settingsCtx.get("spamNodes") as string)
      .split(",")
      .filter((item) => item.length > 0)
      .map((item) => `(${item})`)
      .join("|");
    return spamNodes ? new RegExp(String.raw`${spamNodes}`, "g") : undefined;
  }

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setAvoidGroupWithOneItem(settingsCtx.get("avoidGroupWithOneItem") as string);
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
    setSpamNodesRegExp(getSpamNodesRegExp());
    setOpenScreenByDefault(settingsCtx.get("openScreenByDefault") as string);
  }, [settingsCtx, settingsCtx.changed]);

  const createTreeFromNodes: (nodes: RosNode[]) => void = useCallback((nodes) => {
    const namespaceSystemNodes: string = settingsCtx.get("namespaceSystemNodes") as string;
    const expandedGroups: string[] = [];
    const nodeItemMap = new Map();
    const newKeyNodeList: KeyTreeItem[] = [];
    // generate node tree structure based on node list
    // reference: https://stackoverflow.com/questions/57344694/create-a-tree-from-a-list-of-strings-containing-paths-of-files-javascript
    // ...and keep a list of the tree nodes
    const nodeTree: NodeTreeItem[] = [];
    const level: NodeTree = { nodeTree };
    for (const node of nodes) {
      let remoteLocationId = "";
      if (!node.isLocal) {
        // adds a new provider, which has no mas daemon and discovery
        remoteLocationId = idFromDDSLocations(Array.isArray(node.location) ? node.location : [node.location]);
        if (!remoteLocationId) {
          remoteLocationId = node.providerId;
        }
      }
      let nodePath = node.isLocal && node.providerId ? node.providerId : remoteLocationId;
      const isSpamNode = spamNodesRegExp && node.name.match(spamNodesRegExp);
      if (node.system_node && namespaceSystemNodes && !isSpamNode) {
        // for system nodes, e.g.: /{SYSTEM}/ns
        nodePath += namespaceSystemNodes;
        if (node.namespace !== "/") {
          nodePath += node.namespace;
        }
      } else {
        // for nodes, e.g.: /robot_ns/{CAP_GROUP}/sub_ns
        let groupNamespace = "";
        let groupName = "";
        let nodeRestNamespace = node.namespace !== "/" ? node.namespace : "";
        if (node.capabilityGroup.namespace && node.namespace.startsWith(node.capabilityGroup.namespace)) {
          groupNamespace = `${node.capabilityGroup.namespace}`;
          nodeRestNamespace = node.namespace.replace(groupNamespace, "");
        }
        if (node.capabilityGroup.name) {
          groupName = `/${node.capabilityGroup.name}`;
        }
        if (isSpamNode) {
          groupName = "/{SPAM}";
        }
        nodePath += `${groupNamespace}${groupName}${nodeRestNamespace}`;
      }
      nodePath += `/${node.idGlobal}`;
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
              const guidStr = node.guid ? ` ${node.guid}` : "";
              const groupName = a.slice(0, -1).join("/");
              const treePath = `${groupName}#${nodeNameWithoutNamespace(node)}${guidStr}`;
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node,
                name: nodeNameWithoutNamespace(node),
                providerId: undefined,
                providerName: undefined,
              });
              newKeyNodeList.push({
                key: treePath,
                idGlobal: node.idGlobal,
              });
            } else {
              // create a (sub)group
              const treePath = name ? a.slice(0, idx + 1).join("/") : "";
              r.nodeTree.push({
                treePath,
                children: r[name].nodeTree,
                node: null,
                providerId: idx === 0 ? remoteLocationId || node.providerId : undefined,
                providerName: idx === 0 ? remoteLocationId || node.providerName : undefined,
                name: idx === 0 ? node.providerName || "" : a.slice(idx, idx + 1).join("/"), // top level is the provider
              });
              expandedGroups.push(treePath);
              newKeyNodeList.push({
                key: treePath,
                idGlobal: undefined,
              });
            }
          }
          return r[name];
        }, level);
      }
    }
    setKeyNodeList(newKeyNodeList);
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
  }, [spamNodesRegExp]);

  useEffect(() => {
    createTreeFromNodes(visibleNodes);
  }, [visibleNodes, avoidGroupWithOneItem, spamNodesRegExp]);

  /**
   * Callback when items on the tree are expanded/retracted
   */
  function handleToggle(_event: React.SyntheticEvent | null, nodeIds: string[]): void {
    setExpanded(nodeIds);
  }

  /**
   * Callback when items on the tree are double clicked
   */
  const handleDoubleClick = useCallback(
    (_event: React.MouseEvent, id: string): void => {
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
   * Get nodes for selected ids
   */
  const getNodeIdsFromTreeIds = useCallback(
    (itemIds: string[]): string[] => {
      let nodeList: string[] = [];
      for (const item of itemIds) {
        nodeList = [
          ...nodeList,
          ...keyNodeList
            .filter((entry) => {
              // running nodes ends with id in their name, loaded from launch file not
              // we have to remove id if we compare running with not running nodes
              const [entryName, entryId] = entry.key.split(" ");
              const [itemName, itemId] = item.split(" ");
              if (entryName === itemName) {
                if (entryId && itemId) {
                  return entryId === itemId;
                }
                return true;
              }
              return false;
            })
            .map((entry) => {
              return entry.idGlobal ? entry.idGlobal : "";
            }),
        ];
      }
      // filter duplicate entries
      return [...new Set(nodeList)];
    },
    [keyNodeList]
  );

  /**
   * Callback when items on the tree are double clicked
   */

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const handleDoubleClickOnNode = useCallback(
    (event: React.MouseEvent, id: string): void => {
      const nodeIds = getNodeIdsFromTreeIds([id]);
      nodeIds.map((nodeId) => {
        const node = rosCtx.nodeMap.get(nodeId);
        if (node) {
          let startStopAction = false;
          if (event.nativeEvent.ctrlKey && !event.nativeEvent.shiftKey && !node.system_node) {
            if (node.pid > 0 || (node.screens || []).length > 0) {
              // stop node
              stopNodes([node.idGlobal]);
              startStopAction = true;
            } else {
              // stop node
              startNodes([node.idGlobal]);
              startStopAction = true;
            }
          }
          // open screen or log
          if (!startStopAction) {
            if (
              Boolean(event.nativeEvent.shiftKey) !== Boolean(openScreenByDefault) &&
              (node.screens || []).length > 0
            ) {
              for (const screen of node.screens || []) {
                navCtx.openTerminal(
                  CmdType.SCREEN,
                  node.providerId as string,
                  node.name,
                  screen,
                  "",
                  false,
                  event.nativeEvent.ctrlKey
                );
              }
            } else {
              navCtx.openTerminal(
                CmdType.LOG,
                node.providerId as string,
                node.name,
                "",
                "",
                false,
                event.nativeEvent.ctrlKey
              );
            }
          }
        }
      });
    },
    [rosCtx.nodeMap, keyNodeList, openScreenByDefault]
  );

  /**
   * Callback when items on the tree are middle clicked
   */

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const handleMiddleClickOnNode = useCallback(
    (event: React.MouseEvent, id: string): void => {
      const nodeIds = getNodeIdsFromTreeIds([id]);
      nodeIds.map((nodeId) => {
        const node = rosCtx.nodeMap.get(nodeId);
        if (node) {
          navCtx.openTerminal(
            CmdType.SCREEN,
            node.providerId as string,
            node.name,
            "",
            "",
            event.nativeEvent.shiftKey,
            event.nativeEvent.ctrlKey
          );
        }
      });
    },
    [rosCtx.nodeMap, keyNodeList]
  );

  const handleClickOnLoggers = useCallback(
    (_event: React.MouseEvent, id: string): void => {
      if (showLoggers) {
        const nodeIds = getNodeIdsFromTreeIds([id]);
        showLoggers(nodeIds);
      }
    },
    [rosCtx.nodeMap]
  );

  function allChildrenSelected(groupName: string, children: string[]): boolean {
    // get all nodes of the group and check if they are all in children
    const childrenIds = keyNodeList.filter((node) => node.key.startsWith(`${groupName}`)).map((node) => node.key);
    const notInAllIds = childrenIds.filter((childId) => children.indexOf(childId) === -1);
    return notInAllIds.length === 1 && notInAllIds[0] === groupName;
  }

  /**
   * Function to get all the IDs belonging to a list of parent IDs
   */
  const getParentAndChildrenIds = useCallback(
    (parentIds: string[]): string[] => {
      let allIds = parentIds;
      let updatedGroup = false;
      // get all children IDs
      for (const id of parentIds) {
        const parsedId = id.split("#");
        // a group (with children) must have 1 substring
        if (parsedId.length === 1) {
          // get the children IDs
          const childrenIds = keyNodeList.filter((node) => node.key.startsWith(id)).map((node) => node.key);
          if (childrenIds) {
            allIds = [...allIds, ...childrenIds];
          }
        }
      }
      // get all parent IDs with all children selected
      for (const id of allIds) {
        const parsedId = id.split("#");
        // a group (with children) must have 1 substring
        if (parsedId.length === 1) {
          if (parsedId[0].indexOf("/") > -1) {
            const parentGroup = parsedId[0].slice(0, parsedId[0].lastIndexOf("/"));
            // add parent group to the list of IDs if it is not already there and has all its children selected
            if (allChildrenSelected(parentGroup, allIds)) {
              allIds.push(parentGroup);
              updatedGroup = true;
            }
          }
        } else {
          // we have a node, get the group name and check if it is in allIds
          // or if all nodes of the group are in the allIds, select it too.
          const groupName = parsedId[0];
          if (allIds.indexOf(groupName) === -1) {
            // add if all children are in allIds
            if (allChildrenSelected(groupName, allIds)) {
              allIds.push(groupName);
              updatedGroup = true;
            }
          }
        }
      }
      if (updatedGroup) {
        // if a new group was added we have to check if parent group has all children selected.
        allIds = getParentAndChildrenIds(allIds);
      }
      // remove multiple copies of a selected item
      return [...new Set(allIds)];
    },
    [keyNodeList]
  );

  /**
   * Get providers for selected ids
   */
  function getProvidersFromIds(itemIds: string[]): string[] {
    const provList: string[] = [];
    for (const item of itemIds) {
      if (!item.includes("#") && !item.includes("/")) {
        provList.push(item);
      }
    }
    return provList;
  }

  /**
   * Callback when items on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (_event: React.SyntheticEvent | null, itemIds: string[]): void => {
      // update selected state
      setSelectedItems((prevSelected) => {
        // start with the clicked items, preserving the previous order
        let selectedIds = prevSelected.filter((prevId) => itemIds.includes(prevId));
        selectedIds = [...new Set([...selectedIds, ...itemIds])];
        // in the case of multiple selection (CTRL or SHIFT modifiers):
        if (selectedIds.length > 1) {
          // if a group was previously selected but not anymore, deselect all its children
          for (const prevId of prevSelected) {
            const prevParsedId = prevId.split("#");
            if (prevParsedId.length === 1 && !selectedIds.includes(prevId)) {
              selectedIds = selectedIds.filter((e) => !e.startsWith(prevId));
            }
          }
          // remove group selection if a node in it was deselected
          for (const prevId of prevSelected) {
            if (!selectedIds.some((id) => id === prevId)) {
              for (const id of selectedIds) {
                if (prevId.startsWith(id)) {
                  selectedIds = selectedIds.filter((e) => e !== id);
                }
              }
            }
          }
        }
        // add child items for selected groups
        return getParentAndChildrenIds(selectedIds);
      });
      // inform details panel tab about selected nodes by user
      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.DETAILS, "default"));
    },
    [keyNodeList]
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
    return { isValidNode: node !== undefined, provider: provider, node_name: removeDDSuid(namespace.join("/")) };
  }

  /**
   * synchronize selected items and available nodes (important in ROS2)
   * since the running node has an DDS id at the end of the node name separated by '-'
   */
  const updateSelectedNodeIds = useCallback((): void => {
    setSelectedItems((prevItems) => [
      ...getParentAndChildrenIds(
        prevItems.map((selItem) => {
          const selItemSplitted = keyToNodeName(selItem);
          if (selItemSplitted.isValidNode) {
            const newKey = keyNodeList.filter((keyItem) => {
              const keyItemSplitted = keyToNodeName(keyItem.key);
              if (keyItemSplitted.isValidNode) {
                if (selItemSplitted.provider === keyItemSplitted.provider) {
                  // running nodes ends with id in their name, loaded from launch file not
                  // we have to remove id if we compare running with not running nodes
                  const [entryName, entryId] = keyItemSplitted.node_name.split(" ");
                  const [itemName, itemId] = selItemSplitted.node_name.split(" ");
                  if (entryName === itemName) {
                    if (entryId && itemId) {
                      return entryId === itemId;
                    }
                    return true;
                  }
                }
                return false;
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
  }, [keyNodeList, rosCtx.mapProviderRosNodes]);

  /**
   * synchronize selected items and available nodes (important in ROS2)
   * since the running node has an DDS id at the end of the node name separated by '-'
   */

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    updateSelectedNodeIds();
  }, [
    // update only if providerNodeTree was changed
    providerNodeTree,
  ]);

  /**
   * effect to update parent selected nodes when the tree selection changes
   */

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    if (onNodeSelect) {
      onNodeSelect(getNodeIdsFromTreeIds(selectedItems));
    }
    if (onProviderSelect) {
      onProviderSelect(getProvidersFromIds(selectedItems));
    }
  }, [selectedItems]);

  useEffect(() => {
    if (
      navCtx.selectedProviders.length === 1 &&
      (selectedItems.length !== 1 || !selectedItems.includes(navCtx.selectedProviders[0]))
    ) {
      handleSelect(null, navCtx.selectedProviders);
    }
  }, [navCtx.selectedProviders]);

  /**
   * Callback when the event of removing a launch file is triggered
   */
  const onRemoveLaunch = useCallback(
    async (providerId: string, path: string, masteruri: string): Promise<void> => {
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
          logCtx.success(
            `Launch file [${getFileName(path)}] removed`,
            `Path: ${path}`,
            `${getFileName(path)} unloaded`
          );
        } else if (resultLaunchUnloadFile.status.code === "FILE_NOT_FOUND") {
          logCtx.error("Could not remove launch file", `File not found: ${path}`, "file not found");
        } else {
          logCtx.error(
            "Could not remove launch file",
            `Error: ${resultLaunchUnloadFile.status.msg}`,
            "could not remove launch file"
          );
        }
        provider.updateLaunchContent();
      } else {
        logCtx.error(
          "Invalid reply from [launchUnloadFile]",
          "This is probably a bug, please report it as issue.",
          "invalid reply"
        );
      }
    },
    [logCtx, rosCtx.providers]
  );

  /**
   * Callback when the event of reloading a launch file is triggered
   */
  const onReloadLaunch = useCallback(
    async (providerId: string, path: string, masteruri: string): Promise<void> => {
      console.debug(`unused masteruri: ${masteruri}`);
      await rosCtx.reloadLaunchFile(providerId, path);
    },
    [rosCtx.reloadLaunchFile]
  );

  /**
   * Select all nodes on the tree, that belongs to a given launch file and provider
   */
  const selectNodesFromLaunch = useCallback(
    (providerId: string, launch: LaunchContent): void => {
      let treeNodes: string[] = [];
      // launch file contains the names of the nodes
      // find ros nodes with this name
      const providerNodes = rosCtx.mapProviderRosNodes.get(providerId);
      if (providerNodes) {
        for (const treeNode of providerNodes.values()) {
          const nodes = launch.nodes?.filter((lNode) => {
            return lNode.node_name === treeNode.name;
          });
          if ((nodes || []).length > 0) {
            treeNodes = [...treeNodes, treeNode.idGlobal];
          }
        }
      }
      // get the tree ids for the nodes ids
      const newSelItems = keyNodeList
        .filter((kNode) => {
          return kNode.idGlobal && treeNodes.includes(kNode.idGlobal);
        })
        .map((kNode) => kNode.key);
      setSelectedItems(getParentAndChildrenIds(newSelItems));
    },
    [keyNodeList, rosCtx.mapProviderRosNodes]
  );

  /**
   * Create elements of the tree view component
   */
  const buildHostTreeViewItem = useCallback(
    (providerId: string, treeItem: NodeTreeItem): JSX.Element => {
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
        name = child.name;
      }
      const itemId = treePath;
      if (node && children && children.length === 0) {
        // no children means that item is a RosNode
        return (
          <NodeItem
            // add all relevant infos to key to update the TreeItem visualization
            key={`${itemId}-${JSON.stringify(node.screens)}`}
            itemId={itemId}
            node={node}
            namespacePart={namespacePart}
            onDoubleClick={(event: React.MouseEvent, itemId: string) => handleDoubleClickOnNode(event, itemId)}
            onMiddleClick={(event: React.MouseEvent, itemId: string) => handleMiddleClickOnNode(event, itemId)}
            onShowLoggersClick={(event: React.MouseEvent, itemId: string) => handleClickOnLoggers(event, itemId)}
          />
        );
      }
      // valid children means that item is a group
      return (
        <GroupItem
          key={itemId}
          itemId={itemId}
          groupName={`${namespacePart}${name}`}
          icon={<GroupIcon treeItems={children} isDarkMode={isDarkMode} groupName={name} />}
          countChildren={NodesCount(children)}
          onDoubleClick={(event: React.MouseEvent, id: string) => {
            handleDoubleClick(event, id);
          }}
        >
          {children.sort(compareTreeItems).map((tItem) => {
            return buildHostTreeViewItem(providerId, tItem);
          })}
        </GroupItem>
      );
    },
    [
      // do not include keyNodeList
      avoidGroupWithOneItem,
      visibleNodes,
      expanded,
      providerNodeTree,
      selectedItems,
      rosCtx,
      settingsCtx.changed,
      rosCtx.nodeMap,
      // NodesCount,
    ]
  );

  /**
   * Memoize the generation of the tree to improve render performance
   * The idea is to prevent rerendering when scrolling/focusing the component
   */
  // const generateTree = useMemo(() => {
  //   const newKeyNodeList: { key: string; idGlobal: string }[] = [];
  //   const tree = (
  //     <Box
  //       width="100%"
  //       height="100%"
  //       overflow="auto"
  //       onClick={() => {
  //         // deselect topics
  //         setSelectedItems([]);
  //       }}
  //     >
  //       {(!rosCtx.providers || rosCtx.providers.length === 0) && (
  //         <Alert severity="info">
  //           <AlertTitle>No providers available</AlertTitle>
  //           Please connect to a ROS provider
  //         </Alert>
  //       )}
  //       <SimpleTreeView
  //         onClick={(event) => {
  //           // enable deselection
  //           event.stopPropagation();
  //         }}
  //         // apiRef={apiRef}
  //         aria-label="node list"
  //         slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
  //         multiSelect
  //         expandedItems={expanded}
  //         // sx={{ height: '100%' }}
  //         selectedItems={selectedItems}
  //         onExpandedItemsChange={(event: React.SyntheticEvent | null, itemIds: string[]) =>
  //           handleToggle(event, itemIds)
  //         }
  //         onSelectedItemsChange={(event: React.SyntheticEvent | null, itemIds: string[]) =>
  //           handleSelect(event, itemIds)
  //         }
  //         expansionTrigger={"iconContainer"}
  //         // selectionPropagation={{ parents: true, descendants: true }}
  //       >
  //         {providerNodeTree?.sort(compareTreeProvider).map((item) => {
  //           let providerIsAvailable = false;
  //           let p = rosCtx.getProviderById(item.providerId as string, true);
  //           if (p?.isAvailable()) {
  //             providerIsAvailable = true;
  //           }
  //           if (!p) {
  //             // no provider was found: no provider found: we have a remote node and the provider daemon is not started there.
  //             // We create a new "not connected" provider.
  //             if (item.providerId) {
  //               p = new Provider(settingsCtx, item.providerId, rosCtx.rosInfo?.version || "2");
  //               p.id = item.providerId;
  //             } else {
  //               return "";
  //             }
  //           }
  //           // loop through available hosts
  //           return (
  //             <HostItem
  //               key={p.id}
  //               provider={p}
  //               stopNodes={(idGlobalNodes: string[]) => {
  //                 stopNodes(idGlobalNodes);
  //               }}
  //               onDoubleClick={(event: React.MouseEvent, id: string) => {
  //                 handleDoubleClick(event, id);
  //               }}
  //             >
  //               {/* Show launch files if host is available (have children) */}
  //               {providerIsAvailable && (
  //                 <LaunchFileList
  //                   onMouseOver={(event: React.MouseEvent) => {
  //                     event.stopPropagation();
  //                   }}
  //                   providerId={p.id}
  //                   launchContentList={p.launchFiles}
  //                   selectNodesFromLaunch={(providerId: string, launch: LaunchContent) =>
  //                     selectNodesFromLaunch(providerId, launch)
  //                   }
  //                   onRemoveLaunch={(providerId: string, path: string, masteruri: string) =>
  //                     onRemoveLaunch(providerId, path, masteruri)
  //                   }
  //                   onReloadLaunch={(providerId: string, path: string, masteruri: string) =>
  //                     onReloadLaunch(providerId, path, masteruri)
  //                   }
  //                 />
  //               )}

  //               {item.children.sort(compareTreeItems).map((sortItem) => {
  //                 return buildHostTreeViewItem(item.providerId as string, sortItem, newKeyNodeList);
  //               })}
  //             </HostItem>
  //           );
  //         })}
  //       </SimpleTreeView>
  //     </Box>
  //   );
  //   setKeyNodeList(newKeyNodeList);
  //   return tree;
  // }, [
  //   expanded,
  //   providerNodeTree,
  //   selectedItems,
  //   rosCtx,
  //   settingsCtx.changed,
  //   rosCtx.nodeMap,
  //   // handleToggle, <= causes too many re-renders
  //   // handleSelect, <= causes too many re-renders
  //   // getMasterSyncNode,     <= causes too many re-renders
  //   // handleDoubleClick,     <= causes too many re-renders
  //   // getProviderTags,       <= causes too many re-renders
  //   // selectNodesFromLaunch, <= causes too many re-renders
  //   // onRemoveLaunch,        <= causes too many re-renders
  //   // onReloadLaunch,        <= causes too many re-renders
  //   // toggleMasterSync,      <= causes too many re-renders
  //   // createSingleTerminalCmdPanel,  <= causes too many re-renders
  //   // buildHostTreeViewItem, <= causes too many re-renders
  //   // setKeyNodeList,        <= causes too many re-renders
  // ]);

  // const newKeyNodeList: { key: string; idGlobal: string }[] = [];
  // setKeyNodeList(newKeyNodeList);
  return (
    <Box
      width="100%"
      height="100%"
      overflow="auto"
      onClick={() => {
        // deselect topics
        setSelectedItems([]);
      }}
    >
      {(!rosCtx.providers || rosCtx.providers.length === 0) && (
        <Alert severity="info">
          <AlertTitle>No providers available</AlertTitle>
          Please connect to a ROS provider
        </Alert>
      )}
      <SimpleTreeView
        onClick={(event) => {
          // enable deselection
          event.stopPropagation();
        }}
        // apiRef={apiRef}
        aria-label="node list"
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        multiSelect
        expandedItems={expanded}
        // sx={{ height: '100%' }}
        selectedItems={selectedItems}
        onExpandedItemsChange={(event: React.SyntheticEvent | null, itemIds: string[]) => handleToggle(event, itemIds)}
        onSelectedItemsChange={(event: React.SyntheticEvent | null, itemIds: string[]) => handleSelect(event, itemIds)}
        expansionTrigger={"iconContainer"}
        // selectionPropagation={{ parents: true, descendants: true }}
      >
        {providerNodeTree?.sort(compareTreeProvider).map((item) => {
          let providerIsAvailable = false;
          let p = rosCtx.getProviderById(item.providerId as string, true);
          if (p?.isAvailable()) {
            providerIsAvailable = true;
          }
          if (!p) {
            // no provider was found: no provider found: we have a remote node and the provider daemon is not started there.
            // We create a new "not connected" provider.
            if (item.providerId) {
              p = new Provider(settingsCtx, item.providerId, rosCtx.rosInfo?.version || "2");
              p.id = item.providerId;
            } else {
              return "";
            }
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
                  providerId={p.id}
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
                return buildHostTreeViewItem(item.providerId as string, sortItem);
              })}
            </HostItem>
          );
        })}
      </SimpleTreeView>
    </Box>
  );
}
