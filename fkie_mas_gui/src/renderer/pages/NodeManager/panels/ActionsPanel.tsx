import RefreshIcon from "@mui/icons-material/Refresh";
import StartIcon from "@mui/icons-material/Start";
import TroubleshootIcon from "@mui/icons-material/Troubleshoot";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { Virtuoso } from "react-virtuoso";

import ActionGroupTreeItem from "@/renderer/components/ActionTreeView/ActionGroupTreeItem";
import ActionTreeItem, { ActionInfo, ActionNodeInfo } from "@/renderer/components/ActionTreeView/ActionTreeItem";
import LongPressIconButton from "@/renderer/components/UI/LongPressIconButton";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";
import { TContentId } from "../../../components/layout/LayoutTabConfig";

type TTreeItem = {
  groupKey: string;
  groupName: string;
  children: TTreeItem[];
  count: number;
  fullPrefix: string;
  actionType: string;
  groupKeys: string[];
  actionInfo: ActionInfo | null;
};

type FlatRow = {
  id: string;
  type: "group" | "action";
  depth: number;
  treeItem: TTreeItem;
  rootPath: string;
};

interface ActionsPanelProps {
  contentId?: TContentId;
  initialSearchTerm?: string;
}

const EXPAND_ON_SEARCH_MIN_CHARS = 2;
const ACTION_SUFFIX_SEND_GOAL = "/_action/send_goal";
const ACTION_SUFFIX_GET_RESULT = "/_action/get_result";

export default function ActionsPanel({ contentId, initialSearchTerm = "" }: ActionsPanelProps): JSX.Element {
  const rosCtx = useRosContext();
  const navCtx = useNavigationContext();

  const [actions, setActions] = useState<ActionInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expandedItems, setExpandedItems] = useState<string[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [selectedAction, setSelectedAction] = useState<ActionInfo | undefined>();

  const [avoidGroupWithOneItem] = useSetting<boolean>("avoidGroupWithOneItem");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [buttonLocation] = useSetting<string>("buttonLocation");

  const genKey = useCallback((items: string[]): string => items.join("#"), []);

  /**
   * Collect action provider nodes from the send_goal service provider ids.
   * Local nodes are preferred over remote nodes, following the same idea as services.
   */
  const collectActionProviderNodes = useCallback(
    (provider: (typeof rosCtx.providers)[number], serviceProviderIds: string[] | undefined): ActionNodeInfo[] => {
      const nodeProviders: ActionNodeInfo[] = [];

      for (const rosNode of provider.rosNodes) {
        for (const providerNodeId of serviceProviderIds || []) {
          if (providerNodeId !== rosNode.id) {
            continue;
          }

          if (rosNode.isLocal) {
            // If a local node is found, keep only local nodes from this point on
            const localOnly = nodeProviders.filter((item) => item.isLocal);
            nodeProviders.length = 0;
            nodeProviders.push(...localOnly);
          }

          const hasNode = nodeProviders.some((item) => item.nodeId === rosNode.id);
          if (!hasNode || rosNode.isLocal) {
            nodeProviders.push({
              nodeName: rosNode.name,
              nodeId: rosNode.id,
              isLocal: rosNode.isLocal,
              providerId: rosNode.providerId,
              providerName: rosNode.providerName,
            });
          }
        }
      }

      return nodeProviders;
    },
    [rosCtx.providers]
  );

  /**
   * Discover ROS 2 actions by detecting /_action/send_goal services.
   * Goal, result and feedback type names are derived from naming conventions.
   * Provider nodes are resolved from the service provider node ids.
   */
  const updateActionList = useCallback(async () => {
    if (!rosCtx.initialized) return;

    const actionMap = new Map<string, ActionInfo>();

    const selectedDomainId = contentId?.domainId;
    const selectedProviderId = contentId?.providerId;

    for (const provider of rosCtx.providers) {
      if (selectedProviderId !== undefined) {
        if (provider.id !== selectedProviderId) continue;
      } else if (selectedDomainId !== undefined) {
        const providerDomainId = provider.connection?.domainId;
        if (providerDomainId !== selectedDomainId) continue;
      }

      // Build a name -> type lookup for companion action services
      const serviceTypeMap = new Map<string, string>();
      for (const service of provider.rosServices) {
        serviceTypeMap.set(service.name, service.srv_type);
      }

      for (const service of provider.rosServices) {
        if (!service.name.endsWith(ACTION_SUFFIX_SEND_GOAL)) continue;

        const actionName = service.name.slice(0, -ACTION_SUFFIX_SEND_GOAL.length);
        if (actionMap.has(actionName)) continue;

        // Example:
        // "nav2_msgs/action/NavigateToPose_SendGoal" -> "nav2_msgs/action/NavigateToPose"
        let actionType = service.srv_type;
        const goalType = service.srv_type;
        if (actionType.endsWith("_SendGoal")) {
          actionType = actionType.slice(0, -"_SendGoal".length);
        }

        const getResultName = `${actionName}${ACTION_SUFFIX_GET_RESULT}`;
        const resultType = serviceTypeMap.get(getResultName) || `${actionType}_GetResult`;

        // Feedback is not a service type, but follows the ROS 2 naming convention
        const feedbackType = `${actionType}_FeedbackMessage`;

        const nodeProviders = collectActionProviderNodes(provider, service.provider);

        actionMap.set(actionName, {
          name: actionName,
          actionType,
          goalType,
          resultType,
          feedbackType,
          providerId: provider.id,
          nodeProviders,
        });
      }
    }

    const list = Array.from(actionMap.values()).sort((a, b) => a.name.localeCompare(b.name));
    setActions(list);
  }, [rosCtx.initialized, rosCtx.providers, contentId, collectActionProviderNodes]);

  const getActionList = useCallback(() => {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }, [rosCtx.providers]);

  const flattenSingleChildGroups = useCallback((node: TTreeItem): TTreeItem => {
    if (node.children.length === 1 && !node.actionInfo) {
      const child = node.children[0];
      node.groupName = `${node.groupName}/${child.groupName}`;
      node.groupKey = `${node.groupKey}-${child.groupKey}`;
      node.children = child.children;
      node.count = child.count;
      node.groupKeys = [...node.groupKeys, ...child.groupKeys];
      node.actionInfo = child.actionInfo;
      node.actionType = child.actionType || node.actionType;

      if (node.children.length === 1 && !node.actionInfo) {
        return flattenSingleChildGroups(node);
      }
    } else {
      node.children = node.children.map((child) => flattenSingleChildGroups(child));
    }
    return node;
  }, []);

  const buildTree = useCallback(
    (actionList: ActionInfo[], avoidSingle: boolean): TTreeItem[] => {
      const nodes = new Map<string, TTreeItem>();
      const rootNodes: TTreeItem[] = [];

      for (const action of actionList) {
        const parts = action.name.split("/").filter(Boolean);
        let currentPath = "";

        for (let i = 0; i < parts.length; i += 1) {
          const path = parts.slice(0, i + 1).join("/");
          currentPath = `/${path}`;

          if (!nodes.has(currentPath)) {
            nodes.set(currentPath, {
              groupKey: path.replace(/\//g, "-"),
              groupName: parts[i],
              children: [],
              count: 0,
              fullPrefix: i > 0 ? `/${parts.slice(0, i).join("/")}` : "",
              actionType: "",
              groupKeys: [],
              actionInfo: null,
            });
          }
        }

        const leafNode = nodes.get(currentPath);
        if (leafNode) {
          leafNode.actionInfo = action;
          leafNode.count = 1;
          leafNode.actionType = action.actionType;
        }
      }

      for (const [path, node] of nodes.entries()) {
        const parentPath = path.substring(0, path.lastIndexOf("/"));
        if (parentPath && nodes.has(parentPath)) {
          const parent = nodes.get(parentPath);
          if (!parent) continue;
          parent.children.push(node);
          parent.groupKeys.push(node.groupKey);
          if (!parent.actionInfo) {
            parent.actionType = node.actionType || parent.actionType;
          }
        } else {
          rootNodes.push(node);
        }
      }

      rootNodes.sort((a, b) => a.groupName.localeCompare(b.groupName));

      let processedRoots: TTreeItem[];
      if (avoidSingle) {
        processedRoots = rootNodes.map((n) => flattenSingleChildGroups(n));
      } else {
        processedRoots = rootNodes;
      }

      const computeCounts = (node: TTreeItem): number => {
        if (node.actionInfo) {
          node.count = 1;
          return 1;
        }
        let sum = 0;
        for (const child of node.children) {
          sum += computeCounts(child);
        }
        node.count = sum;
        return sum;
      };

      for (const root of processedRoots) {
        computeCounts(root);
      }

      return processedRoots;
    },
    [flattenSingleChildGroups]
  );

  const filteredActions = useMemo(() => {
    if (!searchTerm.trim()) return actions;
    return actions.filter((action) =>
      findIn(searchTerm, [
        action.name,
        action.actionType,
        ...action.nodeProviders.map((node) => node.nodeName),
        ...action.nodeProviders.map((node) => node.providerName),
      ])
    );
  }, [actions, searchTerm]);

  const treeData = useMemo(() => {
    return buildTree(filteredActions, searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? avoidGroupWithOneItem : false);
  }, [filteredActions, avoidGroupWithOneItem, searchTerm.length, buildTree]);

  useEffect(() => {
    updateActionList();
  }, []);

  useEffect(() => {
    updateActionList();
  }, [rosCtx.mapProviderRosNodes, updateActionList]);

  useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, updateActionList);

  useEffect(() => {
    setRootDataList(treeData);
  }, [treeData]);

  // Expand all groups while searching
  useEffect(() => {
    if (searchTerm.length >= EXPAND_ON_SEARCH_MIN_CHARS) {
      const allGroupKeys: string[] = [];
      const collectKeys = (node: TTreeItem) => {
        if (!node.actionInfo && node.children.length > 0) {
          allGroupKeys.push(node.groupKey);
          node.children.forEach(collectKeys);
        }
      };
      rootDataList.forEach(collectKeys);
      setExpandedItems(allGroupKeys);
    }
  }, [searchTerm, rootDataList]);

  const onSearch = useCallback((term: string) => {
    setSearchTerm(term);
  }, []);

  const toggleExpanded = useCallback((id: string) => {
    setExpandedItems((prev) => (prev.includes(id) ? prev.filter((x) => x !== id) : [...prev, id]));
  }, []);

  const handleSelect = useCallback((itemId: string) => {
    setSelected(itemId);
  }, []);

  useEffect(() => {
    if (!selected) {
      setSelectedAction(undefined);
      return;
    }
    const found = actions.find((a) => genKey([a.name, a.actionType]) === selected);
    setSelectedAction(found);
  }, [selected, actions, genKey]);

  const flatRows = useMemo<FlatRow[]>(() => {
    const expandedSet = new Set(expandedItems);
    const rows: FlatRow[] = [];

    const walk = (node: TTreeItem, depth: number, rootPath: string) => {
      if (node.actionInfo) {
        rows.push({
          id: genKey([node.actionInfo.name, node.actionInfo.actionType]),
          type: "action",
          depth,
          treeItem: node,
          rootPath,
        });
        return;
      }

      if (avoidGroupWithOneItem && node.children.length === 1) {
        const nextRoot = rootPath ? `${rootPath}/${node.groupName}` : node.groupName;
        walk(node.children[0], depth, nextRoot);
        return;
      }

      rows.push({
        id: node.groupKey,
        type: "group",
        depth,
        treeItem: node,
        rootPath,
      });

      if (expandedSet.has(node.groupKey)) {
        const sortedChildren = [...node.children].sort((a, b) => {
          const aIsGroup = !a.actionInfo;
          const bIsGroup = !b.actionInfo;
          if (aIsGroup && !bIsGroup) return -1;
          if (!aIsGroup && bIsGroup) return 1;
          return a.groupName.localeCompare(b.groupName);
        });

        for (const child of sortedChildren) {
          walk(child, depth + 1, "");
        }
      }
    };

    const sortedRoots = [...rootDataList].sort((a, b) => {
      const aIsGroup = !a.actionInfo;
      const bIsGroup = !b.actionInfo;
      if (aIsGroup && !bIsGroup) return -1;
      if (!aIsGroup && bIsGroup) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    for (const root of sortedRoots) {
      walk(root, 0, "");
    }

    return rows;
  }, [rootDataList, expandedItems, avoidGroupWithOneItem, genKey]);

  const onCallAction = useCallback(
    (action: ActionInfo | undefined, external: boolean, openInTerminal = false) => {
      if (!action) return;
      console.debug(
        `open dialog for action goal: ${action.name} [${action.actionType}]; external=${external} terminal=${openInTerminal}`
      );
      navCtx.openActionSendGoal({
        providerId: action.providerId || "",
        serviceName: action.name,
        serviceType: action.actionType,
        externalKeyModifier: external,
        forceOpenTerminal: openInTerminal,
      });
    },
    [navCtx.openActionSendGoal]
  );

  const onIntrospectAction = useCallback(
    (action: ActionInfo | undefined, external: boolean, openInTerminal = false) => {
      if (!action) return;
      console.debug(
        `open dialog for introspect action: ${action.name} [${action.actionType}]; external=${external} terminal=${openInTerminal}`
      );

      navCtx.openActionIntrospection({
        providerId: action.providerId || "",
        serviceName: action.name,
        serviceType: action.actionType,
        externalKeyModifier: external,
        forceOpenTerminal: openInTerminal,
      });
    },
    [navCtx.openActionIntrospection]
  );

  const buttonBox = useMemo(
    () => (
      <ButtonGroup orientation="vertical" aria-label="action control group">
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Send Goal
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight="bold" fontSize="inherit">
                  Shift or long press:
                </Typography>
                <Typography fontSize="inherit">alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <LongPressIconButton
              disabled={!selectedAction}
              size="medium"
              aria-label="Send Goal"
              onClick={(event) => onCallAction(selectedAction, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey)}
              onLongPress={() => {
                onCallAction(selectedAction, true, false);
              }}
            >
              <StartIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Introspect action
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight="bold" fontSize="inherit">
                  Shift or long press:
                </Typography>
                <Typography fontSize="inherit">alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <LongPressIconButton
              disabled={!selectedAction}
              size="medium"
              aria-label="introspect"
              onClick={(event) =>
                onIntrospectAction(selectedAction, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey)
              }
              onLongPress={() => {
                onIntrospectAction(selectedAction, true, false);
              }}
            >
              <TroubleshootIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>
      </ButtonGroup>
    ),
    [selectedAction, onCallAction, onIntrospectAction]
  );

  const reloadButton = useMemo(
    () => (
      <Tooltip title="Reload action list" placement="left" disableInteractive>
        <IconButton size="small" onClick={getActionList}>
          <RefreshIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    ),
    [getActionList]
  );

  const treeView = useMemo(
    () => (
      <Virtuoso
        style={{ height: "100%" }}
        totalCount={flatRows.length}
        itemContent={(index: number) => {
          const row = flatRows[index];

          if (row.type === "group") {
            return (
              <ActionGroupTreeItem
                key={row.id}
                itemId={row.id}
                rootPath={row.rootPath}
                groupName={row.treeItem.groupName}
                countChildren={row.treeItem.count}
                expanded={expandedItems.includes(row.id)}
                selected={selected === row.id}
                depth={row.depth}
                onToggle={() => toggleExpanded(row.id)}
                onSelect={() => handleSelect(row.id)}
              />
            );
          }

          const actionInfo = row.treeItem.actionInfo;
          if (!actionInfo) return null;

          return (
            <ActionTreeItem
              key={row.id}
              itemId={row.id}
              rootPath={row.rootPath}
              actionInfo={actionInfo}
              selectedItem={selected ?? ""}
              selected={selected === row.id}
              depth={row.depth}
              onSelect={() => handleSelect(row.id)}
            />
          );
        }}
      />
    ),
    [flatRows, selected, expandedItems, toggleExpanded, handleSelect]
  );

  return (
    <Box height="100%" overflow="hidden" sx={{ backgroundColor }}>
      <Stack spacing={1} height="100%">
        <Stack direction="row" spacing={0.5} alignItems="center">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && reloadButton}
          <SearchBar
            onSearch={onSearch}
            placeholder="Filter Actions (OR: <space>, AND: +, NOT: !)"
            defaultValue={searchTerm}
            fullWidth
          />
          {buttonLocation === BUTTON_LOCATIONS.RIGHT && reloadButton}
        </Stack>

        <Stack direction="row" height="100%" overflow="hidden">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}

          <Box width="100%" height="100%" overflow="hidden" onClick={() => setSelected(null)}>
            {treeView}
          </Box>

          {buttonLocation === BUTTON_LOCATIONS.RIGHT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}
        </Stack>
      </Stack>
    </Box>
  );
}
