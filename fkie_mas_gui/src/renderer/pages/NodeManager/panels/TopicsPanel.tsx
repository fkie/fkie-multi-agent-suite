import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ChatOutlinedIcon from "@mui/icons-material/ChatOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import RefreshIcon from "@mui/icons-material/Refresh";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { Virtuoso } from "react-virtuoso";

import TopicGroupTreeItem from "@/renderer/components/TopicTreeView/TopicGroupTreeItem";
import TopicTreeItem from "@/renderer/components/TopicTreeView/TopicTreeItem";
import LongPressIconButton from "@/renderer/components/UI/LongPressIconButton";
import OverflowMenu from "@/renderer/components/UI/OverflowMenu";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { TopicExtendedInfo } from "@/renderer/models";
import { EVENT_PROVIDER_ROS_TOPICS } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";
import { TContentId } from "../../../components/layout/LayoutTabConfig";
import { EVENT_FILTER_TOPICS, TFilterText } from "../../../components/layout/events";

type TTreeItem = {
  groupKey: string;
  groupName: string;
  topics: TTreeItem[];
  count: number;
  fullPrefix: string;
  msgType: string;
  groupKeys: string[];
  topicInfo: TopicExtendedInfo | null;
  hasIncompatibleQos: boolean;
};

type TTreeResult = {
  topics: TTreeItem[];
  count: number;
  groupKeys: string[];
};

type TProviderDescription = {
  providerId: string;
  providerName: string;
};

interface TopicsPanelProps {
  contentId?: TContentId;
  initialSearchTerm?: string;
}

const EXPAND_ON_SEARCH_MIN_CHARS = 2;

// Selected item is just the topic/group id (no domain tracking needed anymore)
type TSelected = string | null;

// Flat entry for Virtuoso
type FlatRow = {
  id: string;
  type: "group" | "topic";
  depth: number;
  treeItem: TTreeItem;
  rootPath: string;
};

export default function TopicsPanel(props: TopicsPanelProps): JSX.Element {
  const { contentId, initialSearchTerm = "" } = props;
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();

  // Topics for this panel (filtered by contentId)
  const [topics, setTopics] = useState<TopicExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expandedItems, setExpandedItems] = useState<string[]>([]);
  const [selected, setSelected] = useState<TSelected>(null);
  const [topicForSelected, setTopicForSelected] = useState<TopicExtendedInfo | undefined>();
  const [availableProviders, setAvailableProviders] = useState<TProviderDescription[]>([]);
  const [avoidGroupWithOneItem] = useSetting<boolean>("avoidGroupWithOneItem");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [buttonLocation] = useSetting<string>("buttonLocation");

  const genKey = useCallback((items: string[]) => items.join("#"), []);

  const getAvailableProviders = useCallback((): TProviderDescription[] => {
    const providers: TProviderDescription[] = [];
    for (const item of rosCtx.providers) {
      providers.push({
        providerId: item.id,
        providerName: item.name(),
      });
    }
    return providers;
  }, [rosCtx.providers]);

  /**
   * Build the TopicExtendedInfo list for this panel.
   * Filtering is done based on contentId:
   * - contentId.providerId: only topics of this provider
   * - contentId.domainId: topics from all providers in this domain
   */
  const updateTopicList = useCallback(async () => {
    if (!rosCtx.initialized) {
      return;
    }

    // topicKey -> TopicExtendedInfo
    const topicMap = new Map<string, TopicExtendedInfo>();

    const selectedDomainId = contentId?.domainId;
    const selectedProviderId = contentId?.providerId;

    for (const provider of rosCtx.providers) {
      // Filter by providerId if set
      if (selectedProviderId !== undefined) {
        if (provider.id !== selectedProviderId) {
          continue;
        }
      } else if (selectedDomainId !== undefined) {
        // Otherwise filter by domainId if set
        const providerDomainId = provider.connection?.domainId;
        if (providerDomainId !== selectedDomainId) {
          continue;
        }
      }

      for (const topic of provider.rosTopics) {
        // keep original logic: add all rosNodes of provider
        for (const rosNode of provider.rosNodes) {
          const key = genKey([topic.name, topic.msg_type]);
          let topicInfo = topicMap.get(key);

          if (!topicInfo) {
            topicInfo = new TopicExtendedInfo(topic, rosNode);
            topicMap.set(key, topicInfo);
          } else {
            topicInfo.add(rosNode);
          }
        }
      }
    }

    const list = Array.from(topicMap.values()).sort((a, b) => {
      const aSeps = (a.name.match(/\//g) || []).length;
      const bSeps = (b.name.match(/\//g) || []).length;
      if (aSeps === bSeps) {
        return a.name.localeCompare(b.name);
      }
      return bSeps - aSeps;
    });

    setTopics(list);
  }, [rosCtx.initialized, rosCtx.providers, genKey, contentId]);

  const getTopicList = useCallback(() => {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }, [rosCtx.providers]);

  const flattenSingleChildGroups = useCallback((node: TTreeItem): TTreeItem => {
    if (node.topics.length === 1 && !node.topicInfo) {
      const child = node.topics[0];

      node.groupName = `${node.groupName}/${child.groupName}`;
      node.groupKey = `${node.groupKey}-${child.groupKey}`;
      node.topics = child.topics;
      node.count = child.count;
      node.groupKeys = [...node.groupKeys, ...child.groupKeys];
      node.topicInfo = child.topicInfo;
      node.hasIncompatibleQos = node.hasIncompatibleQos || child.hasIncompatibleQos;
      node.msgType = child.msgType || node.msgType;

      if (node.topics.length === 1 && !node.topicInfo) {
        return flattenSingleChildGroups(node);
      }
    } else {
      const nextChildren: TTreeItem[] = [];
      for (let i = 0; i < node.topics.length; i += 1) {
        nextChildren.push(flattenSingleChildGroups(node.topics[i]));
      }
      node.topics = nextChildren;
    }

    return node;
  }, []);

  /**
   * Tree structure builder (same logic as before, just commented in English).
   */
  const buildTree = useCallback(
    (topicsList: TopicExtendedInfo[], avoidSingle: boolean): TTreeResult => {
      // Map of full path ("/foo/bar") to tree node
      const nodes = new Map<string, TTreeItem>();
      const rootNodes: TTreeItem[] = [];

      /**
       * Phase 1: create a node for every path segment and attach TopicExtendedInfo
       * - For topic "/foo/bar/baz", we create nodes for:
       *   "/foo", "/foo/bar", "/foo/bar/baz"
       * - Only the last segment (leaf) holds topicInfo
       */
      for (const topic of topicsList) {
        const parts = topic.name.split("/").filter(Boolean);
        let currentPath = "";

        for (let i = 0; i < parts.length; i += 1) {
          const path = parts.slice(0, i + 1).join("/");
          currentPath = `/${path}`;

          if (!nodes.has(currentPath)) {
            nodes.set(currentPath, {
              groupKey: path.replace(/\//g, "-"),
              groupName: parts[i],
              topics: [],
              count: 0,
              fullPrefix: i > 0 ? `/${parts.slice(0, i).join("/")}` : "",
              msgType: "",
              groupKeys: [],
              topicInfo: null,
              hasIncompatibleQos: false,
            });
          }
        }

        // Attach TopicExtendedInfo to the leaf node
        const leafNode = nodes.get(currentPath);
        if (leafNode) {
          leafNode.topicInfo = topic;
          // Leaf node always represents exactly one topic initially
          leafNode.count = 1;
          leafNode.hasIncompatibleQos = topic.hasIncompatibleQos;
        }
      }

      /**
       * Phase 2: connect nodes to a tree based on their parent paths
       * - Determine parent by stripping the last "/segment" from the path
       * - Root nodes have no valid parent in the map and are pushed to rootNodes
       */
      nodes.forEach((node, path) => {
        const parentPath = path.substring(0, path.lastIndexOf("/"));

        if (parentPath && nodes.has(parentPath)) {
          const parent = nodes.get(parentPath);
          if (parent) {
            parent.topics.push(node);
            parent.groupKeys.push(node.groupKey);

            // Propagate QoS information upwards if this group itself has no topic
            if (!parent.topicInfo) {
              parent.hasIncompatibleQos = parent.hasIncompatibleQos || node.hasIncompatibleQos;
              parent.msgType = node.msgType || parent.msgType;
            }
          }
        } else {
          // No parent => this is a root node
          rootNodes.push(node);
        }
      });

      /**
       * Phase 3: sort root nodes
       * - Groups before leaf topics
       * - Alphabetical by groupName
       */
      rootNodes.sort((a, b) => {
        const aIsGroup = a.topics.length > 0;
        const bIsGroup = b.topics.length > 0;
        if (aIsGroup && !bIsGroup) return -1;
        if (!aIsGroup && bIsGroup) return 1;
        return a.groupName.localeCompare(b.groupName);
      });

      /**
       * Phase 4: optionally flatten groups that contain only a single child
       * - This is only a structural change; counts will be recalculated afterwards
       */
      const processedRoots: TTreeItem[] = [];
      if (avoidSingle) {
        for (const node of rootNodes) {
          processedRoots.push(flattenSingleChildGroups(node));
        }
      } else {
        processedRoots.push(...rootNodes);
      }

      /**
       * Phase 5: recursively compute counts for all nodes
       * - Leaf node (topicInfo != null) has count = 1
       * - Group node has count = sum of all leaf topics in its subtree
       * - This guarantees that count includes all topics in subgroups
       */
      const computeCounts = (node: TTreeItem): number => {
        // Leaf topic
        if (node.topicInfo) {
          node.count = 1;
          return 1;
        }

        let sum = 0;
        for (const child of node.topics) {
          sum += computeCounts(child);
        }
        node.count = sum;
        return sum;
      };

      for (const root of processedRoots) {
        computeCounts(root);
      }

      return { topics: processedRoots, count: processedRoots.length, groupKeys: [] };
    },
    [flattenSingleChildGroups]
  );

  /**
   * All topics filtered by search term.
   */
  const filteredTopics = useMemo(() => {
    if (!searchTerm.trim()) return topics;
    return topics.filter((topic) =>
      findIn(searchTerm, [
        topic.name,
        topic.msgType,
        ...topic.publishers.map((p) => `${p.info.node_id} ${p.providerName}`),
        ...topic.subscribers.map((s) => `${s.info.node_id} ${s.providerName}`),
      ])
    );
  }, [topics, searchTerm]);

  const treeData = useMemo(() => {
    const treeResult = buildTree(
      filteredTopics,
      searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? avoidGroupWithOneItem : false
    );
    return treeResult.topics;
  }, [filteredTopics, avoidGroupWithOneItem, searchTerm.length, buildTree]);

  useEffect(() => {
    setAvailableProviders(getAvailableProviders());
  }, [rosCtx.providers, getAvailableProviders]);

  useEffect(() => {
    updateTopicList();
  }, [rosCtx.mapProviderRosNodes, updateTopicList]);

  useEffect(() => {
    updateTopicList();
  }, []); // initial

  useEffect(() => {
    setRootDataList(treeData);
  }, [treeData]);

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, updateTopicList);
  useCustomEventListener(EVENT_FILTER_TOPICS, (filter: TFilterText) => setSearchTerm(filter.data));

  const onEchoClick = useCallback(
    (topic: TopicExtendedInfo | undefined, external: boolean, openInTerminal = false, providerId = "") => {
      if (!topic) return;

      let provId = providerId;
      if (!provId) {
        if (topic.publishers.length > 0) {
          provId = topic.publishers[0].providerId;
        } else if (topic.subscribers.length > 0) {
          provId = topic.subscribers[0].providerId;
        }
      }

      if (provId) {
        navCtx.openSubscriber(provId, topic.name, true, false, external, openInTerminal);
      } else {
        logCtx.warn("no publisher available");
      }
    },
    [navCtx, logCtx]
  );

  const onPublishClick = useCallback(
    (topic: TopicExtendedInfo | undefined, external: boolean, openInTerminal = false) => {
      if (!topic) return;

      let provId = "";
      if (topic.subscribers.length > 0) {
        provId = topic.subscribers[0].providerId;
      } else if (topic.publishers.length > 0) {
        provId = topic.publishers[0].providerId;
      }

      navCtx.startPublisher(provId, topic.name, topic.msgType, external, openInTerminal);
    },
    [navCtx]
  );

  const onSearch = useCallback((term: string) => {
    setSearchTerm(term);
  }, []);

  const buttonBox = useMemo(
    () => (
      <ButtonGroup orientation="vertical" aria-label="topic control group">
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Echo
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
              disabled={!topicForSelected}
              size="medium"
              onClick={(e) =>
                onEchoClick(topicForSelected, e.nativeEvent.shiftKey as boolean, e.nativeEvent.ctrlKey as boolean)
              }
              onLongPress={() => {
                onEchoClick(topicForSelected, true, false);
              }}
            >
              <ChatBubbleOutlineIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>

        <OverflowMenu
          disabled={!topicForSelected}
          icon={
            <Tooltip
              title={
                <div>
                  <Typography fontWeight="bold" fontSize="inherit">
                    Echo with provider selection
                  </Typography>
                  <Stack direction="row" spacing={"0.2em"}>
                    <Typography fontWeight="bold" fontSize="inherit">
                      Shift:
                    </Typography>
                    <Typography fontSize="inherit">alternative open location</Typography>
                  </Stack>
                </div>
              }
              placement="left"
              disableInteractive
            >
              <ChatOutlinedIcon fontSize="inherit" />
            </Tooltip>
          }
          size="medium"
          sx={{ margin: 0 }}
          options={availableProviders.map((provider) => ({
            name: provider.providerName,
            key: provider.providerId,
            onClick: (e?: React.MouseEvent) =>
              onEchoClick(topicForSelected, !!e?.nativeEvent.shiftKey, !!e?.nativeEvent.ctrlKey, provider.providerId),
          }))}
          id={`echo-provider-menu-${topicForSelected?.name || ""}`}
        />

        <Tooltip title="Echo in Terminal" placement="left" disableInteractive>
          <span>
            <IconButton
              disabled={!topicForSelected}
              size="medium"
              onClick={(e) => onEchoClick(topicForSelected, e.nativeEvent.shiftKey as boolean, true)}
            >
              <DvrIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>

        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Create a publisher
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
              size="medium"
              onClick={(e) =>
                onPublishClick(topicForSelected, e.nativeEvent.shiftKey as boolean, e.nativeEvent.ctrlKey as boolean)
              }
              onLongPress={() => {
                onPublishClick(topicForSelected, true, false);
              }}
            >
              <PlayCircleOutlineIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>
      </ButtonGroup>
    ),
    [topicForSelected, availableProviders, onEchoClick, onPublishClick]
  );

  const reloadButton = useMemo(
    () => (
      <Tooltip title="Reload topic list" placement="left" disableInteractive>
        <IconButton size="small" onClick={getTopicList}>
          <RefreshIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    ),
    [getTopicList]
  );

  // expand/collapse for groups
  const toggleExpanded = useCallback((id: string) => {
    setExpandedItems((prev) => (prev.includes(id) ? prev.filter((x) => x !== id) : [...prev, id]));
  }, []);

  // Resolve the currently selected topic (if any)
  useEffect(() => {
    if (!selected) {
      setTopicForSelected(undefined);
      return;
    }
    const t = topics.find((item) => genKey([item.name, item.msgType]) === selected);
    setTopicForSelected(t);
  }, [selected, topics, genKey]);

  // Selection handler for groups and topics
  const handleSelect = useCallback((itemId: string) => {
    setSelected(itemId);
  }, []);

  // Flat rows for Virtuoso
  const flatRows = useMemo<FlatRow[]>(() => {
    const expandedSet = new Set(expandedItems);
    const rows: FlatRow[] = [];

    const walk = (node: TTreeItem, depth: number, rootPath: string) => {
      // leaf topic
      if (node.topicInfo) {
        rows.push({
          id: genKey([node.topicInfo.name, node.topicInfo.msgType]),
          type: "topic",
          depth,
          treeItem: node,
          rootPath,
        });
        return;
      }

      // optionally flatten groups with a single child
      if (avoidGroupWithOneItem && node.topics.length === 1) {
        const nextRoot = rootPath ? `${rootPath}/${node.groupName}` : node.groupName;
        walk(node.topics[0], depth, nextRoot);
        return;
      }

      // group entry
      rows.push({
        id: node.groupKey,
        type: "group",
        depth,
        treeItem: node,
        rootPath,
      });

      // add children only if group is expanded
      if (expandedSet.has(node.groupKey)) {
        const sortedChildren = [...node.topics].sort((a, b) => {
          const aIsGroup = !a.topicInfo;
          const bIsGroup = !b.topicInfo;
          if (aIsGroup && !bIsGroup) return -1;
          if (!aIsGroup && bIsGroup) return 1;
          return a.groupName.localeCompare(b.groupName);
        });

        for (let i = 0; i < sortedChildren.length; i += 1) {
          const child = sortedChildren[i];
          walk(child, depth + 1, "");
        }
      }
    };

    // sort root level
    const sortedRoots = [...rootDataList].sort((a, b) => {
      const aIsGroup = !a.topicInfo;
      const bIsGroup = !b.topicInfo;
      if (aIsGroup && !bIsGroup) return -1;
      if (!aIsGroup && bIsGroup) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    for (let i = 0; i < sortedRoots.length; i += 1) {
      walk(sortedRoots[i], 0, "");
    }

    return rows;
  }, [rootDataList, expandedItems, avoidGroupWithOneItem, genKey]);

  const treeView = useMemo(
    () => (
      <Virtuoso
        style={{ height: "100%" }}
        totalCount={flatRows.length}
        itemContent={(index) => {
          const row = flatRows[index];

          if (row.type === "group") {
            const node = row.treeItem;
            const isSelected = selected === row.id;

            return (
              <TopicGroupTreeItem
                key={row.id}
                itemId={row.id}
                rootPath={row.rootPath}
                groupName={node.groupName}
                countChildren={node.count}
                hasIncompatibleQos={node.hasIncompatibleQos}
                depth={row.depth}
                expanded={expandedItems.includes(row.id)}
                selected={isSelected}
                onToggle={() => toggleExpanded(row.id)}
                onSelect={() => handleSelect(row.id)}
              />
            );
          }

          const topicInfo = row.treeItem.topicInfo;
          if (!topicInfo) return null;
          const id = row.id;
          const isSelected = selected === id;

          return (
            <TopicTreeItem
              key={id}
              itemId={id}
              rootPath={row.rootPath}
              topicInfo={topicInfo}
              selectedItem={selected ?? ""} // light highlight by id
              selected={isSelected}
              depth={row.depth}
              onSelect={() => handleSelect(id)}
            />
          );
        }}
      />
    ),
    [flatRows, selected, expandedItems, toggleExpanded, handleSelect]
  );

  return (
    <Box height="100%" overflow="hidden" sx={{ backgroundColor: backgroundColor }}>
      <Stack spacing={1} height="100%">
        <Stack direction="row" spacing={0.5} alignItems="center">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && reloadButton}
          <SearchBar
            onSearch={onSearch}
            placeholder="Filter Topics (OR: <space>, AND: +, NOT: !)"
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

          {/* Click on empty area deselects the current item */}
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
