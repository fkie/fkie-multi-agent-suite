import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ChatOutlinedIcon from "@mui/icons-material/ChatOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import RefreshIcon from "@mui/icons-material/Refresh";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { grey } from "@mui/material/colors";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { Virtuoso } from "react-virtuoso";

import { DomainFlexLayout } from "@/renderer/components/layout/DomainFlexLayout";
import TopicGroupTreeItem from "@/renderer/components/TopicTreeView/TopicGroupTreeItem";
import TopicTreeItem from "@/renderer/components/TopicTreeView/TopicTreeItem";
import OverflowMenu from "@/renderer/components/UI/OverflowMenu";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { TopicExtendedInfo } from "@/renderer/models";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_PROVIDER_ROS_TOPICS } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";
import { EVENT_FILTER_TOPICS, TFilterText } from "../layout/events";

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

type TSettings = {
  tooltipDelay: number;
  avoidGroupWithOneItem: boolean;
  backgroundColor: string;
  buttonLocation: string;
};

type TProviderDescription = {
  providerId: string;
  providerName: string;
};

interface TopicsPanelProps {
  initialSearchTerm?: string;
}

const EXPAND_ON_SEARCH_MIN_CHARS = 2;

type TSelected = { id: string; domainId: string } | null;

// Flat entry for Virtuoso
type FlatRow = {
  id: string;
  type: "group" | "topic";
  depth: number;
  treeItem: TTreeItem;
  rootPath: string;
};

export default function TopicsPanel({ initialSearchTerm = "" }: TopicsPanelProps): JSX.Element {
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  // topics grouped by domainId
  const [topicsByDomain, setTopicsByDomain] = useState<Record<string, TopicExtendedInfo[]>>({});
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expandedItems, setExpandedItems] = useState<string[]>([]);
  const [selected, setSelected] = useState<TSelected>(null);
  const [topicForSelected, setTopicForSelected] = useState<TopicExtendedInfo | undefined>();
  const [availableProviders, setAvailableProviders] = useState<TProviderDescription[]>([]);
  const [settings, setSettings] = useState<TSettings>({
    tooltipDelay: settingsCtx.get("tooltipEnterDelay") as number,
    avoidGroupWithOneItem: settingsCtx.get("avoidGroupWithOneItem") as boolean,
    backgroundColor: settingsCtx.get("backgroundColor") as string,
    buttonLocation: settingsCtx.get("buttonLocation") as string,
  });

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
   * Build per-domain TopicExtendedInfo lists.
   * For each domainId we collect topics from all providers in that domain.

   */
  const updateTopicList = useCallback(async () => {
    if (!rosCtx.initialized) {
      return;
    }

    // domainId -> (topicKey -> TopicExtendedInfo)
    const domainTopicMaps = new Map<string, Map<string, TopicExtendedInfo>>();

    for (const provider of rosCtx.providers) {
      const rawDomainId = provider.connection?.domainId ?? "default";
      const domainId = String(rawDomainId);

      let topicMap = domainTopicMaps.get(domainId);
      if (!topicMap) {
        topicMap = new Map<string, TopicExtendedInfo>();
        domainTopicMaps.set(domainId, topicMap);
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

    const nextTopicsByDomain: Record<string, TopicExtendedInfo[]> = {};
    const domainIdsFromMap = Array.from(domainTopicMaps.keys());

    for (let i = 0; i < domainIdsFromMap.length; i += 1) {
      const domainId = domainIdsFromMap[i];
      const topicMap = domainTopicMaps.get(domainId);
      if (!topicMap) {
        continue;
      }
      const list = Array.from(topicMap.values()).sort((a, b) => {
        const aSeps = (a.name.match(/\//g) || []).length;
        const bSeps = (b.name.match(/\//g) || []).length;
        if (aSeps === bSeps) {
          return a.name.localeCompare(b.name);
        }
        return bSeps - aSeps;
      });
      nextTopicsByDomain[domainId] = list;
    }

    setTopicsByDomain(nextTopicsByDomain);
  }, [rosCtx.initialized, rosCtx.providers, genKey]);

  const getTopicList = useCallback(() => {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }, [rosCtx.providers]);

  /**
   * Tree structure builder (same logic as before, just commented in English).

   */
  const buildTree = useCallback((topicsList: TopicExtendedInfo[], avoidSingle: boolean): TTreeResult => {
    const nodes = new Map<string, TTreeItem>();
    const rootNodes: TTreeItem[] = [];

    // Phase 1: create nodes for all path segments
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

      const leafNode = nodes.get(currentPath);
      if (leafNode) {
        leafNode.topicInfo = topic;
        leafNode.count = 1;
        leafNode.hasIncompatibleQos = topic.hasIncompatibleQos;
      }
    }

    // Phase 2: connect nodes and propagate QoS flags upwards
    nodes.forEach((node, path) => {
      const parentPath = path.substring(0, path.lastIndexOf("/"));

      if (parentPath && nodes.has(parentPath)) {
        const parent = nodes.get(parentPath);
        if (parent) {
          parent.topics.push(node);
          parent.count += node.count;
          parent.groupKeys.push(node.groupKey);

          if (!parent.topicInfo) {
            parent.hasIncompatibleQos = parent.hasIncompatibleQos || node.hasIncompatibleQos;
            parent.msgType = node.msgType || parent.msgType;
          }
        }
      } else {
        rootNodes.push(node);
      }
    });

    // Sort: groups before leaf topics, then alphabetically
    rootNodes.sort((a, b) => {
      const aIsGroup = a.topics.length > 0;
      const bIsGroup = b.topics.length > 0;
      if (aIsGroup && !bIsGroup) return -1;
      if (!aIsGroup && bIsGroup) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    // Optionally flatten groups with a single child
    if (avoidSingle) {
      const flattenedRoots: TTreeItem[] = [];
      for (const node of rootNodes) {
        const flattened = flattenSingleChildGroups(node);
        flattenedRoots.push(flattened);
      }
      return { topics: flattenedRoots, count: flattenedRoots.length, groupKeys: [] };
    }

    return { topics: rootNodes, count: rootNodes.length, groupKeys: [] };
  }, []);

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
   * All topics across all domains (used for filtering and selection).

   */
  const allTopics = useMemo(() => {
    const result: TopicExtendedInfo[] = [];
    const domainIds = Object.keys(topicsByDomain);
    for (let i = 0; i < domainIds.length; i += 1) {
      const list = topicsByDomain[domainIds[i]] || [];
      for (let j = 0; j < list.length; j += 1) {
        result.push(list[j]);
      }
    }
    return result;
  }, [topicsByDomain]);

  const filteredTopics = useMemo(() => {
    if (!searchTerm.trim()) return allTopics;
    return allTopics.filter((topic) =>
      findIn(searchTerm, [
        topic.name,
        topic.msgType,
        ...topic.publishers.map((p) => `${p.info.node_id} ${p.providerName}`),
        ...topic.subscribers.map((s) => `${s.info.node_id} ${s.providerName}`),
      ])
    );
  }, [allTopics, searchTerm]);

  const treeData = useMemo(() => {
    const treeResult = buildTree(
      filteredTopics,
      searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? settings.avoidGroupWithOneItem : false
    );
    return treeResult.topics;
  }, [filteredTopics, settings.avoidGroupWithOneItem, searchTerm.length, buildTree]);

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
    setSettings({
      tooltipDelay: settingsCtx.get("tooltipEnterDelay") as number,
      avoidGroupWithOneItem: settingsCtx.get("avoidGroupWithOneItem") as boolean,
      backgroundColor: settingsCtx.get("backgroundColor") as string,
      buttonLocation: settingsCtx.get("buttonLocation") as string,
    });
  }, [settingsCtx.changed]);

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
          title="Echo (shift+click for alternative open location)"
          placement="left"
          enterDelay={settings.tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!topicForSelected}
              size="medium"
              onClick={(e) =>
                onEchoClick(topicForSelected, e.nativeEvent.shiftKey as boolean, e.nativeEvent.ctrlKey as boolean)
              }
            >
              <ChatBubbleOutlineIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>

        <OverflowMenu
          disabled={!topicForSelected}
          icon={
            <Tooltip
              title="Echo with provider selection (shift+click for alternative)"
              placement="left"
              enterDelay={settings.tooltipDelay}
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

        <Tooltip title="Echo in Terminal" placement="left" enterDelay={settings.tooltipDelay} disableInteractive>
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

        <Tooltip title="Create a publisher" placement="left" enterDelay={settings.tooltipDelay} disableInteractive>
          <IconButton
            size="medium"
            onClick={(e) =>
              onPublishClick(topicForSelected, e.nativeEvent.shiftKey as boolean, e.nativeEvent.ctrlKey as boolean)
            }
          >
            <PlayCircleOutlineIcon fontSize="inherit" />
          </IconButton>
        </Tooltip>
      </ButtonGroup>
    ),
    [topicForSelected, availableProviders, settings.tooltipDelay, onEchoClick, onPublishClick]
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

  useEffect(() => {
    if (!selected) {
      setTopicForSelected(undefined);
      return;
    }
    const domainTopics = topicsByDomain[selected.domainId] ?? [];
    const t = domainTopics.find((item) => genKey([item.name, item.msgType]) === selected.id);
    setTopicForSelected(t);
  }, [selected, topicsByDomain, genKey]);

  // selection
  // we keep track of which domain the selection belongs to
  const handleSelect = useCallback((itemId: string, domainId: string) => {
    setSelected({ id: itemId, domainId });
  }, []);

  // Map provider -> domain
  const providerDomainMap = useMemo(() => {
    const m = new Map<string, string>();
    for (const p of rosCtx.providers) {
      const raw = p.connection?.domainId ?? "default";
      m.set(p.id, String(raw));
    }
    return m;
  }, [rosCtx.providers]);

  // list of domainIds
  const domainIds = useMemo(() => {
    const set = new Set<string>();
    for (const domainId of providerDomainMap.values()) {
      set.add(domainId);
    }
    return Array.from(set.values());
  }, [providerDomainMap]);

  // eine sinnvolle Domain-ID für den Single-Domain-Fall
  const singleDomainId = domainIds.length === 1 ? domainIds[0] : "default";

  // flat rows for single-domain case (same as before, but based on rootDataList)
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
      if (settings.avoidGroupWithOneItem && node.topics.length === 1) {
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
  }, [rootDataList, expandedItems, settings.avoidGroupWithOneItem, genKey]);

  /**
   * Tree view:
   * - Single-domain: one Virtuoso (same behavior as before).
   * - Multi-domain: DomainFlexLayout with one Virtuoso per domain.
   */
  const treeView = useMemo(
    () =>
      domainIds.length <= 1 ? (
        <Virtuoso
          style={{ height: "100%" }}
          totalCount={flatRows.length}
          itemContent={(index) => {
            const row = flatRows[index];

            if (row.type === "group") {
              const node = row.treeItem;
              const isSelected = selected?.id === row.id && selected?.domainId === singleDomainId;

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
                  onSelect={() => handleSelect(row.id, singleDomainId)}
                />
              );
            }

            const topicInfo = row.treeItem.topicInfo;
            if (!topicInfo) return null;
            const id = row.id;
            const isSelected = selected?.id === id && selected?.domainId === singleDomainId;

            return (
              <TopicTreeItem
                key={id}
                itemId={id}
                rootPath={row.rootPath}
                topicInfo={topicInfo}
                selectedItem={selected?.id ?? ""} // light highlight by id
                selected={isSelected} // only strong selection in this domain
                depth={row.depth}
                onSelect={() => handleSelect(id, singleDomainId)}
              />
            );
          }}
        />
      ) : (
        <DomainFlexLayout
          key="domain-topic-layout"
          storageKey="layoutTopicDomains"
          ids={domainIds}
          componentName="domainTopicTree"
          configKey="domainId"
          insideTabId={LAYOUT_TABS.TOPICS}
          factory={(_, domainId) => {
            // topics which appear in this domain (based on publishers/subscribers)
            const topicsInDomain = filteredTopics.filter((topic) => {
              const providerIds = new Set<string>();

              for (let i = 0; i < topic.publishers.length; i += 1) {
                providerIds.add(topic.publishers[i].providerId);
              }
              for (let i = 0; i < topic.subscribers.length; i += 1) {
                providerIds.add(topic.subscribers[i].providerId);
              }

              const arr = Array.from(providerIds.values());
              for (let i = 0; i < arr.length; i += 1) {
                const pid = arr[i];
                if (providerDomainMap.get(pid) === domainId) {
                  return true;
                }
              }
              return false;
            });

            // IMPORTANT: no flattening here, we do it only in walkDomain
            const treeResult = buildTree(topicsInDomain, false);
            const roots = treeResult.topics;

            const expandedSet = new Set(expandedItems);
            const rows: FlatRow[] = [];
            const avoidSingle = searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? settings.avoidGroupWithOneItem : false;

            const walkDomain = (node: TTreeItem, depth: number, rootPath: string) => {
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

              // optional single-child group flattening for display only
              if (avoidSingle && node.topics.length === 1) {
                const nextRoot = rootPath ? `${rootPath}/${node.groupName}` : node.groupName;
                walkDomain(node.topics[0], depth, nextRoot);
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
                  walkDomain(child, depth + 1, "");
                }
              }
            };

            const sortedRoots = [...roots].sort((a, b) => {
              const aIsGroup = !a.topicInfo;
              const bIsGroup = !b.topicInfo;
              if (aIsGroup && !bIsGroup) return -1;
              if (!aIsGroup && bIsGroup) return 1;
              return a.groupName.localeCompare(b.groupName);
            });

            for (let i = 0; i < sortedRoots.length; i += 1) {
              walkDomain(sortedRoots[i], 0, "");
            }

            return (
              <Virtuoso
                style={{ height: "100%" }}
                totalCount={rows.length}
                itemContent={(index) => {
                  const row = rows[index];

                  if (row.type === "group") {
                    const node = row.treeItem;
                    const isSelected = selected?.id === row.id && selected?.domainId === domainId;

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
                        onSelect={() => handleSelect(row.id, domainId)}
                      />
                    );
                  }

                  const topicInfo = row.treeItem.topicInfo;
                  if (!topicInfo) return null;
                  const id = row.id;
                  const isSelected = selected?.id === id && selected?.domainId === domainId;

                  return (
                    <TopicTreeItem
                      key={id}
                      itemId={id}
                      rootPath={row.rootPath}
                      topicInfo={topicInfo}
                      selectedItem={selected?.id ?? ""} // light highlight for same id in other domains
                      selected={isSelected} // strong selection only in this domain
                      depth={row.depth}
                      onSelect={() => handleSelect(id, domainId)}
                    />
                  );
                }}
              />
            );
          }}
        />
      ),
    [
      domainIds,
      flatRows,
      selected,
      expandedItems,
      singleDomainId,
      toggleExpanded,
      handleSelect,
      filteredTopics,
      providerDomainMap,
      buildTree,
      searchTerm,
      settings.avoidGroupWithOneItem,
      genKey,
    ]
  );

  return (
    <Box height="100%" overflow="hidden" sx={{ backgroundColor: settings.backgroundColor }}>
      <Stack spacing={1} height="100%">
        <Stack direction="row" spacing={0.5} alignItems="center">
          {settings.buttonLocation === BUTTON_LOCATIONS.LEFT && reloadButton}
          <SearchBar
            onSearch={onSearch}
            placeholder="Filter Topics (OR: <space>, AND: +, NOT: !)"
            defaultValue={searchTerm}
            fullWidth
          />
          {settings.buttonLocation === BUTTON_LOCATIONS.RIGHT && reloadButton}
        </Stack>

        <Stack direction="row" height="100%" overflow="hidden">
          {settings.buttonLocation === BUTTON_LOCATIONS.LEFT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}

          {/* Click on empty area deselects the current item */}
          <Box width="100%" height="100%" overflow="hidden" onClick={() => setSelected(null)}>
            {treeView}
          </Box>

          {settings.buttonLocation === BUTTON_LOCATIONS.RIGHT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}
        </Stack>
      </Stack>
    </Box>
  );
}
