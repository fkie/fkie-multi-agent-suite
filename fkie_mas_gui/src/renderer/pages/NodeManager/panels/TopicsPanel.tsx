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

// flacher Eintrag für Virtuoso
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

  const [topics, setTopics] = useState<TopicExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expandedItems, setExpandedItems] = useState<string[]>([]);
  const [selectedItem, setSelectedItem] = useState("");
  const [topicForSelected, setTopicForSelected] = useState<TopicExtendedInfo | undefined>();
  const [availableProviders, setAvailableProviders] = useState<TProviderDescription[]>([]);
  const [settings, setSettings] = useState<TSettings>({
    tooltipDelay: settingsCtx.get("tooltipEnterDelay") as number,
    avoidGroupWithOneItem: settingsCtx.get("avoidGroupWithOneItem") as boolean,
    backgroundColor: settingsCtx.get("backgroundColor") as string,
    buttonLocation: settingsCtx.get("buttonLocation") as string,
  });

  const genKey = useCallback((items: string[]) => items.join("#"), []);

  const getAvailableProviders = useCallback(() => {
    const providers: TProviderDescription[] = [];
    for (const item of rosCtx.providers) {
      providers.push({
        providerId: item.id,
        providerName: item.name(),
      });
    }
    return providers;
  }, [rosCtx.providers]);

  const updateTopicList = useCallback(async () => {
    if (!rosCtx.initialized) return;

    const newTopicsMap = new Map<string, TopicExtendedInfo>();

    for (const provider of rosCtx.providers) {
      for (const topic of provider.rosTopics) {
        for (const rosNode of provider.rosNodes) {
          const key = genKey([topic.name, topic.msg_type]);
          let topicInfo = newTopicsMap.get(key);
          if (topicInfo === undefined) {
            topicInfo = new TopicExtendedInfo(topic, rosNode);
            newTopicsMap.set(key, topicInfo);
          } else {
            topicInfo.add(rosNode);
          }
        }
      }
    }

    const newTopics = Array.from(newTopicsMap.values()).sort((a, b) => {
      const aSeps = (a.name.match(/\//g) || []).length;
      const bSeps = (b.name.match(/\//g) || []).length;
      return aSeps === bSeps ? a.name.localeCompare(b.name) : bSeps - aSeps;
    });

    setTopics(newTopics);
  }, [rosCtx.initialized, rosCtx.providers, rosCtx.mapProviderRosNodes, genKey]);

  const getTopicList = useCallback(() => {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }, [rosCtx.providers]);

  // Baumstruktur wie bisher
  const buildTree = useCallback((topicsList: TopicExtendedInfo[], avoidSingle: boolean): TTreeResult => {
    const nodes = new Map<string, TTreeItem>();
    const rootNodes: TTreeItem[] = [];

    // Phase 1: Knoten erzeugen
    for (const topic of topicsList) {
      const parts = topic.name.split("/").filter(Boolean);
      let currentPath = "";

      for (let i = 0; i < parts.length; i++) {
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

    // Phase 2: Verknüpfen, QoS nach oben propagieren
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

    // Sortierung: Gruppen vor Topics, dann alphabetisch
    rootNodes.sort((a, b) => {
      if (a.topics.length > 0 && b.topics.length === 0) return -1;
      if (b.topics.length > 0 && a.topics.length === 0) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    // Optional: Single-Child-Gruppen flatten
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
      node.topics = node.topics.map(flattenSingleChildGroups);
    }

    return node;
  }, []);

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
      searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? settings.avoidGroupWithOneItem : false
    );
    return treeResult.topics;
  }, [filteredTopics, settings.avoidGroupWithOneItem, searchTerm.length, buildTree]);

  useEffect(() => {
    setAvailableProviders(getAvailableProviders());
  }, [rosCtx.providers, getAvailableProviders]);

  useEffect(() => {
    updateTopicList();
  }, [rosCtx.mapProviderRosNodes]);

  useEffect(() => {
    updateTopicList();
  }, []);

  useEffect(() => {
    setSettings({
      tooltipDelay: settingsCtx.get("tooltipEnterDelay") as number,
      avoidGroupWithOneItem: settingsCtx.get("avoidGroupWithOneItem") as boolean,
      backgroundColor: settingsCtx.get("backgroundColor") as string,
      buttonLocation: settingsCtx.get("buttonLocation") as string,
    });
  }, [settingsCtx.changed]);

  useEffect(() => {
    const selectedTopic = filteredTopics.find((item) => genKey([item.name, item.msgType]) === selectedItem);
    setTopicForSelected(selectedTopic);
  }, [filteredTopics, selectedItem, genKey]);

  useEffect(() => {
    setRootDataList(treeData);
  }, [treeData]);

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, updateTopicList);
  useCustomEventListener(EVENT_FILTER_TOPICS, (filter: TFilterText) => setSearchTerm(filter.data));

  const onEchoClick = useCallback(
    (topic: TopicExtendedInfo | undefined, external: boolean, openInTerminal = false, providerId = "") => {
      if (!topic) return;

      const provId = providerId || topic.publishers[0]?.providerId || topic.subscribers[0]?.providerId;

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
      const provId = topic.subscribers[0]?.providerId || topic.publishers[0]?.providerId || "";
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

  // expand/collapse für Gruppen
  const toggleExpanded = useCallback((id: string) => {
    setExpandedItems((prev) => (prev.includes(id) ? prev.filter((x) => x !== id) : [...prev, id]));
  }, []);

  // Auswahl
  const handleSelect = useCallback((itemId: string) => {
    setSelectedItem(itemId);
  }, []);

  // Baum -> flache Liste (sichtbare Zeilen) wie bisherige topicTreeToStyledItems-Logik
  const flatRows = useMemo<FlatRow[]>(() => {
    const expandedSet = new Set(expandedItems);
    const rows: FlatRow[] = [];

    const walk = (node: TTreeItem, depth: number, rootPath: string) => {
      // Leaf-Topic
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

      // Optional: Single-Child-Gruppen flatten (Anzeige)
      if (settings.avoidGroupWithOneItem && node.topics.length === 1) {
        const nextRoot = rootPath ? `${rootPath}/${node.groupName}` : node.groupName;
        walk(node.topics[0], depth, nextRoot);
        return;
      }

      // Gruppen-Eintrag
      rows.push({
        id: node.groupKey,
        type: "group",
        depth,
        treeItem: node,
        rootPath,
      });

      // Kinder nur einfügen, wenn expanded
      if (expandedSet.has(node.groupKey)) {
        // Sortierung: Gruppen vor Topics, dann alphabetisch
        const sortedChildren = [...node.topics].sort((a, b) => {
          const aIsGroup = !a.topicInfo;
          const bIsGroup = !b.topicInfo;
          if (aIsGroup && !bIsGroup) return -1;
          if (!aIsGroup && bIsGroup) return 1;
          return a.groupName.localeCompare(b.groupName);
        });

        for (const child of sortedChildren) {
          // wie im Original: rootPath für Kinder auf "" zurücksetzen
          walk(child, depth + 1, "");
        }
      }
    };

    // Root-Ebene ebenfalls sortieren
    const sortedRoots = [...rootDataList].sort((a, b) => {
      const aIsGroup = !a.topicInfo;
      const bIsGroup = !b.topicInfo;
      if (aIsGroup && !bIsGroup) return -1;
      if (!aIsGroup && bIsGroup) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    for (const root of sortedRoots) {
      walk(root, 0, "");
    }

    return rows;
  }, [rootDataList, expandedItems, settings.avoidGroupWithOneItem, genKey]);

  const treeView = useMemo(
    () => (
      <Virtuoso
        style={{ height: "100%" }}
        totalCount={flatRows.length}
        itemContent={(index) => {
          const row = flatRows[index];

          if (row.type === "group") {
            const node = row.treeItem;
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
                selected={selectedItem === row.id}
                onToggle={() => toggleExpanded(row.id)}
                onSelect={() => handleSelect(row.id)}
              />
            );
          }

          const topicInfo = row.treeItem.topicInfo;
          if (!topicInfo) return;
          const id = row.id;

          return (
            <TopicTreeItem
              key={id}
              itemId={id}
              rootPath={row.rootPath}
              topicInfo={topicInfo}
              selectedItem={selectedItem}
              selected={selectedItem === id}
              depth={row.depth}
              onSelect={() => handleSelect(id)}
            />
          );
        }}
      />
    ),
    [flatRows, expandedItems, selectedItem, toggleExpanded, handleSelect]
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

          {/* Klick auf leeren Bereich des Panels deselektiert alles */}
          <Box width="100%" height="100%" overflow="hidden" onClick={() => setSelectedItem("")}>
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
