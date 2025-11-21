import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import ChatOutlinedIcon from "@mui/icons-material/ChatOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import RefreshIcon from "@mui/icons-material/Refresh";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { grey } from "@mui/material/colors";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { useContext, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import TopicGroupTreeItem from "@/renderer/components/TopicTreeView/TopicGroupTreeItem";
import TopicTreeItem from "@/renderer/components/TopicTreeView/TopicTreeItem";
import OverflowMenu from "@/renderer/components/UI/OverflowMenu";
import SearchBar from "@/renderer/components/UI/SearchBar";
import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { BUTTON_LOCATIONS, SettingsContext } from "@/renderer/context/SettingsContext";
import { RosNode, RosTopic, TopicExtendedInfo } from "@/renderer/models";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import { Provider } from "@/renderer/providers";
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

type TProviderDescription = {
  providerId: string;
  providerName: string;
};
interface TopicsPanelProps {
  initialSearchTerm?: string;
}

export default function TopicsPanel(props: TopicsPanelProps): JSX.Element {
  const { initialSearchTerm = "" } = props;

  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topics, setTopics] = useState<TopicExtendedInfo[]>([]); // [topicInfo: TopicExtendedInfo]
  const [filteredTopics, setFilteredTopics] = useState<TopicExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState<string>(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expanded, setExpanded] = useState<string[]>([]);
  const [expandedFiltered, setExpandedFiltered] = useState<string[]>([]);
  const [topicForSelected, setTopicForSelected] = useState<TopicExtendedInfo | undefined>();
  const [selectedItem, setSelectedItem] = useState<string>("");
  const [availableProviders, setAvailableProviders] = useState<TProviderDescription[]>([]);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [avoidGroupWithOneItem, setAvoidGroupWithOneItem] = useState<boolean>(
    settingsCtx.get("avoidGroupWithOneItem") as boolean
  );
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [buttonLocation, setButtonLocation] = useState<string>(settingsCtx.get("buttonLocation") as string);

  useEffect(() => {
    setAvoidGroupWithOneItem(settingsCtx.get("avoidGroupWithOneItem") as boolean);
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setButtonLocation(settingsCtx.get("buttonLocation") as string);
  }, [settingsCtx, settingsCtx.changed]);

  function getAvailableProviders(): TProviderDescription[] {
    return rosCtx.providers.map((item: Provider) => {
      return { providerId: item.id, providerName: item.name() } as TProviderDescription;
    });
  }

  function genKey(items: string[]): string {
    return `${items.join("#")}`;
    //  return `${name}#${type}#${providerId}`;
  }

  async function updateTopicList(): Promise<void> {
    if (!rosCtx.initialized) return;

    function addTopic(rosTopic: RosTopic, rosNode: RosNode): void {
      const topicInfo: TopicExtendedInfo = newTopicsMap.get(genKey([rosTopic.name, rosTopic.msg_type]));
      if (topicInfo) {
        topicInfo.add(rosNode);
      } else {
        newTopicsMap.set(genKey([rosTopic.name, rosTopic.msg_type]), new TopicExtendedInfo(rosTopic, rosNode));
      }
    }

    const newTopicsMap = new Map();
    // Get topics from the ros node list of each provider.
    for (const provider of rosCtx.providers) {
      for (const topic of provider.rosTopics) {
        for (const rosNode of provider.rosNodes) {
          addTopic(topic, rosNode);
        }
      }
    }
    // sort topics by name
    const newTopics = Array.from(newTopicsMap.values());
    newTopics.sort((a, b) => {
      // sort groups first
      const aCountSep = (a.name.match(/\//g) || []).length;
      const bCountSep = (b.name.match(/\//g) || []).length;
      if (aCountSep === bCountSep) {
        return a.name.localeCompare(b.name);
      }
      return aCountSep < bCountSep ? 1 : -1;
    });
    setTopics(newTopics);
  }

  function getTopicList(): void {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, () => {
    updateTopicList();
  });

  useCustomEventListener(EVENT_FILTER_TOPICS, (filter: TFilterText) => {
    setSearchTerm(filter.data);
  });

  // debounced search callback
  const onSearch = useDebounceCallback((newSearchTerm) => {
    setSearchTerm(newSearchTerm);
    if (!newSearchTerm) {
      setFilteredTopics(topics);
      return;
    }

    const newFilteredTopics = topics.filter((topic: TopicExtendedInfo) => {
      const isMatch = findIn(newSearchTerm, [
        topic.name,
        topic.msgType,
        ...topic.publishers.map((item: EndpointExtendedInfo) => `${item.info.node_id} ${item.providerName}`),
        ...topic.subscribers.map((item: EndpointExtendedInfo) => `${item.info.node_id} ${item.providerName}`),
      ]);
      return isMatch;
    });

    setFilteredTopics(newFilteredTopics);
  }, 300);

  // Get topic list
  useEffect(() => {
    updateTopicList();
    setAvailableProviders(getAvailableProviders());
  }, [rosCtx.initialized]);

  // Get new provider list if providers are changed
  useEffect(() => {
    setAvailableProviders(getAvailableProviders());
  }, [rosCtx.providers]);

  // Initial filter when setting the topics
  useEffect(() => {
    onSearch(searchTerm);
  }, [topics, searchTerm]);

  // create tree based on topic namespace
  // topics are grouped only if more then one is in the group
  function fillTree(fullPrefix: string, topicsGroup: TopicExtendedInfo[], itemId: string): TTreeItem {
    const byPrefixP1 = new Map<string, { restNameSuffix: string; topicInfo: TopicExtendedInfo; isGroup: boolean }[]>();
    // create a map with simulated tree for the namespaces of the topic list
    for (const topicInfo of topicsGroup) {
      const nameSuffix = topicInfo.name.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName)?.push({ restNameSuffix, topicInfo, isGroup: true });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, topicInfo, isGroup: true }]);
        }
      } else {
        byPrefixP1.set(`${groupName}#${topicInfo.msgType}`, [{ restNameSuffix: "", topicInfo, isGroup: false }]);
      }
    }

    // create result
    let count = 0;
    const groupKeys: string[] = [];
    const newFilteredTopics: TTreeItem[] = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const newFullPrefix: string = `${fullPrefix}/${groupName}`;
      const topicValues = value.filter((item) => !item.isGroup);
      const groupValues = value.filter((item) => !topicValues.includes(item));
      if (groupValues.length > 0) {
        const groupKey: string = itemId ? `${itemId}-${groupName}` : groupName;
        groupKeys.push(groupKey);
        const groupTopics = groupValues.map((item) => {
          return item.topicInfo;
        });
        const subResult = fillTree(newFullPrefix, groupTopics, groupKey);
        // the result count is 0 -> we added multiple provider for topic with same name.
        if (subResult.count === 0) {
          count += 1;
        } else {
          count += subResult.count;
        }
        if (subResult.groupKeys.length > 0) {
          groupKeys.push(...subResult.groupKeys);
        }
        // if all topics of the group have same message id, show it in the group info
        let msgType: string | undefined = undefined;
        let hasIncompatibleQos: boolean = false;
        groupValues.map((item) => {
          if (msgType === undefined) {
            msgType = item.topicInfo.msgType;
          } else if (msgType !== item.topicInfo.msgType) {
            if (msgType !== "") {
              msgType = "";
            }
          }
          if (item.topicInfo.hasIncompatibleQos) {
            hasIncompatibleQos = item.topicInfo.hasIncompatibleQos;
          }
        });
        newFilteredTopics.push({
          groupKey: groupKey,
          groupName: groupName,
          topics: subResult.topics,
          count: subResult.count,
          fullPrefix: fullPrefix,
          msgType: msgType ? msgType : "",
          groupKeys: groupKeys,
          topicInfo: null,
          hasIncompatibleQos: hasIncompatibleQos,
        } as TTreeItem);
      }
      for (const item of topicValues) {
        newFilteredTopics.push({
          groupKey: "",
          groupName: "",
          topics: [],
          count: 0,
          fullPrefix: fullPrefix,
          msgType: item.topicInfo.msgType,
          groupKeys: groupKeys,
          topicInfo: item.topicInfo,
          hasIncompatibleQos: item.topicInfo.hasIncompatibleQos,
        } as TTreeItem);
        count += 1;
        // if (item.topicInfo.providerName !== groupName) {
        //   // since the same topic can be on multiple provider
        //   // we count only topics
        //   count += 1;
        // }
      }
    });
    return { topics: newFilteredTopics, count, groupKeys } as TTreeItem;
  }

  // create topics tree from filtered topic list
  useEffect(() => {
    const tree = fillTree("", filteredTopics, "");
    setRootDataList(tree.topics);
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      setExpandedFiltered(tree.groupKeys);
    }
  }, [filteredTopics]);

  function onEchoClick(
    topic: TopicExtendedInfo | undefined,
    external: boolean,
    openInTerminal: boolean = false,
    providerId: string = ""
  ): void {
    if (topic) {
      let provId = providerId ? providerId : undefined;
      if (!provId) {
        provId = topic.publishers.length > 0 ? topic.publishers[0].providerId : undefined;
      }
      if (!provId) {
        provId = topic.subscribers.length > 0 ? topic.subscribers[0].providerId : undefined;
      }
      if (provId) {
        navCtx.openSubscriber(provId, topic.name, true, false, external, openInTerminal);
      } else {
        logCtx.warn("no publisher available");
      }
    }
  }

  function onPublishClick(
    topic: TopicExtendedInfo | undefined,
    external: boolean,
    openInTerminal: boolean = false,
    providerId: string = ""
  ): void {
    let provId = providerId ? providerId : undefined;
    if (!provId && topic) {
      provId = topic.subscribers.length > 0 ? topic.subscribers[0].providerId : undefined;
    }
    if (!provId && topic) {
      provId = topic.publishers.length > 0 ? topic.publishers[0].providerId : undefined;
    }
    navCtx.startPublisher(provId || "", topic?.name, topic?.msgType, external, openInTerminal);
  }

  function topicTreeToStyledItems(rootPath: string, treeItem: TTreeItem, selectedItem: string | null): JSX.Element {
    if (treeItem.topicInfo) {
      return (
        <TopicTreeItem
          key={genKey([treeItem.topicInfo.name, treeItem.topicInfo.msgType])}
          itemId={genKey([treeItem.topicInfo.name, treeItem.topicInfo.msgType])}
          rootPath={rootPath}
          topicInfo={treeItem.topicInfo}
          selectedItem={selectedItem}
        />
      );
    }
    if (avoidGroupWithOneItem && treeItem.topics.length === 1) {
      // avoid groups with one item
      return topicTreeToStyledItems(
        rootPath.length > 0 ? `${rootPath}/${treeItem.groupName}` : treeItem.groupName,
        treeItem.topics[0],
        selectedItem
      );
    }
    return (
      <TopicGroupTreeItem
        key={treeItem.groupKey}
        itemId={treeItem.groupKey}
        rootPath={rootPath}
        groupName={treeItem.groupName}
        countChildren={treeItem.count}
        hasIncompatibleQos={treeItem.hasIncompatibleQos}
      >
        {treeItem.topics.map((subItem) => {
          return topicTreeToStyledItems("", subItem, selectedItem);
        })}
      </TopicGroupTreeItem>
    );
  }

  useEffect(() => {
    const selectedTopics = filteredTopics.filter((item) => {
      return genKey([item.name, item.msgType]) === selectedItem;
    });
    if (selectedTopics?.length > 0) {
      setTopicForSelected(selectedTopics[0]);
    } else {
      setTopicForSelected(undefined);
    }
  }, [filteredTopics, selectedItem]);

  function handleToggle(itemIds: string[]): void {
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      setExpanded(itemIds);
    } else {
      setExpandedFiltered(itemIds);
    }
  }

  const createButtonBox = useMemo(() => {
    return (
      <ButtonGroup orientation="vertical" aria-label="topic control group">
        <Tooltip
          title="Echo (shift+click for alternative open location)"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!topicForSelected}
              size="medium"
              aria-label="Subscribe to topic and show the output"
              onClick={(event) => {
                onEchoClick(topicForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey);
              }}
            >
              <ChatBubbleOutlineIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>

        <OverflowMenu
          disabled={!topicForSelected}
          icon={
            <Tooltip
              title="Echo with option to select the host on which the subscriber is to be started (shift+click for alternative open location)"
              placement="left"
              enterDelay={tooltipDelay}
              // enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <ChatOutlinedIcon fontSize="inherit" />
            </Tooltip>
          }
          size="medium"
          sx={{ margin: 0 }}
          options={availableProviders.map((item) => {
            return {
              name: item.providerName,
              key: item.providerId,
              onClick: async (event?: React.MouseEvent): Promise<void> => {
                onEchoClick(
                  topicForSelected,
                  event?.nativeEvent.shiftKey || false,
                  event?.nativeEvent.ctrlKey || false,
                  item.providerId
                );
              },
            };
          })}
          id={`echo-provider-menu-${topicForSelected?.name}`}
        />
        <Tooltip
          title="Echo in Terminal"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!topicForSelected}
              size="medium"
              aria-label="Subscribe to topic and show the output"
              onClick={(event) => {
                onEchoClick(topicForSelected, event.nativeEvent.shiftKey, true);
              }}
            >
              <DvrIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title="Create a publisher"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <IconButton
            size="medium"
            aria-label="Create a publisher"
            onClick={(event) => {
              onPublishClick(topicForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey);
            }}
          >
            <PlayCircleOutlineIcon fontSize="inherit" />
          </IconButton>
        </Tooltip>
      </ButtonGroup>
    );
  }, [tooltipDelay, topicForSelected, availableProviders]);

  const createTreeView = useMemo(() => {
    return (
      <SimpleTreeView
        aria-label="topics"
        expandedItems={searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        expansionTrigger={"iconContainer"}
        onExpandedItemsChange={(_event, itemIds) => handleToggle(itemIds)}
        onSelectedItemsChange={(_event, itemId) => {
          setSelectedItem(itemId || "");
          const copyExpanded = [...(searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
          if (itemId) {
            const index =
              searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS
                ? expanded.indexOf(itemId)
                : expandedFiltered.indexOf(itemId);
            if (index === -1) {
              if (itemId) {
                copyExpanded.push(itemId);
              }
            } else {
              copyExpanded.splice(index, 1);
            }
          }
          if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
            setExpanded(copyExpanded);
          } else {
            setExpandedFiltered(copyExpanded);
          }
        }}
      >
        {rootDataList.map((item) => {
          return topicTreeToStyledItems("", item, selectedItem);
        })}
      </SimpleTreeView>
    );
  }, [expanded, expandedFiltered, rootDataList, searchTerm, filteredTopics, avoidGroupWithOneItem]);

  const createReloadButton = useMemo(() => {
    return (
      <Tooltip title="Reload topic list" placement="left" disableInteractive>
        <IconButton
          size="small"
          onClick={() => {
            getTopicList();
          }}
        >
          <RefreshIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    );
  }, [rosCtx.providers]);

  const createPanel = useMemo(() => {
    return (
      <Box height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
        <Stack
          spacing={1}
          height="100%"
          // sx={{
          //   height: '100%',
          //   display: 'flex',
          // }}
        >
          <Stack direction="row" spacing={0.5} alignItems="center">
            {buttonLocation === BUTTON_LOCATIONS.LEFT && createReloadButton}
            <SearchBar
              onSearch={(term) => onSearch(term)}
              placeholder="Filter Topics (OR: <space>, AND: +, NOT: !)"
              defaultValue={searchTerm}
              fullWidth
            />
            {buttonLocation === BUTTON_LOCATIONS.RIGHT && createReloadButton}
          </Stack>
          <Stack direction="row" height="100%" overflow="auto">
            {buttonLocation === BUTTON_LOCATIONS.LEFT && (
              <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
                {createButtonBox}
              </Box>
            )}
            <Box width="100%" height="100%" overflow="auto">
              {createTreeView}
            </Box>
            {buttonLocation === BUTTON_LOCATIONS.RIGHT && (
              <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
                {createButtonBox}
              </Box>
            )}
          </Stack>
        </Stack>
      </Box>
    );
  }, [
    rootDataList,
    expanded,
    expandedFiltered,
    searchTerm,
    selectedItem,
    topicForSelected,
    backgroundColor,
    buttonLocation,
    avoidGroupWithOneItem,
    rosCtx.providers,
  ]);
  return createPanel;
}
