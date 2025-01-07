import TopicGroupTreeItem from "@/renderer/components/TopicTreeView/TopicGroupTreeItem";
import { EndpointInfo, RosTopic, TopicExtendedInfo } from "@/renderer/models";
import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import RefreshIcon from "@mui/icons-material/Refresh";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { SearchBar, TopicTreeItem } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { findIn } from "../../../utils/index";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import TopicPublishPanel from "./TopicPublishPanel";
import { grey } from "@mui/material/colors";

type TTreeItem = {
  groupKey: string;
  groupName: string;
  topics: TTreeItem[];
  count: number;
  fullPrefix: string;
  msgType: string;
  groupKeys: string[];
  topicInfo: TopicExtendedInfo | null;
};

interface TopicsPanelProps {
  initialSearchTerm?: string;
}

const TopicsPanel = forwardRef<HTMLDivElement, TopicsPanelProps>(function TopicsPanel(props, ref) {
  const { initialSearchTerm = "" } = props;

  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topics, setTopics] = useState<TopicExtendedInfo[]>([]); // [topicInfo: TopicExtendedInfo]
  const [filteredTopics, setFilteredTopics] = useState<TopicExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState<string>(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expanded, setExpanded] = useState<string[]>([]);
  const [expandedFiltered, setExpandedFiltered] = useState<string[]>([]);
  const [topicForSelected, setTopicForSelected] = useState<TopicExtendedInfo | null>(null);
  const [selectedItem, setSelectedItem] = useState<string>("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx, settingsCtx.changed]);

  function genKey(items: string[]) {
    return `${items.join("#")}`;
    //  return `${name}#${type}#${providerId}`;
  }

  const getTopicList = useCallback(async () => {
    if (!rosCtx.initialized) return;

    const newTopicsMap = new Map();

    if (rosCtx.mapProviderRosNodes) {
      // Get topics from the ros node list of each provider.
      rosCtx.mapProviderRosNodes.forEach((nodeList, providerId) => {
        nodeList.forEach((node) => {
          const addTopic = (rosTopic: RosTopic, rosNode) => {
            const topicInfo = newTopicsMap.get(genKey([rosTopic.name, rosTopic.msg_type, providerId]));
            if (topicInfo) {
              topicInfo.add(rosTopic);
            } else {
              newTopicsMap.set(
                genKey([rosTopic.name, rosTopic.msg_type, rosNode.providerId]),
                new TopicExtendedInfo(rosTopic, rosNode.providerId, rosNode.providerName)
              );
            }
          };

          node.publishers.forEach((topic) => {
            addTopic(topic, node);
          });
          node.subscribers.forEach((topic) => {
            addTopic(topic, node);
          });
        });
      });
      // sort topics by name
      const newTopics = Array.from(newTopicsMap.values());
      newTopics.sort(function (a, b) {
        return a.name.localeCompare(b.name);
      });
      setTopics(newTopics);
    }
  }, [rosCtx.initialized, rosCtx.mapProviderRosNodes]);

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
        topic.providerName,
        ...topic.publishers.map((item: EndpointInfo) => item.node_id),
        ...topic.subscribers.map((item: EndpointInfo) => item.node_id),
      ]);
      return isMatch;
    });

    setFilteredTopics(newFilteredTopics);
  }, 300);

  // Get topic list when mounting the component
  useEffect(() => {
    getTopicList();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.initialized, rosCtx.mapProviderRosNodes]);

  // Initial filter when setting the topics
  useEffect(() => {
    onSearch(searchTerm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topics, searchTerm]);

  // create tree based on topic namespace
  // topics are grouped only if more then one is in the group
  const fillTree = (fullPrefix: string, topicsGroup: TopicExtendedInfo[], itemId: string) => {
    const byPrefixP1 = new Map<string, { restNameSuffix: string; topicInfo: TopicExtendedInfo }[]>();
    // create a map with simulated tree for the namespaces of the topic list
    topicsGroup.forEach((topicInfo) => {
      const nameSuffix = topicInfo.id.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName)?.push({ restNameSuffix, topicInfo });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, topicInfo }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ restNameSuffix: "", topicInfo }]);
      }
    });

    // create result
    let count = 0;
    const groupKeys: string[] = [];
    const newFilteredTopics: TTreeItem[] = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const newFullPrefix: string = `${fullPrefix}/${groupName}`;
      if (value.length > 1) {
        const groupKey: string = itemId ? `${itemId}-${groupName}` : groupName;
        groupKeys.push(groupKey);
        const groupTopics = value.map((item) => {
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
        value.map((item) => {
          if (msgType === undefined) {
            msgType = item.topicInfo.msgType;
          } else if (msgType !== item.topicInfo.msgType) {
            if (msgType !== "") {
              msgType = "";
            }
          }
        });
        newFilteredTopics.push({
          groupKey: groupKey,
          groupName: `/${groupName}`,
          topics: subResult.topics,
          count: subResult.count,
          fullPrefix: newFullPrefix,
          msgType: msgType ? msgType : "",
          groupKeys: groupKeys,
          topicInfo: null,
        } as TTreeItem);
      } else {
        newFilteredTopics.push({
          groupKey: "",
          groupName: "",
          topics: [],
          count: 0,
          fullPrefix: newFullPrefix,
          msgType: value[0].topicInfo.msgType,
          groupKeys: groupKeys,
          topicInfo: value[0].topicInfo,
        } as TTreeItem);
        if (value[0].topicInfo.providerName !== groupName) {
          // since the same topic can be on multiple provider
          // we count only topics
          count += 1;
        }
      }
    });
    return { topics: newFilteredTopics, count, groupKeys };
  };

  // create topics tree from filtered topic list
  useEffect(() => {
    const tree = fillTree("", filteredTopics, "");
    setRootDataList(tree.topics);
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      setExpandedFiltered(tree.groupKeys);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [filteredTopics]);

  const onEchoClick = useCallback(
    (topic: TopicExtendedInfo | null, external: boolean, openInTerminal: boolean = false) => {
      if (topic) {
        rosCtx.openSubscriber(topic.providerId, topic.name, true, false, external, openInTerminal);
      }
    },
    [rosCtx]
  );

  const onPublishClick = useCallback((topic: TopicExtendedInfo | null) => {
    if (topic) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `publish-${topic.name}-${topic.providerId}`,
          topic.name,
          <TopicPublishPanel topicName={topic.name} providerId={topic.providerId} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(true, "publish")
        )
      );
    }
  }, []);

  const topicTreeToStyledItems = useCallback((rootPath: string, treeItem: TTreeItem, selectedItem: string | null) => {
    if (treeItem.topicInfo) {
      return (
        <TopicTreeItem
          key={genKey([treeItem.topicInfo.name, treeItem.topicInfo.msgType, treeItem.topicInfo.providerId])}
          itemId={genKey([treeItem.topicInfo.name, treeItem.topicInfo.msgType, treeItem.topicInfo.providerId])}
          rootPath={rootPath}
          topicInfo={treeItem.topicInfo}
          selectedItem={selectedItem}
        />
      );
    } else {
      return (
        <TopicGroupTreeItem
          key={treeItem.groupKey}
          itemId={treeItem.groupKey}
          rootPath={rootPath}
          groupName={treeItem.groupName}
          countChildren={treeItem.count}
        >
          {treeItem.topics.map((subItem) => {
            return topicTreeToStyledItems(treeItem.fullPrefix, subItem, selectedItem);
          })}
        </TopicGroupTreeItem>
      );
    }
  }, []);

  useEffect(() => {
    const selectedTopics = filteredTopics.filter((item) => {
      return genKey([item.name, item.msgType, item.providerId]) === selectedItem;
    });
    if (selectedTopics?.length > 0) {
      setTopicForSelected(selectedTopics[0]);
    } else {
      setTopicForSelected(null);
    }
  }, [filteredTopics, selectedItem]);

  const handleToggle = useCallback(
    (itemIds: string[]) => {
      if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
        setExpanded(itemIds);
      } else {
        setExpandedFiltered(itemIds);
      }
    },
    [searchTerm]
  );

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
          title="Publish to"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!topicForSelected}
              size="medium"
              aria-label="Create a publisher"
              onClick={() => {
                onPublishClick(topicForSelected);
              }}
            >
              <PlayCircleOutlineIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
      </ButtonGroup>
    );
  }, [tooltipDelay, topicForSelected]);

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
  }, [expanded, expandedFiltered, rootDataList, searchTerm]);

  const createPanel = useMemo(() => {
    return (
      <Box
        ref={ref}
        height="100%"
        overflow="auto"
        sx={{ backgroundColor: settingsCtx.get("backgroundColor") as string }}
      >
        <Stack
          spacing={1}
          height="100%"
          // sx={{
          //   height: '100%',
          //   display: 'flex',
          // }}
        >
          <Stack direction="row" spacing={0.5} alignItems="center">
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
            <SearchBar
              onSearch={onSearch}
              placeholder="Filter Topics (OR: <space>, AND: +, NOT: !)"
              defaultValue={initialSearchTerm}
              fullWidth
            />
          </Stack>
          <Stack direction="row" height="100%" overflow="auto">
            <Box height="100%" sx={{ boxShadow: `0px 0px 5px ${alpha(grey[600], 0.4)}` }}>
              {/* <Box height="100%" sx={{ borderRight: "solid", borderColor: "#D3D3D3", borderWidth: 1 }}> */}
              {/* <Paper elevation={0} sx={{ border: 1 }} height="100%"> */}
              {createButtonBox}
              {/* </Paper> */}
            </Box>
            <Box width="100%" height="100%" overflow="auto">
              {createTreeView}
            </Box>
          </Stack>
        </Stack>
      </Box>
    );
  }, [rootDataList, expanded, expandedFiltered, searchTerm, selectedItem, topicForSelected, settingsCtx.changed]);
  return createPanel;
});

export default TopicsPanel;
