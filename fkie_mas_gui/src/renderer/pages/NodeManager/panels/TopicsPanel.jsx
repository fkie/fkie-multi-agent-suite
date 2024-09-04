import { Brightness1 } from "@mui/icons-material";
import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import RefreshIcon from "@mui/icons-material/Refresh";
import { Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { SearchBar, TopicTreeItem } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { CmdType } from "../../../providers";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../../utils/events";
import { findIn } from "../../../utils/index";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import TopicEchoPanel from "./TopicEchoPanel";
import TopicPublishPanel from "./TopicPublishPanel";

class TopicExtendedInfo {
  id;

  name;

  msgtype = "";

  providerId = "";

  providerName = "";

  publishers = [];

  subscribers = [];

  // providers = {}; // {providerId: string, providerName: string}
  // nodes = {}; //{(providerId: string, itemId: string): nodeName: string}

  constructor(topic, providerId, providerName) {
    this.id = `${topic.name}/${providerName}`;
    this.name = topic.name;
    this.msgtype = topic.msgtype;
    this.providerId = providerId;
    this.providerName = providerName;
    this.addPublishers(topic.publisher);
    this.addSubscribers(topic.subscriber);
  }

  addPublishers(publishers) {
    publishers.forEach((pub) => {
      if (!this.publishers.includes(pub)) {
        this.publishers.push(pub);
      }
    });
  }

  addSubscribers(subscribers) {
    subscribers.forEach((sub) => {
      if (!this.subscribers.includes(sub)) {
        this.subscribers.push(sub);
      }
    });
  }

  add(topic) {
    this.addPublishers(topic.publisher);
    this.addSubscribers(topic.subscriber);
  }
}

function TopicsPanel({ initialSearchTerm = "" }) {
  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topics, setTopics] = useState([]); // [topicInfo: TopicExtendedInfo]
  const [filteredTopics, setFilteredTopics] = useState([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState([]);
  const [expanded, setExpanded] = useState([]);
  const [expandedFiltered, setExpandedFiltered] = useState([]);
  const [topicForSelected, setTopicForSelected] = useState(null);
  const [selectedItem, setSelectedItem] = useState("");

  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  function genKey(items) {
    return `${items.join("#")}`;
    //  return `${name}#${type}#${providerId}`;
  }

  const getTopicList = useCallback(async () => {
    if (!rosCtx.initialized) return;

    const newProviderKeyName = {};
    const newNodeKeyName = {};
    const newTopicsMap = new Map();

    if (rosCtx.mapProviderRosNodes) {
      // Get topics from the ros node list of each provider.
      rosCtx.mapProviderRosNodes.forEach((nodeList, providerId) => {
        nodeList.forEach((node) => {
          newNodeKeyName[genKey([providerId, node.id])] = node.name;
          newProviderKeyName[node.providerId] = node.providerName;

          const addTopic = (rosTopic, rosNode) => {
            const topicInfo = newTopicsMap.get(genKey([rosTopic.name, rosTopic.msgtype, providerId]));
            if (topicInfo) {
              topicInfo.add(rosTopic);
            } else {
              newTopicsMap.set(
                genKey([rosTopic.name, rosTopic.msgtype, rosNode.providerId]),
                new TopicExtendedInfo(rosTopic, rosNode.providerId, rosNode.providerName)
              );
            }
          };

          node.subscribers.forEach((topic) => {
            addTopic(topic, node);
          });
          node.publishers.forEach((topic) => {
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
  }, [rosCtx.initialized, rosCtx.mapProviderRosNodes, setTopics]);

  // debounced search callback
  const onSearch = useDebounceCallback((newSearchTerm) => {
    setSearchTerm(newSearchTerm);
    if (!newSearchTerm) {
      setFilteredTopics(topics);
      return;
    }

    const newFilteredTopics = topics.filter((topic) => {
      const isMatch = findIn(newSearchTerm, [
        topic.name,
        topic.msgtype,
        topic.providerName,
        topic.publishers,
        topic.subscribers,
      ]);
      return isMatch;
    });

    setFilteredTopics(newFilteredTopics);
  }, 300);

  // Get topic list when mounting the component
  useEffect(() => {
    getTopicList();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.mapProviderRosNodes]);

  // Initial filter when setting the topics
  useEffect(() => {
    onSearch(searchTerm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topics, searchTerm]);

  // create tree based on topic namespace
  // topics are grouped only if more then one is in the group
  const fillTree = (fullPrefix, topicsGroup, itemId) => {
    const groupKeys = [];
    const byPrefixP1 = new Map("", []);
    // create a map with simulated tree for the namespaces of the topic list
    Object.entries(topicsGroup).forEach(([key, topicInfo]) => {
      const nameSuffix = topicInfo.id.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName).push({ restNameSuffix, topicInfo });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, topicInfo }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ topicInfo }]);
      }
    });

    let count = 0;
    // create result
    const newFilteredTopics = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const groupKey = itemId ? `${itemId}-${groupName}` : groupName;
      const newFullPrefix = `${fullPrefix}/${groupName}`;
      if (value.length > 1) {
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
        let msgType = undefined;
        value.map((item) => {
          if (msgType === undefined) {
            msgType = item.topicInfo.msgtype;
          } else if (msgType !== item.topicInfo.msgtype) {
            if (msgType !== "") {
              msgType = "";
            }
          }
        });
        newFilteredTopics.push([
          {
            groupKey: groupKey,
            groupName: `/${groupName}`,
            topics: subResult.topics,
            count: subResult.count,
            fullPrefix: newFullPrefix,
            msgType: msgType ? msgType : "",
            groupKeys: groupKeys,
          },
        ]);
      } else {
        newFilteredTopics.push(value[0].topicInfo);
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
    (topic, external, openInTerminal = false) => {
      rosCtx.openSubscriber(topic.providerId, topic.name, true, false, external, openInTerminal);
    },
    [rosCtx]
  );

  const onPublishClick = useCallback((topic) => {
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
  }, []);

  const topicTreeToStyledItems = useCallback((rootPath, treeItem, selectedItem) => {
    if (Array.isArray(treeItem)) {
      return treeItem.map((item) => {
        return (
          <TopicTreeItem
            key={item.groupKey}
            itemId={item.groupKey}
            labelRoot={rootPath}
            labelText={item.groupName}
            labelCount={item.count}
            labelInfo={item.msgType}
            selectedItem={selectedItem}
          >
            {item.topics.map((subItem) => {
              return topicTreeToStyledItems(item.fullPrefix, subItem, selectedItem);
            })}
          </TopicTreeItem>
        );
      });
    }
    return (
      <TopicTreeItem
        key={genKey([treeItem.name, treeItem.msgtype, treeItem.providerId])}
        itemId={genKey([treeItem.name, treeItem.msgtype, treeItem.providerId])}
        labelRoot={rootPath}
        labelText={`${treeItem.name}`}
        labelInfo={treeItem.msgtype}
        color="#1a73e8"
        bgColor="#e8f0fe"
        colorForDarkMode="#B8E7FB"
        bgColorForDarkMode="#071318"
        topicInfo={treeItem}
        selectedItem={selectedItem}
      />
    );
  }, []);

  useEffect(() => {
    const selectedTopics = filteredTopics.filter((item) => {
      return genKey([item.name, item.msgtype, item.providerId]) === selectedItem;
    });
    if (selectedTopics?.length >= 0) {
      setTopicForSelected(selectedTopics[0]);
    } else {
      setTopicForSelected(null);
    }
  }, [filteredTopics, selectedItem]);

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
        onSelectedItemsChange={(event, itemId) => {
          setSelectedItem(itemId);
          const index =
            searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS
              ? expanded.indexOf(itemId)
              : expandedFiltered.indexOf(itemId);
          const copyExpanded = [...(searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
          if (index === -1) {
            copyExpanded.push(itemId);
          } else {
            copyExpanded.splice(index, 1);
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
      <Box height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
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
              placeholder="Filter Topics (<space> for OR, + for AND)"
              defaultValue={initialSearchTerm}
              fullWidth
            />
          </Stack>
          <Stack direction="row" height="100%" overflow="auto">
            <Box height="100%" sx={{ borderRight: "solid", borderColor: "#D3D3D3", borderWidth: 1 }}>
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
  }, [rootDataList, expanded, expandedFiltered, searchTerm, selectedItem, topicForSelected]);
  return createPanel;
}

TopicsPanel.propTypes = {
  initialSearchTerm: PropTypes.string,
};

export default TopicsPanel;
