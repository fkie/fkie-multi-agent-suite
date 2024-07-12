import { SimpleTreeView } from '@mui/x-tree-view';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useMemo, useState } from 'react';

import { Box, IconButton, Stack, Tooltip } from '@mui/material';

import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import ChatBubbleOutlineIcon from '@mui/icons-material/ChatBubbleOutline';
import PlayCircleOutlineIcon from '@mui/icons-material/PlayCircleOutline';

import RefreshIcon from '@mui/icons-material/Refresh';

import { emitCustomEvent } from 'react-custom-events';
import { SearchBar, TopicTreeItem } from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { CmdType } from '../../../providers';
import {
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
} from '../../../utils/events';
import { findIn } from '../../../utils/index';
import { LAYOUT_TAB_SETS, LayoutTabConfig } from '../layout';
import TopicEchoPanel from './TopicEchoPanel';
import TopicPublishPanel from './TopicPublishPanel';

class TopicExtendedInfo {
  id;

  name;

  msgtype = '';

  providerId = '';

  providerName = '';

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

function TopicsPanel({ initialSearchTerm = '' }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topics, setTopics] = useState([]); // [topicInfo: TopicExtendedInfo]
  const [filteredTopics, setFilteredTopics] = useState([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState([]);
  const [expanded, setExpanded] = useState([]);
  const [topicForSelected, setTopicForSelected] = useState(null);
  const [selectedItems, setSelectedItems] = useState('');

  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  function genKey(items) {
    return `${items.join('#')}`;
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
            const topicInfo = newTopicsMap.get(
              genKey([rosTopic.name, rosTopic.msgtype, providerId]),
            );
            if (topicInfo) {
              topicInfo.add(rosTopic);
            } else {
              newTopicsMap.set(
                genKey([rosTopic.name, rosTopic.msgtype, rosNode.providerId]),
                new TopicExtendedInfo(
                  rosTopic,
                  rosNode.providerId,
                  rosNode.providerName,
                ),
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
  }, [rosCtx.initialized, rosCtx.mapProviderRosNodes]);

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

  // get group name from id of group tree item
  const fromGroupId = (id) => {
    if (id.endsWith('#')) {
      const trimmed = id.slice(0, -1);
      return {
        groupName: trimmed.substr(
          trimmed.lastIndexOf('/') + 1,
          trimmed.length - 1,
        ),
        fullPrefix: trimmed.substr(0, id.lastIndexOf('/')),
      };
    }
    return { groupName: id, fullPrefix: id };
  };

  // create id for group tree item
  const toGroupId = (groupName, fullPrefix) => {
    return `${fullPrefix}/${groupName}#`;
  };

  // create tree based on topic namespace
  // topics are grouped only if more then one is in the group
  const fillTree = (fullPrefix, topicsGroup) => {
    const byPrefixP1 = new Map('', []);
    // create a map with simulated tree for the namespaces of the topic list
    // for (const [key, topicInfo] of Object.entries(topics)) {
    Object.entries(topicsGroup).forEach(([key, topicInfo]) => {
      const nameSuffix = topicInfo.id.slice(fullPrefix.length + 1);
      const [firstName, ...restName] = nameSuffix.split('/');
      if (restName.length > 0) {
        const groupName = firstName;
        const restNameSuffix = restName.join('/');
        const groupId = toGroupId(groupName, fullPrefix);
        if (byPrefixP1.has(groupId)) {
          byPrefixP1.get(groupId).push({ restNameSuffix, topicInfo });
        } else {
          byPrefixP1.set(groupId, [{ restNameSuffix, topicInfo }]);
        }
      } else {
        byPrefixP1.set(firstName, [{ topicInfo }]);
      }
    });

    let count = 0;
    // create result
    const newFilteredTopics = [];
    byPrefixP1.forEach((value, key) => {
      // don't create group with one parameter
      if (value.length > 1) {
        const { groupName } = fromGroupId(key);
        const groupTopics = value.map((item) => {
          return item.topicInfo;
        });
        const subResult = fillTree(`${fullPrefix}/${groupName}`, groupTopics);
        // the result count is 0 -> we added multiple provider for topic with same name.
        if (subResult.count === 0) {
          count += 1;
        } else {
          count += subResult.count;
        }
        newFilteredTopics.push([
          {
            groupKey: key,
            topics: subResult.topics,
            count: subResult.count,
          },
        ]);
      } else {
        newFilteredTopics.push(value[0].topicInfo);
        if (value[0].topicInfo.providerName !== key) {
          // since the same topic can be on multiple provider
          // we count only topics
          count += 1;
        }
      }
    });
    return { topics: newFilteredTopics, count };
  };

  // create topics tree from filtered topic list
  useEffect(() => {
    const tree = fillTree('', filteredTopics);
    setRootDataList(tree.topics);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [filteredTopics]);

  const onEchoClick = useCallback((topic) => {
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `echo-${topic.name}-${topic.providerId}`,
        topic.name,
        <TopicEchoPanel
          showOptions
          defaultProvider={topic.providerId}
          defaultTopic={topic.name}
          defaultNoData={false}
        />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(true, CmdType.ECHO, {
          type: CmdType.ECHO,
          providerId: topic.providerId,
          topicName: topic.name,
        }),
      ),
    );
  }, []);

  const onPublishClick = useCallback((topic) => {
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `publish-${topic.name}-${topic.providerId}`,
        topic.name,
        <TopicPublishPanel
          topicName={topic.name}
          providerId={topic.providerId}
        />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(true, 'publish'),
      ),
    );
  }, []);

  const topicTreeToStyledItems = useCallback((rootPath, treeItem) => {
    if (Array.isArray(treeItem)) {
      return treeItem.map((item) => {
        const { groupName, fullPrefix } = fromGroupId(item.groupKey);
        const newRootName = `${fullPrefix}/${groupName}`;
        return (
          <TopicTreeItem
            key={item.groupKey}
            itemId={item.groupKey}
            labelRoot={rootPath}
            labelText={`${groupName}`}
            labelCount={item.count}
          >
            {item.topics.map((subItem) => {
              return topicTreeToStyledItems(newRootName, subItem);
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
      />
    );
  }, []);

  useEffect(() => {
    const selectedTopics = filteredTopics.filter((item) => {
      return (
        genKey([item.name, item.msgtype, item.providerId]) === selectedItems
      );
    });
    if (selectedTopics?.length >= 0) {
      setTopicForSelected(selectedTopics[0]);
    } else {
      setTopicForSelected(null);
    }
  }, [filteredTopics, selectedItems]);

  const createTreeView = useMemo(() => {
    return (
      <SimpleTreeView
        aria-label="parameters"
        expandedItems={expanded}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        onSelectedItemsChange={(event, itemId) => {
          setSelectedItems(itemId);
          const index = expanded.indexOf(itemId);
          const copyExpanded = [...expanded];
          if (index === -1) {
            copyExpanded.push(itemId);
          } else {
            copyExpanded.splice(index, 1);
          }
          setExpanded(copyExpanded);
        }}
      >
        {rootDataList.map((item) => {
          return topicTreeToStyledItems('', item);
        })}
      </SimpleTreeView>
    );
  }, [expanded, rootDataList, topicTreeToStyledItems]);

  const createPanel = useMemo(() => {
    return (
      <Box
        height="100%"
        overflow="auto"
        backgroundColor={settingsCtx.get('backgroundColor')}
      >
        <Stack
          spacing={1}
          height="100%"
          // sx={{
          //   height: '100%',
          //   display: 'flex',
          // }}
        >
          <Stack direction="row" spacing={1} alignItems="center">
            <Stack direction="row" paddingTop="5px">
              <Tooltip
                title="Echo"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
                disableInteractive
              >
                <span>
                  <IconButton
                    disabled={!topicForSelected}
                    size="small"
                    aria-label="Subscribe to topic and show the output"
                    onClick={() => {
                      onEchoClick(topicForSelected);
                    }}
                  >
                    <ChatBubbleOutlineIcon fontSize="inherit" />
                  </IconButton>
                </span>
              </Tooltip>
              <Tooltip
                title="Publish to"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
                disableInteractive
              >
                <span>
                  <IconButton
                    disabled={!topicForSelected}
                    size="small"
                    aria-label="Create a publisher"
                    onClick={() => {
                      onPublishClick(topicForSelected);
                    }}
                  >
                    <PlayCircleOutlineIcon fontSize="inherit" />
                  </IconButton>
                </span>
              </Tooltip>
            </Stack>

            <SearchBar
              onSearch={onSearch}
              placeholder="Filter Topics (<space> for OR, + for AND)"
              defaultValue={initialSearchTerm}
              fullWidth
            />
            <Tooltip
              title="Reload topic list"
              placement="left"
              disableInteractive
            >
              <IconButton
                size="small"
                onClick={() => {
                  getTopicList();
                }}
              >
                <RefreshIcon sx={{ fontSize: 'inherit' }} />
              </IconButton>
            </Tooltip>
          </Stack>
          <Stack direction="row" height="100%" overflow="auto">
            <Box width="100%" height="100%" overflow="auto">
              {createTreeView}
            </Box>
          </Stack>
        </Stack>
      </Box>
    );
  }, [rootDataList, expanded, selectedItems, topicForSelected]);
  return createPanel;
}

TopicsPanel.propTypes = {
  initialSearchTerm: PropTypes.string,
};

export default TopicsPanel;
