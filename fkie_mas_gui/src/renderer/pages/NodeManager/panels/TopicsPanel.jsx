import { styled } from '@mui/material/styles';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useState } from 'react';

import {
  Alert,
  AlertTitle,
  Box,
  IconButton,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Tooltip,
} from '@mui/material';

import MuiAccordion from '@mui/material/Accordion';
import MuiAccordionDetails from '@mui/material/AccordionDetails';
import MuiAccordionSummary from '@mui/material/AccordionSummary';

import ChatBubbleOutlineIcon from '@mui/icons-material/ChatBubbleOutline';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import PlayCircleOutlineIcon from '@mui/icons-material/PlayCircleOutline';
import RefreshIcon from '@mui/icons-material/Refresh';

import { emitCustomEvent } from 'react-custom-events';
import { SearchBar, Tag } from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { CmdType } from '../../../providers';
import { pathJoin } from '../../../utils';
import {
  EVENT_OPEN_COMPONENT,
  eventOpenComponent,
} from '../../../utils/events';
import { LAYOUT_TAB_SETS, LayoutTabConfig } from '../layout';
import OverflowMenuProviderSelector from './OverflowMenuProviderSelector';
import TopicEchoPanel from './TopicEchoPanel';
import TopicPublishPanel from './TopicPublishPanel';

const Accordion = styled((props) => (
  <MuiAccordion disableGutters elevation={0} square {...props} />
))(({ theme }) => ({
  border: 'none',
  backgroundColor:
    theme.palette.mode === 'dark'
      ? 'rgba(0, 0, 0, .00)'
      : 'rgba(255, 255, 255, .00)',
}));

const AccordionSummary = styled((props) => <MuiAccordionSummary {...props} />)(
  () => ({
    minHeight: '12px',
    '& .MuiAccordionSummary-content': {
      justifyContent: 'center',
      margin: 0,
    },
  }),
);

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  padding: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const headers = [
  {
    key: 'actions',
    header: 'Actions',
  },
  {
    key: 'name',
    header: 'Name',
  },
  {
    key: 'msgtype',
    header: 'Message Type',
  },
  {
    key: 'publisher',
    header: 'Publishers',
  },
  {
    key: 'subscriber',
    header: 'Subscribers',
  },
];

function TopicsPanel({ initialSearchTerm }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topics, setTopics] = useState([]);
  const [filteredTopics, setFilteredTopics] = useState([]);
  const [providers, setProviders] = useState({}); // {<topicName: string>: Map< <providerId: string>: <name: string>>}
  const [nodeKeyName, setNodeKeyName] = useState({});
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);

  const getTopicList = useCallback(async () => {
    if (rosCtx.mapProviderRosNodes) {
      // Get topics from the ros node list of each provider.
      const topicList = [];
      const providerList = {};
      const nodes = {};
      rosCtx.mapProviderRosNodes.forEach((nodeList, providerId) => {
        nodeList.forEach((node) => {
          nodes[node.id] = node.name;

          node.subscribers.forEach((topic) => {
            if (!topicList.find((t) => t.name === topic.name)) {
              topicList.push(topic);
              const newMap = new Map();
              newMap.set(node.providerId, node.providerName);
              providerList[topic.name] = newMap;
            } else {
              providerList[topic.name].set(node.providerId, node.providerName);
            }
          });
          node.publishers.forEach((topic) => {
            if (!topicList.find((t) => t.name === topic.name)) {
              topicList.push(topic);
              const newMap = new Map();
              newMap.set(node.providerId, node.providerName);
              providerList[topic.name] = newMap;
            } else {
              providerList[topic.name].set(node.providerId, node.providerName);
            }
          });
        });
      });
      // sort topics by name
      topicList.sort(function (a, b) {
        return a.name.localeCompare(b.name);
      });
      setNodeKeyName(nodes);
      setTopics(topicList);
      setFilteredTopics(topicList);
      setProviders(providerList);
    }
  }, [rosCtx.mapProviderRosNodes]);

  // debounced search callback
  const onSearch = useDebounceCallback((newSearchTerm) => {
    setSearchTerm(newSearchTerm);
    if (!newSearchTerm) {
      setFilteredTopics(topics);
      return;
    }

    const newFilteredTopics = topics.filter((topic) => {
      const matchName =
        topic.name.toLowerCase().indexOf(newSearchTerm.toLowerCase()) !== -1;
      if (matchName) {
        return true;
      }
      const matchType =
        topic.msgtype.toLowerCase().indexOf(newSearchTerm.toLowerCase()) !== -1;
      if (matchType) {
        return true;
      }
      const matchPub = topic.publisher.some((pub) =>
        pub.toLowerCase().includes(newSearchTerm.toLowerCase()),
      );
      if (matchPub) {
        return true;
      }
      const matchSub = topic.subscriber.some((sub) =>
        sub.toLowerCase().includes(newSearchTerm.toLowerCase()),
      );
      if (matchSub) {
        return true;
      }

      let matchProvider = false;
      providers[topic.name].forEach((providerName) => {
        if (
          providerName.toLowerCase().indexOf(searchTerm.toLowerCase()) !== -1
        ) {
          matchProvider = true;
        }
      });
      if (matchProvider) {
        return true;
      }
      return false;
    });

    setFilteredTopics(newFilteredTopics);
  }, 300);

  const onActionClick = useCallback(
    (actionType, providerId, providerName, topic) => {
      if (actionType === 'ECHO') {
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            `echo-${topic.name}-${providerId}`,
            topic.name,
            <TopicEchoPanel
              showOptions
              defaultRosTopicType={actionType}
              defaultProvider={providerId}
              defaultTopic={topic.name}
              defaultNoData={false}
            />,
            true,
            LAYOUT_TAB_SETS.BORDER_RIGHT,
            new LayoutTabConfig(true, CmdType.ECHO, {
              type: CmdType.ECHO,
              providerId,
              topicName: topic.name,
            }),
          ),
        );
      }
      if (actionType === 'PUBLISH') {
        emitCustomEvent(
          EVENT_OPEN_COMPONENT,
          eventOpenComponent(
            `publish-${topic.name}-${providerId}`,
            topic.name,
            <TopicPublishPanel
              topicName={topic.name}
              providerId={providerId}
            />,
            true,
            LAYOUT_TAB_SETS.BORDER_RIGHT,
            new LayoutTabConfig(true, 'publish'),
          ),
        );
      }
    },
    [],
  );
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

  return (
    <Box
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      {filteredTopics && (
        <Stack
          spacing={1}
          sx={{
            height: '100%',
            display: 'flex',
          }}
        >
          <Stack direction="row" spacing={1}>
            <SearchBar
              onSearch={onSearch}
              placeholder="Filter Topics"
              defaultValue={initialSearchTerm}
              fullWidth
            />
            <Tooltip title="Reload topic list" placement="left">
              <IconButton
                size="small"
                onClick={() => {
                  getTopicList();
                }}
              >
                <RefreshIcon sx={{ fontSize: 18 }} />
              </IconButton>
            </Tooltip>
          </Stack>

          {filteredTopics.length > 0 && (
            <TableContainer
              sx={{
                flex: 1,
                minHeight: 0,
              }}
            >
              <Table stickyHeader size="small" aria-label="topic table">
                <TableHead>
                  <TableRow>
                    {headers.map((header) => {
                      if (header.key === 'name') {
                        return (
                          <TableCell
                            key={header.key}
                            sx={{ fontWeight: 'bold' }}
                          >
                            {header.header} [{filteredTopics.length}]
                          </TableCell>
                        );
                      }
                      return (
                        <TableCell key={header.key} sx={{ fontWeight: 'bold' }}>
                          {header.header}
                        </TableCell>
                      );
                    })}
                  </TableRow>
                </TableHead>
                <TableBody>
                  {filteredTopics.map((row) => {
                    const colorPubSub =
                      row.publisher.length > 0 && row.subscriber.length > 0
                        ? 'default'
                        : 'warning';
                    const colorPubSubM =
                      row.publisher.length > 0 && row.subscriber.length > 0
                        ? 'default'
                        : 'warning';
                    return (
                      <TableRow key={row.id}>
                        <TableCell key={`${row.id}-actions`}>
                          <Stack direction="row">
                            <Tooltip title="Echo" enterDelay={1000}>
                              <div>
                                <OverflowMenuProviderSelector
                                  onClick={onActionClick}
                                  providerMap={providers[row.name]}
                                  icon={ChatBubbleOutlineIcon}
                                  actionType="ECHO"
                                  args={row}
                                />
                              </div>
                            </Tooltip>
                            <Tooltip title="Publish to" enterDelay={1000}>
                              <div>
                                <OverflowMenuProviderSelector
                                  onClick={onActionClick}
                                  providerMap={providers[row.name]}
                                  icon={PlayCircleOutlineIcon}
                                  actionType="PUBLISH"
                                  args={row}
                                />
                              </div>
                            </Tooltip>
                          </Stack>
                        </TableCell>
                        <TableCell key={`${row.id}-name`}>
                          <Stack direction="row">
                            {row.name}
                            {/* <CopyButton value={row.name} /> */}
                          </Stack>
                        </TableCell>
                        <TableCell key={`${row.id}-type`}>
                          <Stack direction="row">
                            {row.msgtype}
                            {/* <CopyButton value={row.msgtype} /> */}
                          </Stack>
                        </TableCell>
                        <TableCell key={`${row.id}-publisher`}>
                          {row.publisher.length === 1 && (
                            <Stack>
                              {row.publisher.map((el) => {
                                return (
                                <Tag
                                  color={colorPubSub}
                                  key={`${row.id}-pub-${el}`}
                                  text={nodeKeyName[el]}
                                  wrap={false}
                                />
                              )})}
                            </Stack>
                          )}
                          {row.publisher.length === 0 && (
                            <Stack>
                              <Tag
                                color="default"
                                key={`${row.id}-pub-0`}
                                text="0"
                              />
                            </Stack>
                          )}
                          {row.publisher.length > 1 && (
                            <Accordion>
                              <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                                <Tag
                                  color={colorPubSubM}
                                  key={`${row.id}-pub-m`}
                                  text={row.publisher.length.toString()}
                                />
                              </AccordionSummary>
                              {row.publisher.length > 1 && (
                                <AccordionDetails>
                                  <Stack spacing={0.5}>
                                    {row.publisher.map((el) => (
                                      <Tag
                                        color={colorPubSubM}
                                        key={`${row.id}-pub-${el}`}
                                        text={nodeKeyName[el]}
                                        wrap={false}
                                      />
                                    ))}
                                  </Stack>
                                </AccordionDetails>
                              )}
                            </Accordion>
                          )}
                        </TableCell>
                        <TableCell key={`${row.id}-subscriber`}>
                          {row.subscriber.length === 1 && (
                            <Stack>
                              {row.subscriber.map((el) => (
                                <Tag
                                  color={colorPubSub}
                                  key={`${row.id}-pub-${el}`}
                                  text={nodeKeyName[el]}
                                  wrap={false}
                                />
                              ))}
                            </Stack>
                          )}
                          {row.subscriber.length === 0 && (
                            <Stack>
                              <Tag
                                color="default"
                                key={`${row.id}-pub-0`}
                                text="0"
                              />
                            </Stack>
                          )}
                          {row.subscriber.length > 1 && (
                            <Accordion>
                              <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                                <Tag
                                  color={colorPubSubM}
                                  key={`${row.id}-pub-m`}
                                  text={row.subscriber.length.toString()}
                                />
                              </AccordionSummary>
                              {row.subscriber.length > 1 && (
                                <AccordionDetails>
                                  <Stack spacing={0.5}>
                                    {row.subscriber.map((el) => (
                                      <Tag
                                        color={colorPubSubM}
                                        key={`${row.id}-pub-${el}`}
                                        text={nodeKeyName[el]}
                                        wrap={false}
                                      />
                                    ))}
                                  </Stack>
                                </AccordionDetails>
                              )}
                            </Accordion>
                          )}
                        </TableCell>
                      </TableRow>
                    );
                  })}
                </TableBody>
              </Table>
            </TableContainer>
          )}

          {filteredTopics.length === 0 && (
            <Alert severity="info" style={{ minWidth: 0 }}>
              <AlertTitle>No Topics found</AlertTitle>
            </Alert>
          )}
        </Stack>
      )}
    </Box>
  );
}

TopicsPanel.defaultProps = {
  initialSearchTerm: '',
};

TopicsPanel.propTypes = {
  initialSearchTerm: PropTypes.string,
};

export default TopicsPanel;
