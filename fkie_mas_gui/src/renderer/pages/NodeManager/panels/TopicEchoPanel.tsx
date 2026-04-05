import AbcIcon from "@mui/icons-material/Abc";
import DataArrayIcon from "@mui/icons-material/DataArray";
import DataObjectIcon from "@mui/icons-material/DataObject";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import Filter1Icon from "@mui/icons-material/Filter1";
import KeyboardArrowDownIcon from "@mui/icons-material/KeyboardArrowDown";
import PauseIcon from "@mui/icons-material/Pause";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PlaylistRemoveIcon from "@mui/icons-material/PlaylistRemove";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import {
  Alert,
  Button,
  Divider,
  IconButton,
  Menu,
  MenuItem,
  Paper,
  Select,
  Stack,
  ToggleButton,
  Tooltip,
  Typography,
} from "@mui/material";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { v4 as uuid } from "uuid";

import { Tag } from "@/renderer/components/UI";
import { colorFromHostname } from "@/renderer/components/UI/Colors";
import SearchBar from "@/renderer/components/UI/SearchBar";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { RosNode, RosQos, SubscriberFilter, TSubscriberEventExt } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { EventProviderSubscriberEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX } from "@/renderer/providers/eventTypes";
import { TEventCollapsedState } from "../layout/events";
import MessageFrame from "./MessageFrame";

interface TopicEchoPanelProps {
  showOptions: boolean;
  provider: Provider;
  defaultTopic: string;
  defaultNoData: boolean;
}

export default function TopicEchoPanel(props: TopicEchoPanelProps): JSX.Element {
  const { showOptions, provider, defaultTopic = "", defaultNoData = false } = props;

  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();
  const settingsCtx = useSettingsContext();

  const [selectedProvider] = useState(provider.id);
  const [currentProvider, setCurrentProvider] = useState<Provider>();
  const [subscribed, setSubscribed] = useState(false);
  const [topicName, setTopic] = useState(defaultTopic);
  const [topicType, setTopicType] = useState<string>("");
  const [content, setContent] = useState<TSubscriberEventExt>();
  const [rootCollapsed, setRootCollapsed] = useState<boolean>(false);
  const [collapsedKeys, setCollapsedKeys] = useState<(string | number)[]>(["stamp", "covariance"]);
  const [history, setHistory] = useState<TSubscriberEventExt[]>([]);
  const [showStatistics, setShowStatistics] = useState(false);
  const [showSearchBar, setShowSearchBar] = useState(true);
  const [filterText, setFilterText] = useState("");
  const [showWholeFilteredMessage, setShowWholeFilteredMessage] = useLocalStorage<boolean>(
    "TopicEcho:showWholeFilteredMessage",
    false
  );
  const [noData, setNoData] = useState(defaultNoData);
  const [noStr, setNoStr] = useState(false);
  const [noArr, setNoArr] = useState(false);
  const [hz, setHz] = useState(1.0);
  // TODO add option to change window size to echo topics
  const [windowSize /*, setWindowSize*/] = useState<number>(0);
  const [msgCount, setMsgCount] = useState<number>(10);
  const [arrayItemsCount, setArrayItemsCount] = useLocalStorage<number>(`TopicEcho:arrayLimit:${defaultTopic}`, 15);
  const [pause, setPause] = useState<boolean>(false);
  const [event, setEvent] = useState<TSubscriberEventExt | undefined>();
  const [_currentSubscriberId, setCurrentSubscriberId] = useState<number>(0);
  const [_previousSubscriberId, setPreviousSubscriberId] = useState<number[]>([]);
  const [multipleSubscriberId, setMultipleSubscriberId] = useState<boolean>(false);
  const [receivedIndex, setReceivedIndex] = useState(0);
  const [qosAnchorEl, setQosAnchorEl] = useState(null);
  const [currentQos, setCurrentQos] = useState<RosQos | undefined>(undefined);
  const openQos = Boolean(qosAnchorEl);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  // let receivedIndex = 0;

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  function handleQosClick(event): void {
    setQosAnchorEl(event.currentTarget);
  }
  function handleQosClose(): void {
    setQosAnchorEl(null);
  }

  // set default topic if defined
  useEffect(() => {
    if (defaultTopic && defaultTopic.length > 0) {
      setTopic(defaultTopic);
    }
  }, [defaultTopic]);

  useEffect(() => {
    if (content && history[0]?.receivedIndex !== content.receivedIndex) {
      setHistory((prev) => [content, ...prev.slice(0, msgCount - 1)]);
    }
  }, [content, msgCount]);

  useEffect(() => {
    if (!event || event.receivedIndex !== -1) return;
    event.key = uuid();
    if ((event.data?.header as { seq: number })?.seq) {
      event.seq = (event.data?.header as { seq: number }).seq;
    }
    event.receivedIndex = receivedIndex;
    event.timestamp = Date.now();
    setReceivedIndex((prev) => prev + 1);
    setContent(event);
    setCurrentSubscriberId((prevId) => {
      if (event.subscriber_id !== prevId) {
        if (prevId !== 0) {
          setPreviousSubscriberId((prevIds) => {
            if (prevIds.includes(event.subscriber_id)) {
              setMultipleSubscriberId(true);
            }
            return [...prevIds.filter((a) => a !== prevId), prevId];
          });
        }
        return event.subscriber_id;
      }
      setMultipleSubscriberId(false);
      return prevId;
    });
  }, [event, receivedIndex]);

  useCustomEventListener(
    `${EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX}_${topicName}`,
    (data: EventProviderSubscriberEvent) => {
      const event: TSubscriberEventExt = data.event as TSubscriberEventExt;
      if (event === undefined) return;
      event.receivedIndex = -1;
      setEvent(event);
    },
    [topicName]
  );

  useCustomEventListener(
    `collapse_state_${topicName}`,
    (data: TEventCollapsedState) => {
      if (data.key === "") {
        setRootCollapsed(data.isCollapsed);
      } else {
        setCollapsedKeys((prev) => {
          if (data.isCollapsed) {
            return [...prev, data.key];
          }
          return prev.filter((item) => item !== data.key);
        });
      }
    },
    [topicName]
  );

  const resetStats = useCallback(() => {
    if (!provider) return;
    setContent(undefined);
    const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize, arrayItemsCount, true);
    rosCtx.updateFilterRosTopic(provider, topicName, filterMsg);
  }, [provider, noData, noArr, noStr, hz, windowSize, arrayItemsCount, setContent]);

  // initialize provider
  useEffect(() => {
    if (!provider) return;
    const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize, arrayItemsCount);
    rosCtx.updateFilterRosTopic(provider, topicName, filterMsg);
  }, [provider, noData, noArr, noStr, hz, windowSize, arrayItemsCount]);

  const close = useCallback(() => {
    if (rosCtx) {
      setSubscribed(false);
      logCtx.debug(`unregister subscriber to topic ${topicName}`);
      rosCtx.unregisterSubscriber(provider, topicName);
    }
  }, [provider, rosCtx]);

  useEffect(() => {
    if (subscribed || pause) return;
    // get current provider
    if (!provider) return;
    setCurrentProvider(provider);

    let msgType = "";
    // Get messageType from node list of the provider
    const nodeList: RosNode[] | undefined = rosCtx.mapProviderRosNodes.get(selectedProvider);
    // TODO: select QoS depending on publishers QoS, see choose_qos: https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/verb/echo.py
    let qos: RosQos | undefined = undefined;
    for (const node of nodeList || []) {
      for (const topic of node.subscribers || []) {
        if (msgType === "" && topicName === topic.name) {
          msgType = topic.msg_type;
        }
      }
      if (msgType === "") {
        for (const topic of node.publishers || []) {
          if (topicName === topic.name) {
            if (msgType === "") {
              msgType = topic.msg_type;
            }
          }
        }
      }
    }
    for (const topic of provider.rosTopics) {
      if (topic.name === topicName) {
        for (const pub of topic.publisher || []) {
          if (qos === undefined && pub.qos) {
            qos = pub.qos;
          }
        }
      }
    }
    setTopicType(msgType);
    setCurrentQos(qos);
    if (msgType) {
      logCtx.debug(`register subscriber to topic ${topicName}`);
      const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize, arrayItemsCount);
      rosCtx.registerSubscriber(provider, topicName, msgType, filterMsg, qos);
      setSubscribed(true);
    }
  }, [provider, topicName, rosCtx.mapProviderRosNodes, pause, subscribed]);

  // useEffect(() => {
  //   updateSubscriberNodeState();
  // }, [rosCtx.mapProviderRosNodes]);

  useEffect(() => {
    return (): void => {
      if (topicName) {
        close();
      }
    };
    // no dependencies: execute return statement on close this panel
  }, []);

  // const updateSubscriberNodeState = useCallback(async () => {
  //   const result = await currentProvider?.getCountTopicSubscriptions(topicName);
  //   if ((result?.length || 0) > 0) {
  //     setPause(false);
  //   } else {
  //     setPause(true);
  //   }
  // }, [topicName, currentProvider, rosCtx.mapProviderRosNodes]);

  function normalizePrint(size: number | undefined, fixed: number = 2, per: string = ""): string {
    if (size === undefined) return "n/a";
    if (size > 999999) {
      return `${(size / 1048576.0).toFixed(fixed)}MiB${per}`;
    }
    if (size > 999) {
      return `${(size / 1024.0).toFixed(fixed)}KiB${per}`;
    }
    return `${size.toFixed(fixed)}B${per}`;
  }

  const generateJsonTopics = useMemo(() => {
    return history.map((event) => {
      return (
        <MessageFrame
          key={event.key}
          event={event}
          domainId={`${currentProvider?.connection.domainId}` || "0"}
          qos={currentQos}
          filter={filterText}
          initRootCollapsed={rootCollapsed}
          initCollapsed={collapsedKeys}
        />
      );
    });
  }, [history, settingsCtx.changed, collapsedKeys, filterText, showWholeFilteredMessage]);

  const createStatistics = useMemo(() => {
    return (
      <Stack margin={0} spacing={0}>
        <Stack
          direction="row"
          alignItems="center"
          onClick={() => {
            setShowStatistics((prev) => !prev);
          }}
        >
          <ExpandMoreIcon style={{ transform: `${showStatistics ? "" : "rotate(-90deg)"}` }} fontSize="small" />
          <Button
            size="small"
            style={{
              textTransform: "none",
              color: "inherit",
            }}
          >
            <Stack spacing={1} direction="row" fontSize="0.8em" alignItems="center">
              <Tag text={`${content?.count || 0}`} color="info" tooltip="count of received message" />
              {content?.latched && (
                <Typography variant="body2" style={{ fontWeight: "bold" }}>
                  latched
                </Typography>
              )}
              <Typography variant="body2" style={{ fontWeight: "bold" }}>
                rate:
              </Typography>
              <Typography variant="body2">
                {(content?.rate || -2) > -1 ? `${content?.rate.toFixed(2)} Hz` : "n/a"}
              </Typography>
            </Stack>
          </Button>
          <Tooltip
            title="reset stats"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <IconButton
              size="small"
              onClick={(event) => {
                event.stopPropagation();
                resetStats();
              }}
            >
              <RestartAltIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>
        </Stack>
        {showStatistics && (
          <Stack marginLeft="0.7em" spacing={0} direction="column">
            <Stack spacing={1} direction="row">
              <Typography variant="body2" style={{ fontWeight: "bold" }}>
                size:
              </Typography>
              <Typography variant="body2">
                {(content?.size || -2) > -1
                  ? `${normalizePrint(content?.size, 2)} [min: ${normalizePrint(content?.size_min, 0)}, max: ${normalizePrint(content?.size_max, 0)}]`
                  : "n/a"}
              </Typography>
            </Stack>
            <Stack spacing={1} direction="row">
              <Typography variant="body2" style={{ fontWeight: "bold" }}>
                bw:
              </Typography>
              <Typography variant="body2">
                {(content?.bw || -2) > -1
                  ? `${normalizePrint(content?.bw, 2, "/s")} [min: ${normalizePrint(content?.bw_min, 0, "/s")}, max: ${normalizePrint(content?.bw_max, 0, "/s")}]`
                  : "n/a"}
              </Typography>
            </Stack>
          </Stack>
        )}
      </Stack>
    );
  }, [content, showStatistics]);

  const generateOptions = useMemo(() => {
    return (
      <Stack spacing={0.5} margin={0.5} direction="column">
        <Stack spacing={0.5} direction="row">
          <Tooltip
            title="show message data"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <ToggleButton size="small" value="noData" selected={!noData} onChange={() => setNoData(!noData)}>
              <DataObjectIcon sx={{ fontSize: "inherit" }} />
            </ToggleButton>
          </Tooltip>
          <Tooltip
            title="show arrays"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <ToggleButton size="small" value="noArr" selected={!noArr} onChange={() => setNoArr(!noArr)}>
              <DataArrayIcon sx={{ fontSize: "inherit" }} />
            </ToggleButton>
          </Tooltip>
          <Tooltip
            title="show strings"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <ToggleButton size="small" value="noStr" selected={!noStr} onChange={() => setNoStr(!noStr)}>
              <AbcIcon sx={{ fontSize: "inherit" }} />
            </ToggleButton>
          </Tooltip>
          <Tooltip
            title="limit message forward to 1 Hz"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <ToggleButton
              size="small"
              value="noStr"
              selected={hz === 1.0}
              onChange={() => setHz(hz === 1.0 ? 0.0 : 1.0)}
            >
              <Filter1Icon sx={{ fontSize: "inherit" }} />
            </ToggleButton>
          </Tooltip>

          <Tooltip
            title="count of displayed array values"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <Select
              id="select-array-count"
              autoWidth={false}
              value={arrayItemsCount.toString()}
              onChange={(event) => {
                setArrayItemsCount(Number.parseInt(event.target.value));
              }}
              size="small"
              sx={{ fontSize: "0.5em" }}
            >
              {[0, 5, 15, 25, 55, 155].map((value) => {
                return (
                  <MenuItem key={`array-count-${value}`} value={value} sx={{ fontSize: "0.5em" }}>
                    {value.toString()}
                  </MenuItem>
                );
              })}
            </Select>
          </Tooltip>
          <Tooltip
            title="count of displayed messages"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <Select
              id="select-msg-count"
              autoWidth={false}
              value={msgCount.toString()}
              onChange={(event) => {
                setMsgCount(Number.parseInt(event.target.value));
              }}
              size="small"
              sx={{ fontSize: "0.5em" }}
            >
              {[1, 10, 20, 50, 100].map((value) => {
                return (
                  <MenuItem key={`msg-count-${value}`} value={value} sx={{ fontSize: "0.5em" }}>
                    {value.toString()}
                  </MenuItem>
                );
              })}
            </Select>
          </Tooltip>
        </Stack>
        {createStatistics}
        <Stack margin={0.5} direction="row">
          {currentProvider?.rosVersion === "X" && (
            <>
              <Button
                id="qos-button"
                aria-controls={openQos ? "qos-menu" : undefined}
                aria-haspopup="true"
                aria-expanded={openQos ? "true" : undefined}
                variant="outlined"
                disableElevation
                onClick={handleQosClick}
                endIcon={<KeyboardArrowDownIcon />}
              >
                QoS
              </Button>
              <Menu
                id="qos-menu"
                MenuListProps={{
                  "aria-labelledby": "qos-button",
                }}
                anchorEl={qosAnchorEl}
                open={openQos}
                onClose={handleQosClose}
              >
                <MenuItem onClick={handleQosClose} disableRipple>
                  System Default
                </MenuItem>
                <MenuItem onClick={handleQosClose} disableRipple>
                  Sensor Data
                </MenuItem>
                <Divider sx={{ my: 0.5 }} />
                <MenuItem onClick={handleQosClose} disableRipple>
                  Services Default
                </MenuItem>
                <MenuItem onClick={handleQosClose} disableRipple>
                  Parameters
                </MenuItem>
                <MenuItem onClick={handleQosClose} disableRipple>
                  Parameter events
                </MenuItem>
                <MenuItem onClick={handleQosClose} disableRipple>
                  Action Status Default
                </MenuItem>
              </Menu>
            </>
          )}
          <Tooltip
            title={pause ? "start subscriber" : "stop subscriber"}
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <IconButton
              size="small"
              onClick={() => {
                setPause((prev) => {
                  if (!prev) {
                    close();
                  }
                  return !prev;
                });
              }}
            >
              {pause ? <PlayArrowIcon sx={{ fontSize: "inherit" }} /> : <PauseIcon sx={{ fontSize: "inherit" }} />}
            </IconButton>
          </Tooltip>
          <Tooltip
            title="clear messages"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <IconButton
              size="small"
              onClick={() => {
                setHistory([]);
              }}
            >
              <PlaylistRemoveIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>

          {showSearchBar && (
            <Stack spacing={1} direction="row" style={{ display: "flex", flexGrow: 1 }}>
              <SearchBar
                onSearch={(value) => {
                  setFilterText(value);
                }}
                placeholder="grep for (OR: <space>, AND: +, NOT: !)"
                defaultValue={filterText}
                searchIcon={
                  <Tooltip
                    title="shows complete message if search result is valid"
                    placement="bottom"
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                    disableInteractive
                  >
                    <ToggleButton
                      size="small"
                      value="noArr"
                      selected={showWholeFilteredMessage}
                      onChange={() => setShowWholeFilteredMessage((prev) => !prev)}
                      style={{ padding: 2, marginRight: "0.5em" }}
                    >
                      <DataObjectIcon sx={{ fontSize: "inherit" }} />
                    </ToggleButton>
                  </Tooltip>
                }
              />
            </Stack>
          )}
        </Stack>
      </Stack>
    );
  }, [
    currentProvider,
    noArr,
    noData,
    noStr,
    hz,
    pause,
    tooltipDelay,
    msgCount,
    arrayItemsCount,
    qosAnchorEl,
    openQos,
    showStatistics,
    showSearchBar,
    showWholeFilteredMessage,
    history,
    topicType,
    currentQos,
    content,
    showStatistics,
  ]);

  const getHostStyle = useCallback(
    function getHostStyle(): object {
      const providerName = currentProvider?.name();
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          flexGrow: 1,
          borderTopStyle: "solid",
          borderTopColor: colorFromHostname(providerName),
          borderTopWidth: "0.3em",
          backgroundColor: backgroundColor,
        };
      }
      return { flexGrow: 1, backgroundColor: backgroundColor };
    },
    [currentProvider, backgroundColor, settingsCtx.changed]
  );

  function onKeyDown(event: React.KeyboardEvent): void {
    if (event.ctrlKey && event.key === "f") {
      setShowSearchBar(!showSearchBar);
      setFilterText("");
    }
  }

  return (
    <Stack
      onKeyDown={(event) => onKeyDown(event)}
      height="100%"
      overflow="auto"
      alignItems="center"
      sx={getHostStyle()}
    >
      <Stack spacing={1} height="100%" width="100%">
        <Paper elevation={1}>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Typography fontWeight="bold">{topicName}</Typography>
            <Typography color="grey" fontSize="0.8em">
              {currentProvider?.name()}
            </Typography>
          </Stack>
          <Stack>{showOptions && generateOptions}</Stack>
        </Paper>
        {multipleSubscriberId && <Typography color="red">Multiple MAS-Subscriber detected!</Typography>}
        <Stack width="100%" height="100%" overflow="auto">
          {history && !noData && generateJsonTopics}
          {!currentProvider && (
            <Alert severity="info">Wait until the provider is initialized: [{selectedProvider}]</Alert>
          )}
        </Stack>
      </Stack>
    </Stack>
  );
}
