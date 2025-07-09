import AbcIcon from "@mui/icons-material/Abc";
import DataArrayIcon from "@mui/icons-material/DataArray";
import DataObjectIcon from "@mui/icons-material/DataObject";
import Filter1Icon from "@mui/icons-material/Filter1";
import KeyboardArrowDownIcon from "@mui/icons-material/KeyboardArrowDown";
import NotesIcon from "@mui/icons-material/Notes";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PlaylistRemoveIcon from "@mui/icons-material/PlaylistRemove";
import SearchIcon from "@mui/icons-material/Search";
import StopIcon from "@mui/icons-material/Stop";
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
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import JsonView from "react18-json-view";
import { v4 as uuid } from "uuid";

import { CopyButton, Tag } from "@/renderer/components/UI";
import { colorFromHostname } from "@/renderer/components/UI/Colors";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { RosNode, RosQos, SubscriberEvent, SubscriberFilter } from "@/renderer/models";
import { qosFromJson } from "@/renderer/models/RosQos";
import { Provider } from "@/renderer/providers";
import { EventProviderSubscriberEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils";

interface TSubscriberEventExt extends SubscriberEvent {
  key: string;
  seq?: number;
  receivedIndex: number;
}

interface TopicEchoPanelProps {
  showOptions: boolean;
  defaultProvider: string;
  defaultTopic: string;
  defaultNoData: boolean;
}

const TopicEchoPanel = forwardRef<HTMLDivElement, TopicEchoPanelProps>(function TopicEchoPanel(props, ref) {
  const { showOptions = true, defaultProvider = "", defaultTopic = "", defaultNoData = false } = props;

  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const [selectedProvider] = useState(defaultProvider);
  const [currentProvider, setCurrentProvider] = useState<Provider>();
  const [subscribed, setSubscribed] = useState(false);
  const [topicName, setTopic] = useState(defaultTopic);
  const [topicType, setTopicType] = useState<string>("");
  const [content, setContent] = useState<TSubscriberEventExt>();
  const [collapsedKeys, setCollapsedKeys] = useState<(string | number)[]>(["stamp", "covariance"]);
  const [history, setHistory] = useState<TSubscriberEventExt[]>([]);
  const [showStatistics, setShowStatistics] = useState(true);
  const [showSearchBar, setShowSearchBar] = useState(false);
  const [filterText, setFilterText] = useState("");
  const [noData, setNoData] = useState(defaultNoData);
  const [noStr, setNoStr] = useState(false);
  const [noArr, setNoArr] = useState(false);
  const [hz, setHz] = useState(1.0);
  // TODO add option to change window size to echo topics
  const [windowSize /*, setWindowSize*/] = useState<number>(0);
  const [msgCount, setMsgCount] = useState<number>(10);
  const [arrayItemsCount, setArrayItemsCount] = useLocalStorage<number>(`TopicEcho:arrayLimit:${defaultTopic}`, 15);
  const [pause, setPause] = useState<boolean>(false);
  // const [receivedIndex, setReceivedIndex] = useState(0);
  const [qosAnchorEl, setQosAnchorEl] = useState(null);
  const [currentQos, setCurrentQos] = useState<RosQos | undefined>(undefined);
  const openQos = Boolean(qosAnchorEl);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

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
  let receivedIndex = 0;

  // set default topic if defined
  useEffect(() => {
    if (defaultTopic && defaultTopic.length > 0) {
      setTopic(defaultTopic);
    }
  }, [defaultTopic]);

  useEffect(() => {
    if (content) {
      setHistory((prev) => [content, ...prev.slice(0, msgCount - 1)]);
    }
  }, [content]);

  useCustomEventListener(
    `${EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX}_${topicName}`,
    (data: EventProviderSubscriberEvent) => {
      const event: TSubscriberEventExt = data.event as TSubscriberEventExt;
      if (event === undefined) return;
      event.key = uuid();
      if ((event.data?.header as { seq: number })?.seq) {
        event.seq = (event.data?.header as { seq: number }).seq;
      }
      event.receivedIndex = receivedIndex;
      receivedIndex += 1;
      setContent(event);
    },
    [topicName]
  );

  // initialize provider
  useEffect(() => {
    const provider = rosCtx.getProviderById(selectedProvider);
    if (!provider) return;
    const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize, arrayItemsCount);
    rosCtx.updateFilterRosTopic(provider, topicName, filterMsg);
  }, [noData, noArr, noStr, hz, windowSize, arrayItemsCount]);

  function close(): void {
    if (rosCtx) {
      setSubscribed(false);
      logCtx.debug(`unregister subscriber to topic ${topicName}`);
      rosCtx.unregisterSubscriber(selectedProvider, topicName);
    }
  }

  useEffect(() => {
    if (subscribed || pause) return;
    // get current provider
    const provider = rosCtx.getProviderById(selectedProvider);
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
      rosCtx.registerSubscriber(selectedProvider, topicName, msgType, filterMsg, qos);
      setSubscribed(true);
    }
  }, [topicName, rosCtx.mapProviderRosNodes, rosCtx.registerSubscriber, pause, subscribed, setSubscribed]);

  // initialize provider
  useEffect(() => {
    if (pause) {
      close();
    }
  }, [pause]);

  useEffect(() => {
    return (): void => {
      if (topicName) {
        close();
      }
    };
    // no dependencies: execute return statement on close this panel
  }, []);

  function normalizePrint(size: number, fixed: number = 2, per: string = ""): string {
    if (size > 999999) {
      return `${(size / 1048576.0).toFixed(fixed)}MiB${per}`;
    }
    if (size > 999) {
      return `${(size / 1024.0).toFixed(fixed)}KiB${per}`;
    }
    return `${size.toFixed(fixed)}B${per}`;
  }

  function isObject(item: object | Array<object> | null): boolean {
    return (item && typeof item === "object") || Array.isArray(item);
  }

  function filterJson(data: object | Array<object> | null, filter: string): object | Array<object> | null {
    if (filter.length < 2) {
      return data;
    }
    const result = {};
    if (isObject(data)) {
      for (const key in data) {
        if (isObject(data[key])) {
          const res = filterJson(data[key], filter);
          if (res && Object.keys(res).length > 0) {
            result[key] = res;
          }
        } else {
          if (findIn(filter, [key, JSON.stringify(data[key])])) {
            result[key] = data[key];
          }
        }
      }
    }
    return result;
  }

  function topicsToJson(): JSX.Element[] {
    return history.map((event) => {
      return (
        // <Box key={`box-${event.key}`}>
        <Stack key={`box-${event.key}`} marginTop={1} spacing={1} direction="row">
          {/* <Tag
            key={event.receivedIndex}
            color="info"
            title={`${event.receivedIndex}`}
          /> */}
          {event.seq === undefined && (
            <Typography fontStyle="italic" fontSize="0.8em" color="gray">
              {event.receivedIndex}:
            </Typography>
          )}
          {event.seq !== undefined && (
            <Typography fontStyle="italic" fontSize="0.8em" color="gray">
              {event.seq}
            </Typography>
            // <Tag
            //   key={event.seq}
            //   color="info"
            //   title="seq: "
            //   text={`${event.seq}`}
            // />
          )}
          <JsonView
            key={`${event.key}`}
            src={filterJson(event?.data, filterText)}
            dark={settingsCtx.get("useDarkMode") as boolean}
            theme="a11y"
            enableClipboard={false}
            ignoreLargeArray={false}
            collapseObjectsAfterLength={3}
            displaySize={"collapsed"}
            collapsed={(params: {
              node: Record<string, unknown> | Array<unknown>; // Object or array
              indexOrName: number | string | undefined;
              depth: number;
              size: number; // Object's size or array's length
            }) => {
              if (params.indexOrName === undefined) return false;
              const idx = Number.isInteger(params.indexOrName) ? JSON.stringify(params.node) : params.indexOrName;
              return collapsedKeys.includes(idx);
            }}
            onCollapse={(params: {
              isCollapsing: boolean;
              node: Record<string, unknown> | Array<unknown>;
              indexOrName: string | number | undefined;
              depth: number;
            }) => {
              if (params.indexOrName !== undefined) {
                const idx = Number.isInteger(params.indexOrName) ? JSON.stringify(params.node) : params.indexOrName;
                if (!params.isCollapsing) {
                  if (!collapsedKeys.includes(idx)) {
                    setCollapsedKeys((prev) => [...prev, idx as string | number]);
                  }
                } else {
                  if (collapsedKeys.includes(idx)) {
                    setCollapsedKeys((prev) => prev.filter((item) => item !== idx));
                  }
                }
              }
            }}
          />
        </Stack>
        // </Box>
      );
    });
  }

  const generateJsonTopics = useMemo(() => {
    return topicsToJson();
  }, [history, settingsCtx.changed, collapsedKeys, filterText]);

  const generateOptions = useMemo(() => {
    return (
      <Stack spacing={0.5} margin={0.5} direction="row">
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
          <ToggleButton size="small" value="noStr" selected={hz === 1.0} onChange={() => setHz(hz === 1.0 ? 0.0 : 1.0)}>
            <Filter1Icon sx={{ fontSize: "inherit" }} />
          </ToggleButton>
        </Tooltip>
        <Divider orientation="vertical" />
        <Tooltip
          title="show statistics"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <ToggleButton
            size="small"
            value="statistics"
            selected={showStatistics}
            onChange={() => setShowStatistics(!showStatistics)}
          >
            <NotesIcon sx={{ fontSize: "inherit" }} />
          </ToggleButton>
        </Tooltip>
        <Tooltip
          title="show search bar"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <ToggleButton
            size="small"
            value="search bar"
            selected={showSearchBar}
            onChange={() => {
              setShowSearchBar(!showSearchBar);
              setFilterText("");
            }}
            autoFocus
          >
            <SearchIcon sx={{ fontSize: "inherit" }} />
          </ToggleButton>
        </Tooltip>
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
              setPause(!pause);
            }}
          >
            {pause ? <PlayArrowIcon sx={{ fontSize: "inherit" }} /> : <StopIcon sx={{ fontSize: "inherit" }} />}
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
        <Divider orientation="vertical" />
        <Tooltip
          title="count of displayed messages"
          placement="right"
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
        <Tooltip
          title="count of displayed array values"
          placement="right"
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
        {history.length > 0 && (
          <Tooltip
            title="Copy last message with 'ros2 topic pub' command"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
            disableInteractive
          >
            <pre>
              <CopyButton
                value={`ROS_DOMAIN_ID=${currentProvider?.rosState.ros_domain_id || 0} ros2 topic pub -1 --keep-alive 3 ${currentQos ? qosFromJson(currentQos).toString() : ""} ${topicName} ${topicType} '${JSON.stringify(history[0])}'`}
                logText="ros2 pub string copied"
              />
            </pre>
          </Tooltip>
        )}
        {/* <FormControl disabled sx={{ m: 1, pt: 0.5 }} variant="standard">
        <ProviderSelector
          defaultProvider={selectedProvider}
          setSelectedProvider={(provId) => setSelectedProvider(provId)}
        />
      </FormControl> */}
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
    history,
    topicType,
    currentQos,
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
      ref={ref}
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
          {content && showStatistics && (
            <Stack margin={0.5} spacing={0}>
              <Stack spacing={1} direction="row" fontSize="0.8em">
                <Tag text={`${content.count}`} color="info" tooltip="count of received message" />
                {content.latched && (
                  <Typography variant="body2" style={{ fontWeight: "bold" }}>
                    latched
                  </Typography>
                )}
                <Typography variant="body2" style={{ fontWeight: "bold" }}>
                  rate:
                </Typography>
                <Typography variant="body2">{content.rate > -1 ? `${content.rate.toFixed(2)} Hz` : "n/a"}</Typography>
              </Stack>
              <Stack spacing={0} direction="column">
                <Stack spacing={1} direction="row">
                  <Typography variant="body2" style={{ fontWeight: "bold" }}>
                    size:
                  </Typography>
                  <Typography variant="body2">
                    {normalizePrint(content.size, 2)} [min: {normalizePrint(content.size_min, 0)}, max:{" "}
                    {normalizePrint(content.size_max, 0)}]
                  </Typography>
                </Stack>
                <Stack spacing={1} direction="row">
                  <Typography variant="body2" style={{ fontWeight: "bold" }}>
                    bw:
                  </Typography>
                  <Typography variant="body2">
                    {content.bw > -1 ? `${normalizePrint(content.bw, 2, "/s")} [min: ${normalizePrint(content.bw_min, 0, "/s")}, max: ${normalizePrint(content.bw_max, 0, "/s")}]` : "n/a"}
                  </Typography>
                </Stack>
              </Stack>
            </Stack>
          )}
          {showSearchBar && (
            <Stack spacing={1} direction="row">
              <SearchBar
                onSearch={(value) => {
                  setFilterText(value);
                }}
                placeholder="grep for (OR: <space>, AND: +, NOT: !)"
                defaultValue={filterText}
                // fullWidth
              />
            </Stack>
          )}
        </Paper>
        <Stack width="100%" height="100%" overflow="auto">
          {history && !noData && generateJsonTopics}
          {!currentProvider && (
            <Alert severity="info" style={{ minWidth: 0, marginTop: 10 }}>
              <Alert severity="info">Wait until the provider is initialized: [${selectedProvider}]</Alert>
            </Alert>
          )}
        </Stack>
      </Stack>
    </Stack>
  );
});

export default TopicEchoPanel;
