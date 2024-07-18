import AbcIcon from "@mui/icons-material/Abc";
import DataArrayIcon from "@mui/icons-material/DataArray";
import DataObjectIcon from "@mui/icons-material/DataObject";
import Filter1Icon from "@mui/icons-material/Filter1";
import KeyboardArrowDownIcon from "@mui/icons-material/KeyboardArrowDown";
import NotesIcon from "@mui/icons-material/Notes";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PlaylistRemoveIcon from "@mui/icons-material/PlaylistRemove";
import StopIcon from "@mui/icons-material/Stop";
import {
  Alert,
  AlertTitle,
  Box,
  Button,
  Divider,
  FormControl,
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
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { JSONTree } from "react-json-tree";
import { v4 as uuid } from "uuid";
import { ProviderSelector } from "../../../components";
import { LoggingContext } from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { SubscriberFilter } from "../../../models";
import { EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX } from "../../../providers/eventTypes";
import { darkThemeJson } from "../../../themes/darkTheme";
import { lightThemeJson } from "../../../themes/lightTheme";

function TopicEchoPanel({
  showOptions = true,
  showDetails = false,
  defaultProvider = "",
  defaultTopic = "",
  defaultNoData = false,
}) {
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const [selectedProvider, setSelectedProvider] = useState(defaultProvider);
  const [currentProvider, setCurrentProvider] = useState(null);
  const [subscribed, setSubscribed] = useState(false);
  const [topicName, setTopic] = useState(defaultTopic);
  const [content, setContent] = useState(null);
  const [history, setHistory] = useState([]);
  const [showStatistics, setShowStatistics] = useState(true);
  const [noData, setNoData] = useState(defaultNoData);
  const [noStr, setNoStr] = useState(false);
  const [noArr, setNoArr] = useState(false);
  const [hz, setHz] = useState(1.0);
  // TODO add option to change window size to echo topics
  const [windowSize, setWindowSize] = useState(0);
  const [msgCount, setMsgCount] = useState(10);
  const [pause, setPause] = useState(false);
  // const [receivedIndex, setReceivedIndex] = useState(0);
  const [qosAnchorEl, setQosAnchorEl] = useState(null);
  const openQos = Boolean(qosAnchorEl);
  const handleQosClick = (event) => {
    setQosAnchorEl(event.currentTarget);
  };
  const handleQosClose = () => {
    setQosAnchorEl(null);
  };
  let receivedIndex = 0;
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  // set default topic if defined
  useEffect(() => {
    if (defaultTopic && defaultTopic.length > 0) {
      setTopic(defaultTopic);
    }
  }, [defaultTopic]);

  useEffect(
    () => {
      if (content) {
        setHistory((prev) => [content, ...prev.slice(0, msgCount - 1)]);
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [content]
  );

  useCustomEventListener(
    `${EVENT_PROVIDER_SUBSCRIBER_EVENT_PREFIX}_${topicName}`,
    (data) => {
      const { provider, event } = data;
      if (event === undefined) return;
      event.key = uuid();
      if (event?.data?.header?.seq) {
        event.seq = event.data.header.seq;
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
    const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize);
    rosCtx.updateFilterRosTopic(provider, topicName, filterMsg);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [noData, noArr, noStr, hz, windowSize]);
  // }, [rosCtx, selectedProvider, topicName, noData, noArr, noStr, hz, windowSize]);

  const close = useCallback(() => {
    if (rosCtx) {
      setSubscribed(false);
      logCtx.debug(`unregister subscriber to topic ${topicName}`);
      rosCtx.unregisterSubscriber(selectedProvider, topicName);
    }
  }, [rosCtx, logCtx, topicName, selectedProvider, setSubscribed]);

  useEffect(() => {
    if (subscribed || pause) return;
    // get current provider
    const provider = rosCtx.getProviderById(selectedProvider);
    if (!provider) return;
    setCurrentProvider(provider);

    let msgType = "";
    // Get messageType from node list of the provider
    const nodeList = rosCtx.mapProviderRosNodes.get(selectedProvider);
    // TODO: select QoS depending on publishers QoS, see choose_qos: https://github.com/ros2/ros2cli/blob/rolling/ros2topic/ros2topic/verb/echo.py
    nodeList.forEach((node) => {
      node.subscribers.forEach((topic) => {
        if (msgType === "" && topicName === topic.name) {
          msgType = topic.msgtype;
        }
      });
      if (msgType === "") {
        node.publishers.forEach((topic) => {
          if (msgType === "" && topicName === topic.name) {
            msgType = topic.msgtype;
          }
        });
      }
    });
    if (msgType) {
      logCtx.debug(`register subscriber to topic ${topicName}`);
      const filterMsg = new SubscriberFilter(noData, noArr, noStr, hz, windowSize);
      rosCtx.registerSubscriber(selectedProvider, topicName, msgType, filterMsg);
      setSubscribed(true);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topicName, rosCtx.mapProviderRosNodes, rosCtx.registerSubscriber, pause, subscribed, setSubscribed]);

  // initialize provider
  useEffect(() => {
    if (pause) {
      close();
    }
  }, [pause]);

  useEffect(() => {
    return () => {
      if (topicName) {
        close();
      }
    };
    // no dependencies: execute return statement on close this panel
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  function normalizePrint(size, fixed = 2, per = "") {
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
        <Box key={`box-${event.key}`}>
          <Stack marginTop={1} spacing={1} direction="row">
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
            <JSONTree
              key={`${event.key}`}
              data={event?.data}
              sortObjectKeys={true}
              theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
              invertTheme={false}
              hideRoot={true}
              shouldExpandNodeInitially={([key]) => {
                return true;
              }}
            />
          </Stack>
        </Box>
      );
    });
  }, [history, settingsCtx]);

  return (
    <Box width="100%" height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
      <Paper
        width="100%"
        elevation={1}
        sx={
          {
            // position: 'fixed',
            // borderTop: 1,
          }
        }
        overflow="auto"
      >
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography fontWeight="bold">{topicName}</Typography>
          <Typography color="grey" fontSize="0.8em">
            {currentProvider?.name()}
          </Typography>
        </Stack>
        <Stack>
          {showOptions && (
            <Stack spacing={0.5} margin={0.5} direction="row">
              <Tooltip
                title="show message data"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <ToggleButton size="small" value="noData" selected={!noData} onChange={() => setNoData(!noData)}>
                  <DataObjectIcon />
                </ToggleButton>
              </Tooltip>
              <Tooltip title="show arrays" placement="bottom" enterDelay={tooltipDelay} enterNextDelay={tooltipDelay}>
                <ToggleButton size="small" value="noArr" selected={!noArr} onChange={() => setNoArr(!noArr)}>
                  <DataArrayIcon />
                </ToggleButton>
              </Tooltip>
              <Tooltip title="show strings" placement="bottom" enterDelay={tooltipDelay} enterNextDelay={tooltipDelay}>
                <ToggleButton size="small" value="noStr" selected={!noStr} onChange={() => setNoStr(!noStr)}>
                  <AbcIcon />
                </ToggleButton>
              </Tooltip>
              <Tooltip
                title="limit message forward to 1 Hz"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <ToggleButton
                  size="small"
                  value="noStr"
                  selected={hz === 1.0}
                  onChange={() => setHz(hz === 1.0 ? 0.0 : 1.0)}
                >
                  <Filter1Icon />
                </ToggleButton>
              </Tooltip>
              <Divider orientation="vertical" />
              <Tooltip
                title="show statistics"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <ToggleButton
                  size="small"
                  value="statistics"
                  selected={showStatistics}
                  onChange={() => setShowStatistics(!showStatistics)}
                >
                  <NotesIcon />
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
              >
                <IconButton
                  size="medium"
                  onClick={() => {
                    setPause(!pause);
                  }}
                >
                  {pause ? <PlayArrowIcon /> : <StopIcon />}
                </IconButton>
              </Tooltip>
              <Tooltip
                title="clear messages"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <IconButton
                  size="medium"
                  onClick={() => {
                    setHistory([]);
                  }}
                >
                  <PlaylistRemoveIcon />
                </IconButton>
              </Tooltip>
              <Divider orientation="vertical" />
              <Tooltip
                title="count of displayed messages"
                placement="right"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <Select
                  id="select-msg-count"
                  autoWidth={false}
                  value={msgCount.toString()}
                  onChange={(event) => {
                    setMsgCount(event.target.value);
                  }}
                  size="small"
                >
                  {[1, 10, 20, 50, 100].map((value) => {
                    return (
                      <MenuItem key={`msg-count-${value}`} value={value}>
                        {value.toString()}
                      </MenuItem>
                    );
                  })}
                </Select>
              </Tooltip>
              {/* <FormControl disabled sx={{ m: 1, pt: 0.5 }} variant="standard">
                <ProviderSelector
                  defaultProvider={selectedProvider}
                  setSelectedProvider={(provId) => setSelectedProvider(provId)}
                />
              </FormControl> */}
            </Stack>
          )}
        </Stack>
        {content && showStatistics && (
          <Stack margin={0.5} spacing={1}>
            <Stack spacing={1} direction="row" fontSize="0.8em">
              {content.latched && (
                <Box
                  style={{
                    fontWeight: "bold",
                  }}
                >
                  latched
                </Box>
              )}
              <Box>{content.count} messages</Box>
              <Box>average rate: {content.rate.toFixed(2)} Hz</Box>
            </Stack>
            <Stack spacing={1} direction="row" fontSize="0.8em">
              <Box>
                bw: {normalizePrint(content.bw, 2, "/s")} [min: {normalizePrint(content.bw_min, 0, "/s")}, max:{" "}
                {normalizePrint(content.bw_max, 0, "/s")}]
              </Box>
              <Box>
                size: {normalizePrint(content.size, 2)} [min: {normalizePrint(content.size_min, 0)}, max:{" "}
                {normalizePrint(content.size_max, 0)}]
              </Box>
            </Stack>
          </Stack>
        )}
      </Paper>
      <Stack sx={{ width: "100%" }}>
        {history && !noData && generateJsonTopics}
        {!currentProvider && (
          <Alert severity="info" style={{ minWidth: 0, marginTop: 10 }}>
            <AlertTitle>{`Invalid provider: [${selectedProvider}]`}</AlertTitle>
            Please report this bug.
          </Alert>
        )}
      </Stack>
    </Box>
  );
}

TopicEchoPanel.propTypes = {
  showOptions: PropTypes.bool,
  showDetails: PropTypes.bool,
  defaultProvider: PropTypes.string,
  defaultTopic: PropTypes.string,
  defaultNoData: PropTypes.bool,
};

export default TopicEchoPanel;
