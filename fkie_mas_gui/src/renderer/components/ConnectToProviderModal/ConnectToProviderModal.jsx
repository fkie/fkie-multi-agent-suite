import { useCallback, useContext, useEffect, useMemo, useState } from "react";

import CheckBoxIcon from "@mui/icons-material/CheckBox";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import SettingsOutlinedIcon from "@mui/icons-material/SettingsOutlined";
import {
  AccordionSummary,
  // AccordionDetails,
  Autocomplete,
  Box,
  Button,
  Checkbox,
  CircularProgress,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  FormControlLabel,
  FormGroup,
  Grid,
  IconButton,
  Link,
  Paper,
  Radio,
  RadioGroup,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import MuiAccordion from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import { grey } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import { useCustomEventListener } from "react-custom-events";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext, getDefaultPortFromRos } from "../../context/SettingsContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import ProviderLaunchConfiguration from "../../models/ProviderLaunchConfiguration";
import Provider from "../../providers/Provider";
import { EVENT_PROVIDER_ROS_NODES } from "../../providers/eventTypes";
import { generateUniqueId } from "../../utils";
import { EVENT_OPEN_CONNECT } from "../../utils/events";
import CopyButton from "../UI/CopyButton";
import DraggablePaper from "../UI/DraggablePaper";

const Accordion = styled((props) => <MuiAccordion disableGutters elevation={0} square {...props} />)(({ theme }) => ({
  border: "none",
  backgroundColor: theme.palette.mode === "dark" ? "rgba(0, 0, 0, .00)" : "rgba(255, 255, 255, .00)",
}));

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  paddingTop: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;

const DEFAULT_PARAMETER = {
  rosVersion: 1,
  networkId: 0,
  daemon: {},
  discovery: {
    robotHosts: [],
    heartbeatHz: 0,
  },
  sync: {
    enable: false,
    doNotSync: [],
    syncTopics: [],
  },
  ttyd: {
    port: 7681,
  },
};

function ConnectToProviderModal() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);

  const [open, setOpen] = useState(false);
  const [openTerminalTooltip, setOpenTerminalTooltip] = useState(false);

  const [hostList, setHostList] = useState([]);
  const [hostValues, setHostValues] = useState([]);
  const [hostInputValue, setHostInputValue] = useState("");
  const [robotHostInputValue, setRobotHostInputValue] = useState("");

  DEFAULT_PARAMETER.rosVersion = settingsCtx.get("rosVersion");
  const [startParameterDefault, setStartConfigurationsDefault] = useLocalStorage(
    "ConnectToProviderModal:startParameter",
    DEFAULT_PARAMETER
  );
  const [startParameter, setStartParameter] = useState(startParameterDefault);
  const [startConfigurations, setStartConfigurations] = useLocalStorage(
    "ConnectToProviderModal:startConfigurations",
    []
  );
  const [selectedHistory, setSelectedHistory] = useState("");
  const [forceRestart, setForceRestart] = useState(true);
  const [saveDefaultParameter, setSaveDefaultParameter] = useState(false);
  const [enableDaemonNode, setEnableDaemonNode] = useState(true);
  const [enableDiscoveryNode, setEnableDiscoveryNode] = useState(true);
  const [enableSyncNode, setEnableSyncNode] = useState(startParameter.sync.enable);
  const [enableTerminalManager, setEnableTerminalManager] = useState(true);

  const [tsList, setTSList] = useState([]);
  const [topicList, setTopicList] = useState([]);

  const [startProviderStatus, setStartProviderStatus] = useState("");
  const [startProviderDescription, setStartProviderDescription] = useState("");
  const [startProviderIsSubmitting, setStartProviderIsSubmitting] = useState(false);

  const handleOpen = () => setOpen(true);
  const handleClose = (event, reason) => {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
    setOpenTerminalTooltip(false);
    setStartProviderIsSubmitting(false);
  };

  useEffect(() => {
    if (!rosCtx.systemInfo) return;
    if (!rosCtx.systemInfo.hosts) return;

    // update list of hosts
    const hostListLocal = [];
    const hostSystem = rosCtx.systemInfo.hosts;
    // only add valid ipv4 addresses
    const ipv4format =
      /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    hostSystem.forEach((h) => {
      const ip = h[0];
      if (ip.match(ipv4format)) {
        const lastName = h[1].split(" ").pop();
        hostListLocal.push({ ip, host: lastName });
      }
    });
    hostListLocal.sort((a, b) => -b.host.localeCompare(a.host));
    setHostList(hostListLocal);
    setHostValues(["localhost"]);
  }, [rosCtx.systemInfo]);

  useCustomEventListener(EVENT_PROVIDER_ROS_NODES, (data) => {
    // trigger add new provider
    const newAcTsSet = new Set();
    const newAcTopicSet = new Set();
    rosCtx.providersConnected.forEach((prov) => {
      prov.rosNodes.forEach((node) => {
        newAcTsSet.add(`${node.name}/*`);
        newAcTopicSet.add(`${node.name}/*`);
        node.publishers.forEach((topic) => {
          newAcTsSet.add(topic.name);
          newAcTsSet.add(topic.msgtype);
          newAcTopicSet.add(topic.name);
        });
        node.subscribers.forEach((topic) => {
          newAcTsSet.add(topic.name);
          newAcTsSet.add(topic.msgtype);
          newAcTopicSet.add(topic.name);
        });
        node.services.forEach((service) => {
          newAcTsSet.add(service.name);
          newAcTsSet.add(service.srvtype);
        });
      });
    });
    setTSList(Array.from(newAcTsSet));
    setTopicList(Array.from(newAcTopicSet));
  });

  useCustomEventListener(EVENT_OPEN_CONNECT, (data) => {
    handleOpen();
  });

  const getHosts = () => {
    const hosts = hostValues;
    if (hostInputValue !== "" && !hosts.includes(hostInputValue)) {
      hosts.push(hostInputValue);
    }
    return hosts.map((h) => (h.host ? h.ip : h));
  };

  const getRobotHosts = () => {
    const robotHosts = [];
    startParameter.discovery.robotHosts.forEach((h) => {
      if (h.host) robotHosts.push(h.host);
      else if (typeof h === "string") robotHosts.push(h);
    });
    // add temporal values to the list as well
    if (robotHostInputValue.length > 0) robotHosts.push(robotHostInputValue);
    return robotHosts;
  };

  const setRosVersion = useCallback(
    (rosVersion) => {
      startParameter.rosVersion = rosVersion;
      setStartParameter(JSON.parse(JSON.stringify(startParameter)));
    },
    [startParameter]
  );

  const setNetworkId = (networkId) => {
    startParameter.networkId = networkId;
    setStartParameter(JSON.parse(JSON.stringify(startParameter)));
  };

  const setRobotHostValues = (robotHosts) => {
    startParameter.discovery.robotHosts = robotHosts;
    setStartParameter(JSON.parse(JSON.stringify(startParameter)));
  };

  const setDoNotSync = (doNotSync) => {
    startParameter.sync.doNotSync = doNotSync;
    setStartParameter(JSON.parse(JSON.stringify(startParameter)));
  };

  const setSyncTopics = (syncTopics) => {
    startParameter.sync.syncTopics = syncTopics;
    setStartParameter(JSON.parse(JSON.stringify(startParameter)));
  };

  const setTtydPort = (port) => {
    startParameter.ttyd.port = port;
    setStartParameter(JSON.parse(JSON.stringify(startParameter)));
  };

  const stringifyStartConfig = (cfg) => {
    let result = `${cfg.hosts.join()}; ros${cfg.params.rosVersion}`;
    if (cfg.params.rosVersion === "1") {
      result = `${result}; network id: ${cfg.params.networkId}`;
    } else {
      result = `${result}; domain id: ${cfg.params.networkId}`;
    }
    const robotHosts = cfg.params.discovery.robotHosts.join(",");
    if (robotHosts.length > 0) {
      result = `${result}; robotHosts: [${robotHosts}]`;
    }
    if (cfg.params.sync.enable) {
      result = `${result}; +sync: doNotSync[${cfg.params.sync.doNotSync.join(",")}], syncTopics[${cfg.params.sync.syncTopics.join(",")}]`;
    }
    result = `${result}; ttyd port: ${cfg.params.ttyd.port}`;
    return result;
  };

  const createLaunchConfigFor = (host) => {
    const launchCfg = new ProviderLaunchConfiguration(host, startParameter.rosVersion);
    launchCfg.daemon.enable = enableDaemonNode;
    launchCfg.discovery.enable = enableDiscoveryNode;
    if (startParameter.networkId) launchCfg.networkId = startParameter.networkId;
    if (startParameter.discovery.robotHosts.length > 0)
      launchCfg.discovery.robotHosts = startParameter.discovery.robotHosts;
    launchCfg.sync.enable = startParameter.sync.enable;
    launchCfg.sync.doNotSync = startParameter.sync.doNotSync;
    launchCfg.sync.syncTopics = startParameter.sync.syncTopics;
    launchCfg.terminal.enable = enableTerminalManager;
    launchCfg.terminal.port = startParameter.ttyd.port;
    launchCfg.autoConnect = true;
    launchCfg.autostart = rosCtx.isLocalHost(host);
    launchCfg.forceRestart = forceRestart;
    return launchCfg;
  };

  const handleStartProvider = async () => {
    if (!rosCtx.multimasterManager) return;

    setStartProviderStatus("active");
    setStartProviderDescription("Starting nodes on selected hosts");
    setStartProviderIsSubmitting(true);
    if (saveDefaultParameter) {
      setStartConfigurationsDefault(startParameter);
    }

    const launchCfgs = [];
    const hosts = getHosts();

    // update recent start configurations
    const startCfg = {
      id: generateUniqueId(),
      hosts: hosts.sort(),
      params: startParameter,
    };
    const oldStartConfigurations = startConfigurations.filter(
      (cfg) => stringifyStartConfig(cfg) !== stringifyStartConfig(startCfg)
    );
    if (startCfg.hosts.length > 0) {
      setStartConfigurations([startCfg, ...oldStartConfigurations]);
    }

    // create launch configuration
    hosts.forEach((host) => {
      const launchCfg = createLaunchConfigFor(host);
      launchCfgs.push(launchCfg);
    });

    // start provider
    let successStart = true;
    await Promise.all(
      launchCfgs.map(async (launchCfg) => {
        if (!(await rosCtx.startConfig(launchCfg))) {
          successStart = false;
        }
      })
    );
    if (successStart) {
      setStartProviderStatus("finished");
      setStartProviderDescription("");
    } else {
      setStartProviderStatus("error");
    }

    // remove loading message
    setTimeout(() => {
      setStartProviderIsSubmitting(false);
      setStartProviderDescription("");
      setStartProviderStatus("inactive");
      handleClose();
    }, 500);
  };

  const handleJoinProvider = async () => {
    setStartProviderStatus("active");
    setStartProviderIsSubmitting(true);
    if (saveDefaultParameter) {
      setStartConfigurationsDefault(startParameter);
    }
    const hosts = hostValues;
    if (hostInputValue !== "" && !hosts.includes(hostInputValue)) {
      hosts.push(hostInputValue);
    }
    const port = startParameter.port
      ? startParameter.port
      : getDefaultPortFromRos(Provider.type, startParameter.rosVersion) + startParameter.networkId;
    // join each host separately
    await Promise.all(
      hosts.map(async (remoteHost) => {
        let host = remoteHost;
        if (remoteHost.ip) host = remoteHost.ip;
        setStartProviderDescription(`Connecting to ${host} ...`);
        console.log(`connecting to ${host}:${port}`);
        const newProvider = new Provider(settingsCtx, host, startParameter.rosVersion, port, undefined, logCtx);
        const launchCfg = createLaunchConfigFor(host);
        newProvider.startConfiguration = launchCfg;
        await rosCtx.connectToProvider(newProvider);
      })
    );
    // remove loading message and close dialog
    setTimeout(() => {
      setStartProviderIsSubmitting(false);
      setStartProviderDescription("");
      setStartProviderStatus("inactive");
      handleClose();
    }, 500);
  };

  useEffect(() => {
    // use system ros version
    if (rosCtx.rosInfo?.version) {
      setRosVersion(rosCtx.rosInfo?.version);
    }
    // do not include to dependency (loop!): setRosVersion
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.rosInfo]);

  const generateHistoryView = useMemo(() => {
    if (startConfigurations.length === 0) {
      // eslint-disable-next-line react/jsx-no-useless-fragment
      return <></>;
    }
    return (
      <Stack
        maxHeight="13em"
        // backgroundColor={settingsCtx.get('backgroundColor')}
        paddingBottom="1.2em"
      >
        <Typography paddingBottom="0.3em" variant="body2">
          Recent:
        </Typography>
        <Paper
          sx={{
            maxHeight: "13em",
            width: "100%",
            overflow: "auto",
            backgroundColor: settingsCtx.get("backgroundColor"),
          }}
        >
          <TableContainer>
            <Table aria-label="favorite table">
              <TableBody>
                {startConfigurations.map((cfg) => (
                  <TableRow
                    hover
                    key={cfg.id}
                    onClick={() => {
                      setStartParameter(cfg.params);
                      setHostValues(cfg.hosts);
                      setHostInputValue("");
                      setSelectedHistory(cfg.id);
                    }}
                  >
                    <TableCell>
                      <Typography
                        noWrap
                        variant="body2"
                        fontWeight={selectedHistory === cfg.id ? "bold" : "normal"}
                        style={{
                          cursor: "pointer",
                        }}
                      >
                        {stringifyStartConfig(cfg)}
                      </Typography>
                    </TableCell>
                    <TableCell style={{ padding: 0, width: "2em" }}>
                      <Tooltip title="Delete entry" placement="bottom">
                        <IconButton
                          color="error"
                          onClick={() => {
                            setStartConfigurations((prev) => prev.filter((pCfg) => pCfg.id !== cfg.id));
                          }}
                          size="small"
                        >
                          <DeleteOutlineOutlinedIcon fontSize="inherit" />
                        </IconButton>
                      </Tooltip>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </Paper>
      </Stack>
    );
  }, [startConfigurations, settingsCtx, selectedHistory, setStartConfigurations]);

  return (
    <>
      <Dialog
        keepMounted
        id="connect-to-ros-modal"
        scroll="paper"
        PaperComponent={DraggablePaper}
        aria-labelledby="draggable-dialog-title"
        fullWidth
        maxWidth="md"
        open={open}
        onClose={handleClose}
        // disableEscapeKeyDown
      >
        <DialogTitle style={{ cursor: "move" }} id="draggable-dialog-title">
          Connect To ROS
        </DialogTitle>
        <DialogContent>
          <Stack direction="row" spacing="0.5em">
            <Stack flexGrow={1}>
              <Box>
                {startConfigurations.length === 0 && (
                  <Typography paddingBottom="1em" color="green">
                    No saved start configurations found. Select hosts that you want to join or on which you want to
                    start the required MAS system nodes. Use <RocketLaunchIcon fontSize="inherit" /> to open this
                    dialog.
                  </Typography>
                )}
                {generateHistoryView}
                <Autocomplete
                  disablePortal
                  multiple
                  id="auto-complete-hosts"
                  size="medium"
                  options={hostList}
                  freeSolo
                  ListboxProps={{ style: { maxHeight: 150 } }}
                  sx={{ margin: 0 }}
                  getOptionLabel={(option) => (option.host ? `${option.host} [${option.ip}]` : option)}
                  renderInput={(params) => (
                    <TextField {...params} variant="standard" label="Add Hosts" placeholder=" add " />
                  )}
                  value={hostValues}
                  onChange={(event, newValue) => {
                    setHostValues(newValue);
                  }}
                  inputValue={hostInputValue}
                  onInputChange={(event, newInputValue) => {
                    setHostInputValue(newInputValue);
                  }}
                  disableCloseOnSelect
                  renderOption={(props, option, { selected }) => (
                    <li {...props} style={{ height: "1.5em" }}>
                      <Checkbox
                        icon={icon}
                        checkedIcon={checkedIcon}
                        style={{ marginRight: 8 }}
                        checked={selected}
                        size="small"
                      />
                      {`${option.host} [${option.ip}]`}
                    </li>
                  )}
                />
              </Box>

              <Stack direction="row">
                {/* <FormControl> */}
                <RadioGroup
                  row
                  aria-labelledby="ros-version-select-group-label"
                  name="ros-version-select-group"
                  value={startParameter.rosVersion}
                  onChange={(event) => {
                    setRosVersion(event.target.value);
                  }}
                >
                  <FormControlLabel value="1" control={<Radio />} label="ROS1" />
                  <FormControlLabel value="2" control={<Radio />} label="ROS2" />
                </RadioGroup>
                {/* </FormControl> */}
                <TextField
                  type="number"
                  id="network-id"
                  min={0}
                  max={99}
                  label="Network/Domain ID"
                  size="small"
                  variant="outlined"
                  style={{ minWidth: "9em" }}
                  InputProps={{ inputProps: { min: 0, max: 99 } }}
                  // fullWidth
                  onChange={(e) => setNetworkId(Number(`${e.target.value}`))}
                  value={startParameter.networkId}
                />
              </Stack>
              <Stack spacing={2} direction="row">
                {startProviderIsSubmitting ? (
                  <Stack direction="row" spacing={1} marginLeft="1rem">
                    <CircularProgress size="1em" />
                    <div>{startProviderDescription}</div>
                  </Stack>
                ) : (
                  <Stack direction="row" spacing="1em" marginLeft="1rem">
                    <Button
                      type="submit"
                      variant="contained"
                      // display="flex"
                      color="success"
                      onClick={() => {
                        handleStartProvider();
                      }}
                      disabled={
                        !window.CommandExecutor || (hostValues.length === 0 && hostInputValue === "")
                        // hostValues.length === 0 && hostInputValue === ''
                      }
                      style={{ height: "3em", textAlign: "center" }}
                      endIcon={<RocketLaunchIcon />}
                    >
                      Start
                    </Button>
                    <Button
                      type="submit"
                      variant="contained"
                      // display="flex"
                      color="success"
                      onClick={() => {
                        handleJoinProvider();
                      }}
                      disabled={
                        hostValues.length === 0 && hostInputValue === ""
                        // hostValues.length === 0 && hostInputValue === ''
                      }
                      style={{ height: "3em", textAlign: "center" }}
                      endIcon={<JoinFullIcon />}
                    >
                      Join
                    </Button>
                  </Stack>
                )}
              </Stack>
              <Accordion
                disabled={!window.CommandExecutor}
                disableGutters
                elevation={0}
                sx={{
                  "&:before": {
                    display: "none",
                  },
                }}
              >
                <AccordionSummary
                  // disabled={!startSystemNodes}
                  expandIcon={<ExpandMoreIcon />}
                  aria-controls="advanced-options"
                  id="advanced-options"
                  sx={{ pl: 0 }}
                >
                  <Stack direction="row" alignItems="center" spacing="0.3em">
                    <SettingsOutlinedIcon fontSize="inherit" />
                    <Typography variant="subtitle1">Advanced parameters:</Typography>
                    {/* <Tooltip
                      title="Reset advanced parameters to default"
                      placement="right"
                    >
                      <IconButton
                        color="default"
                        onClick={(event) => {
                          setStartParameter(DEFAULT_PARAMETER);
                          setForceRestart(true);
                          setSaveDefaultParameter(false);
                          setSelectedHistory('');
                          event.stopPropagation();
                        }}
                        size="small"
                      >
                        <RestartAltIcon fontSize="inherit" />
                      </IconButton>
                    </Tooltip> */}
                  </Stack>
                </AccordionSummary>
                <AccordionDetails>
                  <Stack
                    direction="column"
                    // divider={<Divider orientation="horizontal" />}
                  >
                    <FormGroup aria-label="position" row>
                      <FormControlLabel
                        control={
                          <Checkbox
                            checked={enableDaemonNode}
                            onChange={(event) => {
                              setEnableDaemonNode(event.target.checked);
                            }}
                          />
                        }
                        label="Daemon Node"
                        labelPlacement="end"
                      />
                    </FormGroup>
                    {startParameter.rosVersion === "2" && (
                      <FormGroup aria-label="position" row>
                        <FormControlLabel
                          control={
                            <Checkbox
                              checked={enableDiscoveryNode}
                              onChange={(event) => {
                                setEnableDiscoveryNode(event.target.checked);
                              }}
                            />
                          }
                          label="Discovery Node"
                          labelPlacement="end"
                        />
                      </FormGroup>
                    )}

                    {startParameter.rosVersion === "1" && (
                      <Accordion
                        disableGutters
                        elevation={0}
                        sx={{
                          "&:before": {
                            display: "none",
                          },
                        }}
                      >
                        <AccordionSummary
                          expandIcon={<ExpandMoreIcon />}
                          aria-controls="discovery_panel-content"
                          id="discovery_panel-header"
                          sx={{ pl: 0 }}
                        >
                          <Grid container>
                            <Grid item xs={6} sm={5} md={4} lg={4} xl={3}>
                              <FormGroup
                                aria-label="position"
                                row
                                onClick={(event) => {
                                  event.stopPropagation();
                                }}
                              >
                                <FormControlLabel
                                  control={
                                    <Checkbox
                                      checked={enableDiscoveryNode}
                                      onChange={(event) => {
                                        setEnableDiscoveryNode(event.target.checked);
                                      }}
                                    />
                                  }
                                  label="Discovery Node"
                                  labelPlacement="end"
                                />
                              </FormGroup>
                            </Grid>
                            <Grid item sx={{ alignSelf: "center" }}>
                              <Stack direction="column" sx={{ display: "grid" }}>
                                <Typography
                                  noWrap
                                  variant="body2"
                                  sx={{
                                    color: grey[700],
                                    fontWeight: "inherit",
                                    flexGrow: 1,
                                    ml: 0.5,
                                  }}
                                >
                                  {`Robot Hosts: [${getRobotHosts().join(",")}]`}
                                </Typography>
                              </Stack>
                            </Grid>
                          </Grid>
                        </AccordionSummary>
                        <AccordionDetails>
                          <Stack direction="column" divider={<Divider orientation="vertical" />}>
                            <Box>
                              <Autocomplete
                                disabled={!enableDiscoveryNode}
                                disablePortal
                                multiple
                                id="auto-complete-robot-hosts"
                                size="small"
                                options={hostList}
                                freeSolo
                                sx={{ margin: 0 }}
                                ListboxProps={{ style: { maxHeight: 150 } }}
                                getOptionLabel={(option) => (option.host ? `${option.host} [${option.ip}]` : option)}
                                renderInput={(params) => (
                                  <TextField
                                    {...params}
                                    variant="outlined"
                                    label="Add Robot Hosts"
                                    placeholder="..."
                                    fullWidth
                                  />
                                )}
                                value={startParameter.discovery.robotHosts}
                                onChange={(event, newValue) => {
                                  setRobotHostValues(newValue);
                                }}
                                inputValue={robotHostInputValue}
                                onInputChange={(event, newInputValue) => {
                                  setRobotHostInputValue(newInputValue);
                                }}
                                disableCloseOnSelect
                                renderOption={(props, option, { selected }) => (
                                  <li {...props} style={{ height: "1.5em" }}>
                                    <Checkbox
                                      icon={icon}
                                      checkedIcon={checkedIcon}
                                      // style={{ marginRight: 8 }}
                                      checked={selected}
                                      size="small"
                                    />
                                    {`${option.host} [${option.ip}]`}
                                  </li>
                                )}
                              />
                            </Box>
                          </Stack>
                        </AccordionDetails>
                      </Accordion>
                    )}
                    {startParameter.rosVersion === "1" && (
                      <Accordion
                        disableGutters
                        elevation={0}
                        sx={{
                          "&:before": {
                            display: "none",
                          },
                        }}
                      >
                        <AccordionSummary
                          // disabled={!startSystemNodes}
                          expandIcon={<ExpandMoreIcon />}
                          aria-controls="sync_panel-content"
                          id="sync_panel-header"
                          sx={{ pl: 0 }}
                        >
                          <Grid container>
                            <Grid item xs={6} sm={5} md={4} lg={4} xl={3}>
                              <FormGroup
                                aria-label="position"
                                row
                                onClick={(event) => {
                                  event.stopPropagation();
                                }}
                              >
                                <FormControlLabel
                                  // disabled={!startSystemNodes}
                                  control={
                                    <Checkbox
                                      checked={enableSyncNode}
                                      onChange={(event) => {
                                        setEnableSyncNode(event.target.checked);
                                      }}
                                    />
                                  }
                                  label="Master Sync"
                                  labelPlacement="end"
                                />
                              </FormGroup>
                            </Grid>
                            <Grid item sx={{ alignSelf: "center" }}>
                              <Stack direction="column" sx={{ display: "grid" }}>
                                <Typography
                                  noWrap
                                  variant="body2"
                                  sx={{
                                    color: grey[700],
                                    fontWeight: "inherit",
                                    flexGrow: 1,
                                    ml: 0.5,
                                  }}
                                >
                                  {`DoNotSync: [${startParameter.sync.doNotSync.join()}]`}
                                </Typography>
                                <Typography
                                  noWrap
                                  variant="body2"
                                  sx={{
                                    color: grey[700],
                                    fontWeight: "inherit",
                                    flexGrow: 1,
                                    ml: 0.5,
                                  }}
                                >
                                  {`SyncTopics: [${startParameter.sync.syncTopics.join()}]`}
                                </Typography>
                              </Stack>
                            </Grid>
                          </Grid>
                        </AccordionSummary>
                        <AccordionDetails>
                          <Stack direction="column" divider={<Divider orientation="vertical" />}>
                            <Autocomplete
                              disablePortal
                              multiple
                              id="auto-complete-donotsync"
                              size="small"
                              options={tsList}
                              freeSolo
                              sx={{ margin: 0 }}
                              renderInput={(params) => <TextField {...params} variant="outlined" label="do not sync" />}
                              value={startParameter.sync.doNotSync}
                              onChange={(event, newValue) => {
                                setDoNotSync(newValue);
                              }}
                              // disableCloseOnSelect
                            />
                            <Autocomplete
                              disablePortal
                              multiple
                              id="auto-complete-synctopics"
                              size="small"
                              options={topicList}
                              freeSolo
                              sx={{ margin: 0 }}
                              renderInput={(params) => <TextField {...params} variant="outlined" label="sync topics" />}
                              value={startParameter.sync.syncTopics}
                              onChange={(event, newValue) => {
                                setSyncTopics(newValue);
                              }}
                              // disableCloseOnSelect
                            />
                          </Stack>
                        </AccordionDetails>
                      </Accordion>
                    )}
                    <Accordion
                      disableGutters
                      elevation={0}
                      sx={{
                        "&:before": {
                          display: "none",
                        },
                      }}
                    >
                      <AccordionSummary
                        expandIcon={<ExpandMoreIcon />}
                        aria-controls="ttyd_panel-content"
                        id="ttyd_panel-header"
                        sx={{ pl: 0, margin: 0 }}
                        style={{
                          content: {
                            display: "flex",
                            flexGrow: 1,
                            margin: 0,
                          },
                          margin: 0,
                        }}
                      >
                        <Grid container>
                          <Grid item xs={6} sm={5} md={4} lg={4} xl={3}>
                            <FormGroup
                              aria-label="position"
                              row
                              onClick={(event) => {
                                event.stopPropagation();
                              }}
                            >
                              <FormControlLabel
                                control={
                                  <Checkbox
                                    checked={enableTerminalManager}
                                    onChange={(event) => {
                                      setEnableTerminalManager(event.target.checked);
                                    }}
                                  />
                                }
                                label={
                                  <div>
                                    Terminal Manager (TTYD)
                                    <Tooltip
                                      title={
                                        <>
                                          <Typography color="h2">Install TTYD in the host using:</Typography>
                                          <Stack mt={1} direction="row" justifyContent="center">
                                            <Typography color="body2">sudo snap install ttyd --classic</Typography>
                                            <CopyButton value="sudo snap install ttyd --classic" />
                                          </Stack>
                                          <Link
                                            mt={2}
                                            href="https://github.com/tsl0922/ttyd"
                                            target="_blank"
                                            color="inherit"
                                          >
                                            See https://github.com/tsl0922/ttyd
                                          </Link>
                                        </>
                                      }
                                      // PopperProps={{
                                      //   disablePortal: true,
                                      // }}
                                      disableFocusListener
                                      disableHoverListener
                                      disableTouchListener
                                      open={openTerminalTooltip}
                                      placement="bottom-start"
                                      // enterDelay={tooltipDelay}
                                      // enterNextDelay={tooltipDelay}
                                    >
                                      <IconButton
                                        edge="start"
                                        aria-label="additional terminal information"
                                        onClick={() => {
                                          setOpenTerminalTooltip(!openTerminalTooltip);
                                        }}
                                      >
                                        <InfoOutlinedIcon
                                          sx={{
                                            fontSize: "inherit",
                                            color: "DodgerBlue",
                                          }}
                                        />
                                      </IconButton>
                                    </Tooltip>
                                  </div>
                                }
                                labelPlacement="end"
                              />
                            </FormGroup>
                          </Grid>
                          <Grid item sx={{ alignSelf: "center" }}>
                            <Stack direction="column" sx={{ display: "grid" }}>
                              <Typography
                                variant="body2"
                                sx={{
                                  color: grey[700],
                                  fontWeight: "inherit",
                                  flexGrow: 1,
                                  ml: 0.5,
                                }}
                              >
                                {`Port: ${startParameter.ttyd.port}`}
                              </Typography>
                            </Stack>
                          </Grid>
                        </Grid>
                      </AccordionSummary>
                      <AccordionDetails>
                        <TextField
                          type="number"
                          id="ttyd-port"
                          label="Port"
                          size="small"
                          variant="outlined"
                          fullWidth
                          onChange={(e) => setTtydPort(Number(`${e.target.value}`))}
                          value={startParameter.ttyd.port}
                          disabled={!enableTerminalManager}
                        />
                      </AccordionDetails>
                    </Accordion>
                    <FormGroup aria-label="position" row>
                      <FormControlLabel
                        disabled={!(enableDaemonNode && enableDiscoveryNode)}
                        control={
                          <Checkbox
                            size="small"
                            checked={forceRestart}
                            onChange={(event) => {
                              setForceRestart(event.target.checked);
                            }}
                          />
                        }
                        label="force restart MAS nodes"
                        labelPlacement="end"
                      />
                    </FormGroup>
                    <FormGroup aria-label="position" row>
                      <FormControlLabel
                        disabled={!(enableDaemonNode && enableDiscoveryNode && enableTerminalManager)}
                        control={
                          <Checkbox
                            size="small"
                            checked={
                              enableDaemonNode && enableDiscoveryNode && enableTerminalManager && saveDefaultParameter
                            }
                            onChange={(event) => {
                              setSaveDefaultParameter(event.target.checked);
                            }}
                          />
                        }
                        label="save parameters as default"
                        labelPlacement="end"
                      />
                    </FormGroup>
                    <Box>
                      <Button
                        size="small"
                        type="submit"
                        variant="outlined"
                        // display="flex"
                        color="info"
                        onClick={(event) => {
                          setStartParameter(DEFAULT_PARAMETER);
                          setForceRestart(true);
                          setSaveDefaultParameter(false);
                          setSelectedHistory("");
                          event.stopPropagation();
                        }}
                        style={{
                          height: "1.5em",
                          textTransform: "none",
                        }}
                        endIcon={<RestartAltIcon />}
                      >
                        Reset advanced parameters to default
                      </Button>
                    </Box>
                  </Stack>
                </AccordionDetails>
              </Accordion>
            </Stack>
          </Stack>
        </DialogContent>
        <DialogActions>
          <Button
            color="primary"
            onClick={handleClose}
            variant="text"
            // style={{ height: 40, textAlign: 'center' }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
      <Tooltip title="Start system nodes" placement="bottom" disableInteractive>
        <IconButton
          color="primary"
          onClick={() => {
            handleOpen();
          }}
          size="small"
        >
          <RocketLaunchIcon fontSize="inherit" />
        </IconButton>
      </Tooltip>
    </>
  );
}

ConnectToProviderModal.propTypes = {};

export default ConnectToProviderModal;
