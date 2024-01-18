import { useContext, useEffect, useState } from 'react';

import { grey } from '@mui/material/colors';

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
  FormControl,
  FormControlLabel,
  FormGroup,
  Grid,
  IconButton,
  Link,
  Radio,
  RadioGroup,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from '@mui/material';

import CheckBoxIcon from '@mui/icons-material/CheckBox';
import CheckBoxOutlineBlankIcon from '@mui/icons-material/CheckBoxOutlineBlank';
import MuiAccordion from '@mui/material/Accordion';
import MuiAccordionDetails from '@mui/material/AccordionDetails';
import { styled } from '@mui/material/styles';
// import ArrowForwardIosSharpIcon from '@mui/icons-material/ArrowForwardIosSharp';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';
// import SettingsInputComponentIcon from '@mui/icons-material/SettingsInputComponent';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import InfoOutlinedIcon from '@mui/icons-material/InfoOutlined';

import { useCustomEventListener } from 'react-custom-events';
import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import { SettingsContext } from '../../context/SettingsContext';
import useLocalStorage from '../../hooks/useLocalStorage';
import ProviderLaunchConfiguration from '../../models/ProviderLaunchConfiguration';
import CrossbarIOProvider from '../../providers/crossbar_io/CrossbarIOProvider';
import { EVENT_PROVIDER_ROS_NODES } from '../../providers/events';
import CopyButton from '../UI/CopyButton';
import DraggablePaper from '../UI/DraggablePaper';

const Accordion = styled((props) => (
  <MuiAccordion disableGutters elevation={0} square {...props} />
))(({ theme }) => ({
  border: 'none',
  backgroundColor:
    theme.palette.mode === 'dark'
      ? 'rgba(0, 0, 0, .00)'
      : 'rgba(255, 255, 255, .00)',
}));

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  paddingTop: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;

function ConnectToProviderModal() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);

  const [open, setOpen] = useState(false);
  const [openTerminalTooltip, setOpenTerminalTooltip] = useState(false);
  const [startSystemNodes, setStartSystemNodes] = useState(false);

  const [hostList, setHostList] = useState([]);
  const [hostValues, setHostValues] = useState([]);
  const [hostInputValue, setHostInputValue] = useState('');
  const [robotHostValues, setRobotHostValues] = useState([]);
  const [robotHostInputValue, setRobotHostInputValue] = useState('');

  const [saveInProviderList, setSaveInProviderList] = useLocalStorage(
    'ConnectToProviderModal:saveInProviderList',
    true,
  );

  const [enableDaemonNode, setEnableDaemonNode] = useLocalStorage(
    'ConnectToProviderModal:enableDaemonNode',
    true,
  );
  const [enableDiscoveryNode, setEnableDiscoveryNode] = useLocalStorage(
    'ConnectToProviderModal:enableDiscoveryNode',
    true,
  );
  const [enableSyncNode, setEnableSyncNode] = useLocalStorage(
    'ConnectToProviderModal:enableSyncNode',
    true,
  );
  const [enableTerminalManager, setEnableTerminalManager] = useLocalStorage(
    'ConnectToProviderModal:enableTerminalManager',
    true,
  );

  const [rosVersion, setRosVersion] = useState(settingsCtx.get('rosVersion'));

  const [networkID, setNetworkID] = useLocalStorage(
    'ConnectToProviderModal:networkID',
    0,
  );
  const [tsList, setTSList] = useState([]);
  const [topicList, setTopicList] = useState([]);
  const [doNotSync, setDoNotSync] = useState([]);
  const [syncTopics, setSyncTopics] = useState([]);
  const [TTYDPort, setTTYDPort] = useLocalStorage(
    'ConnectToProviderModal:TTYDPort',
    7681,
  );

  const [startProviderStatus, setStartProviderStatus] = useState('');
  const [startProviderDescription, setStartProviderDescription] = useState('');
  const [startProviderIsSubmitting, setStartProviderIsSubmitting] =
    useState(false);

  const handleOpen = () => setOpen(true);
  const handleClose = (event, reason) => {
    if (reason && reason === 'backdropClick') return;
    setOpen(false);
    setOpenTerminalTooltip(false);
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
        const lastName = h[1].split(' ').pop();
        hostListLocal.push({ ip, host: lastName });
      }
    });
    hostListLocal.sort((a, b) => -b.host.localeCompare(a.host));
    setHostList(hostListLocal);
    setHostValues(['localhost']);
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

  const robotHostValueStr = () => {
    const robotHosts = [];
    robotHostValues.forEach((h) => {
      if (h.host) robotHosts.push(h.host);
      else if (typeof h === 'string') robotHosts.push(h);
    });
    return robotHosts.join();
  };

  const handleStartProvider = async () => {
    if (!rosCtx.multimasterManager) return;

    setStartProviderStatus('active');
    setStartProviderDescription('Starting nodes on selected hosts');
    setStartProviderIsSubmitting(true);

    const robotHosts = [];
    robotHostValues.forEach((h) => {
      if (h.host) robotHosts.push(h.host);
      else if (typeof h === 'string') robotHosts.push(h);
    });
    // add temporal values to the list as well
    if (robotHostInputValue.length > 0) robotHosts.push(robotHostInputValue);

    const launchCfgs = [];
    const hosts = hostValues;
    if (hostInputValue !== '' && !hosts.includes(hostInputValue)) {
      hosts.push(hostInputValue);
    }
    hosts.forEach((h) => {
      let host = h;
      if (h.host) host = h.host;
      const launchCfg = new ProviderLaunchConfiguration(host, rosVersion);
      launchCfg.daemon.enable = enableDaemonNode;
      launchCfg.discovery.enable = enableDiscoveryNode;
      if (networkID) launchCfg.discovery.networkId = networkID;
      if (robotHosts.length > 0) launchCfg.discovery.robotHosts = robotHosts;
      launchCfg.sync.enable = enableSyncNode && rosVersion === '1';
      launchCfg.sync.doNotSync = doNotSync;
      launchCfg.sync.syncTopics = syncTopics;
      launchCfg.terminal.enable = true;
      launchCfg.autoConnect = true;
      launchCfg.autostart = rosCtx.isLocalHost(host);
      if (
        !(
          startSystemNodes &&
          !(enableDaemonNode && enableDiscoveryNode && enableTerminalManager)
        ) &&
        saveInProviderList
      ) {
        rosCtx.saveProviderConfig(launchCfg);
      } else {
        launchCfg.forceRestart = true;
      }
      launchCfgs.push(launchCfg);
    });
    let successStart = true;
    launchCfgs.forEach(async (launchCfg) => {
      if (!(await rosCtx.startConfig(launchCfg))) {
        successStart = false;
      }
    });
    if (successStart) {
      setStartProviderStatus('finished');
      setStartProviderDescription('');
    } else {
      setStartProviderStatus('error');
    }

    // remove loading message
    setTimeout(() => {
      setStartProviderIsSubmitting(false);
      setStartProviderDescription('');
      setStartProviderStatus('inactive');
      handleClose();
    }, 500);
  };

  const handleJoinProvider = async () => {
    setStartProviderStatus('active');
    setStartProviderIsSubmitting(true);
    const hosts = hostValues;
    if (hostInputValue !== '' && !hosts.includes(hostInputValue)) {
      hosts.push(hostInputValue);
    }
    // start each host separately
    hosts.forEach(async (crossbarHost) => {
      if (saveInProviderList) {
        rosCtx.saveProviderConfig(
          new ProviderLaunchConfiguration(crossbarHost, rosVersion),
        );
      }
      setStartProviderDescription(`Connecting to ${crossbarHost} ...`);
      const newProvider = new CrossbarIOProvider(
        settingsCtx,
        crossbarHost,
        rosVersion,
        undefined,
        undefined,
        logCtx,
      );
      await rosCtx.connectToProvider(newProvider);
    });
    // remove loading message and close dialog
    setTimeout(() => {
      setStartProviderIsSubmitting(false);
      setStartProviderDescription('');
      setStartProviderStatus('inactive');
      handleClose();
    }, 500);
  };

  return (
    <>
      <Dialog
        keepMounted
        id="connect-to-ros-modal"
        scroll="paper"
        PaperComponent={DraggablePaper}
        aria-labelledby="draggable-dialog-title"
        fullWidth
        maxWidth="lg"
        open={open}
        onClose={handleClose}
        // disableEscapeKeyDown
      >
        <DialogTitle style={{ cursor: 'move' }} id="draggable-dialog-title">
          Connect To ROS
        </DialogTitle>
        <DialogContent>
          <Box>
            <Autocomplete
              disablePortal
              multiple
              id="auto-complete-hosts"
              size="medium"
              options={hostList}
              freeSolo
              sx={{ margin: 0 }}
              getOptionLabel={(option) =>
                option.host ? `${option.host} [${option.ip}]` : option
              }
              renderInput={(params) => (
                <TextField
                  {...params}
                  variant="outlined"
                  label="Add Hosts"
                  placeholder="..."
                />
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
                <li
                  {...props}
                  style={{
                    height: 30,
                    fontSize: settingsCtx.fontSize + 2,
                  }}
                >
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

          <FormControl>
            {/* <FormLabel id="ros-version-select-label">ROS version</FormLabel> */}
            <RadioGroup
              row
              aria-labelledby="ros-version-select-group-label"
              name="ros-version-select-group"
              value={rosVersion}
              onChange={(event) => {
                console.log(
                  `ROS_VERSION: ${JSON.stringify(event.target.value)}`,
                );
                setRosVersion(event.target.value);
              }}
            >
              <FormControlLabel value="1" control={<Radio />} label="ROS1" />
              <FormControlLabel value="2" control={<Radio />} label="ROS2" />
            </RadioGroup>
          </FormControl>
          <FormGroup aria-label="position" row>
            <FormControlLabel
              disabled={!window.CommandExecutor}
              control={
                <Checkbox
                  checked={startSystemNodes}
                  onChange={(event) => {
                    setStartSystemNodes(event.target.checked);
                  }}
                />
              }
              label="Start system nodes"
              labelPlacement="end"
            />
          </FormGroup>
          <FormGroup aria-label="position" row>
            <FormControlLabel
              disabled={!startSystemNodes}
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

          <Accordion
            disableGutters
            elevation={0}
            sx={{
              '&:before': {
                display: 'none',
              },
            }}
          >
            <AccordionSummary
              disabled={!startSystemNodes}
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
                      disabled={!startSystemNodes}
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
                {rosVersion === '1' && (
                  <Grid item sx={{ alignSelf: 'center' }}>
                    <Stack direction="column" sx={{ display: 'grid' }}>
                      <Typography
                        noWrap
                        variant="body2"
                        sx={{
                          color: grey[700],
                          fontWeight: 'inherit',
                          flexGrow: 1,
                          ml: 0.5,
                        }}
                      >
                        {`Network ID: ${networkID}`}
                      </Typography>
                      <Typography
                        noWrap
                        variant="body2"
                        sx={{
                          color: grey[700],
                          fontWeight: 'inherit',
                          flexGrow: 1,
                          ml: 0.5,
                        }}
                      >
                        {`Robot Hosts: [${robotHostValueStr()}]`}
                      </Typography>
                    </Stack>
                  </Grid>
                )}
              </Grid>
            </AccordionSummary>
            {rosVersion === '1' && (
              <AccordionDetails>
                <Stack
                  direction="column"
                  divider={<Divider orientation="vertical" />}
                >
                  <TextField
                    type="number"
                    id="discovery-port"
                    min={0}
                    max={99}
                    label="Network ID"
                    size="small"
                    variant="outlined"
                    InputProps={{ inputProps: { min: 0, max: 99 } }}
                    fullWidth
                    onChange={(e) => setNetworkID(Number(`${e.target.value}`))}
                    value={networkID}
                    disabled={!enableDiscoveryNode}
                  />

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
                      getOptionLabel={(option) =>
                        option.host ? `${option.host} [${option.ip}]` : option
                      }
                      renderInput={(params) => (
                        <TextField
                          {...params}
                          variant="outlined"
                          label="Add Robot Hosts"
                          placeholder="..."
                          fullWidth
                        />
                      )}
                      value={robotHostValues}
                      onChange={(event, newValue) => {
                        setRobotHostValues(newValue);
                      }}
                      inputValue={robotHostInputValue}
                      onInputChange={(event, newInputValue) => {
                        setRobotHostInputValue(newInputValue);
                      }}
                      disableCloseOnSelect
                      renderOption={(props, option, { selected }) => (
                        <li
                          {...props}
                          style={{
                            height: 30,
                            fontSize: settingsCtx.fontSize + 2,
                          }}
                        >
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
            )}
          </Accordion>
          {rosVersion === '1' && (
            <Accordion
              disableGutters
              elevation={0}
              sx={{
                '&:before': {
                  display: 'none',
                },
              }}
            >
              <AccordionSummary
                disabled={!startSystemNodes}
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
                        disabled={!startSystemNodes}
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
                  <Grid item sx={{ alignSelf: 'center' }}>
                    <Stack direction="column" sx={{ display: 'grid' }}>
                      <Typography
                        noWrap
                        variant="body2"
                        sx={{
                          color: grey[700],
                          fontWeight: 'inherit',
                          flexGrow: 1,
                          ml: 0.5,
                        }}
                      >
                        {`DoNotSync: [${doNotSync.join()}]`}
                      </Typography>
                      <Typography
                        noWrap
                        variant="body2"
                        sx={{
                          color: grey[700],
                          fontWeight: 'inherit',
                          flexGrow: 1,
                          ml: 0.5,
                        }}
                      >
                        {`SyncTopics: [${syncTopics.join()}]`}
                      </Typography>
                    </Stack>
                  </Grid>
                </Grid>
              </AccordionSummary>
              <AccordionDetails>
                <Stack
                  direction="column"
                  divider={<Divider orientation="vertical" />}
                >
                  <Autocomplete
                    disablePortal
                    multiple
                    id="auto-complete-donotsync"
                    size="small"
                    options={tsList}
                    freeSolo
                    sx={{ margin: 0 }}
                    renderInput={(params) => (
                      <TextField
                        {...params}
                        variant="outlined"
                        label="do not sync"
                      />
                    )}
                    value={doNotSync}
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
                    renderInput={(params) => (
                      <TextField
                        {...params}
                        variant="outlined"
                        label="sync topics"
                      />
                    )}
                    value={syncTopics}
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
              '&:before': {
                display: 'none',
              },
            }}
          >
            <AccordionSummary
              disabled={!startSystemNodes}
              expandIcon={<ExpandMoreIcon />}
              aria-controls="ttyd_panel-content"
              id="ttyd_panel-header"
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
                      disabled={!startSystemNodes}
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
                                <Typography color="h2">
                                  Install TTYD in the host using:
                                </Typography>
                                <Stack
                                  mt={1}
                                  direction="row"
                                  justifyContent="center"
                                >
                                  <Typography color="body2">
                                    sudo snap install ttyd --classic
                                  </Typography>
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
                                  fontSize: 'inherit',
                                  color: 'DodgerBlue',
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
                <Grid item sx={{ alignSelf: 'center' }}>
                  <Stack direction="column" sx={{ display: 'grid' }}>
                    <Typography
                      variant="body2"
                      sx={{
                        color: grey[700],
                        fontWeight: 'inherit',
                        flexGrow: 1,
                        ml: 0.5,
                      }}
                    >
                      {`Port: ${TTYDPort}`}
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
                onChange={(e) => setTTYDPort(Number(`${e.target.value}`))}
                value={TTYDPort}
                disabled={!enableTerminalManager}
              />
            </AccordionDetails>
          </Accordion>
          <FormGroup aria-label="position" row>
            <FormControlLabel
              disabled={
                startSystemNodes &&
                !(
                  enableDaemonNode &&
                  enableDiscoveryNode &&
                  enableTerminalManager
                )
              }
              control={
                <Checkbox
                  checked={
                    !(
                      startSystemNodes &&
                      !(
                        enableDaemonNode &&
                        enableDiscoveryNode &&
                        enableTerminalManager
                      )
                    ) && saveInProviderList
                  }
                  onChange={(event) => {
                    setSaveInProviderList(event.target.checked);
                  }}
                />
              }
              label="Save in provider list"
              labelPlacement="end"
            />
          </FormGroup>

          <Stack spacing={2} direction="row">
            {startProviderIsSubmitting ? (
              <Stack direction="row" spacing={1} marginLeft="1rem">
                <CircularProgress size="1em" />
                <div>{startProviderDescription}</div>
              </Stack>
            ) : (
              <Button
                type="submit"
                variant="contained"
                display="flex"
                color="success"
                onClick={
                  startSystemNodes ? handleStartProvider : handleJoinProvider
                }
                disabled={hostValues.length === 0 && hostInputValue === ''}
                // disabled={
                //   !enableDaemonNode &&
                //   !enableDiscoveryNode &&
                //   !(enableSyncNode && rosVersion === '1') &&
                //   !enableTerminalManager
                // }
                style={{ height: 40, textAlign: 'center' }}
                endIcon={<RocketLaunchIcon />}
              >
                {startSystemNodes ? 'Start Provider' : 'Join Provider'}
              </Button>
            )}
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
      {/* <Stack padding={0.5} direction="column">
        <Tooltip title="Connect To ROS" placement="bottom">
          <IconButton color="primary" onClick={handleOpen}>
            <SettingsInputComponentIcon sx={{ fontSize: 28 }} />
          </IconButton>
        </Tooltip>
      </Stack> */}
      <Tooltip title="Start Daemon" placement="bottom">
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
