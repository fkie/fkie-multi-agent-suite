import CheckBoxIcon from "@mui/icons-material/CheckBox";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import ChevronRightIcon from "@mui/icons-material/ChevronRight";
import DeleteIcon from "@mui/icons-material/Delete";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import SaveIcon from "@mui/icons-material/Save";
import {
  Autocomplete,
  Button,
  Checkbox,
  Divider,
  FormControlLabel,
  FormGroup,
  GridLegacy,
  Link,
  Radio,
  RadioGroup,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary from "@mui/material/AccordionSummary";
import { grey } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import { HTMLAttributes, useCallback, useEffect, useMemo, useReducer, useRef, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { CopyButton } from "@/renderer/components/UI";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import { EVENT_CLOSE_COMPONENT, eventCloseComponent } from "../layout/events";

const AccordionAdv = styled((props: AccordionProps) => <MuiAccordion disableGutters elevation={0} square {...props} />)(
  ({ theme }) => ({
    border: "none",
    backgroundColor: theme.palette.mode === "dark" ? "rgba(0, 0, 0, .00)" : "rgba(255, 255, 255, .00)",
    ".MuiAccordionSummary-content": { margin: 0 },
    "&:before": {
      display: "none",
    },
    paddingTop: "9px",
  })
);

const Accordion = styled((props: AccordionProps) => <MuiAccordion disableGutters elevation={0} square {...props} />)(
  ({ theme }) => ({
    border: "none",
    backgroundColor: theme.palette.mode === "dark" ? "rgba(0, 0, 0, .00)" : "rgba(255, 255, 255, .00)",
    ".MuiAccordionSummary-content": { margin: 0 },
    "&:before": {
      display: "none",
    },
    "&:hover": { backgroundColor: theme.palette.action.hover },
  })
);

const AccordionSummary = styled(MuiAccordionSummary)(() => ({
  paddingTop: 0,
  margin: 0,
  minHeight: 26,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  paddingTop: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

type THostIp = { host: string; ip?: string };
const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;

interface ProviderLaunchConfigPanelProps {
  config: ProviderLaunchConfiguration;
  onDelete: (configId: string) => void;
  onSave: (config: ProviderLaunchConfiguration) => void;
}

export default function ProviderLaunchConfigPanel(props: ProviderLaunchConfigPanelProps): JSX.Element {
  const { config, onDelete, onSave } = props;

  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const launchCfgRef = useRef<ProviderLaunchConfiguration>(config);
  const launchCfg = launchCfgRef.current;
  const [_valuesChanged, forceValuesUpdate] = useReducer((x) => x + 1, 0);

  const hostArg: string | undefined = settingsCtx.getArgument("host") as string | undefined;
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  const [openTerminalTooltip, setOpenTerminalTooltip] = useState(false);
  const [startCmdDaemon, setStartCmdDaemon] = useState<string>("");
  const [startCmdDiscovery, setStartCmdDiscovery] = useState<string>("");
  const [startCmdTtyd, setStartCmdTtyd] = useState<string>("");
  const [tsList, setTSList] = useState<string[]>([]);
  const [topicList, setTopicList] = useState<string[]>([]);
  const [forceRmwImplementation, setForceRmwImplementation] = useState<string>("");
  const [forceRestart, setForceRestart] = useState(false);

  const defaultHost: string = hostArg ? hostArg : config?.params.host || "localhost";
  const [selectedHost, setSelectedHost] = useState<THostIp | null>({ host: defaultHost });
  const [hostList, setHostList] = useState<THostIp[]>([{ ip: "127.0.0.1", host: "localhost" }]);
  const [robotHostInputValue, setRobotHostInputValue] = useState("");

  const [inputMasterUri, setInputMasterUri] = useState(
    launchCfg?.params.ros1MasterUri.uri ? launchCfg?.params.ros1MasterUri.uri : "default"
  );
  const [optionsMasterUri, setOptionsMasterUri] = useLocalStorage("ConnectToProviderModal:optionsMasterUri", [
    "http://{HOST}:11311",
  ]);

  const [optionOverrideZenohConfig, setOptionOverrideZenohConfig] = useLocalStorage(
    "ConnectToProviderModal:optionOverrideZenohConfig",
    true
  );
  const [startCmdInfoExpanded, setStartCmdInfoExpanded] = useLocalStorage(
    "ConnectToProviderModal:startCmdInfoExpanded",
    true
  );

  useEffect(() => {
    updateStartParameter();
    updateTopics();
  }, []);

  useEffect(() => {
    launchCfg.params.host = selectedHost?.host || selectedHost?.ip || "";
    forceValuesUpdate();
  }, [selectedHost]);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  function updateTopics(): void {
    // trigger add new provider
    const newAcTsSet = new Set<string>();
    const newAcTopicSet = new Set<string>();
    for (const prov of rosCtx.providers) {
      for (const node of prov.rosNodes) {
        newAcTsSet.add(`${node.name}/*`);
        newAcTopicSet.add(`${node.name}/*`);
        for (const topic of node.publishers || []) {
          newAcTsSet.add(topic.name);
          newAcTsSet.add(topic.msg_type);
          newAcTopicSet.add(topic.name);
        }
        for (const topic of node.subscribers || []) {
          newAcTsSet.add(topic.name);
          newAcTsSet.add(topic.msg_type);
          newAcTopicSet.add(topic.name);
        }
        for (const service of node.services || []) {
          newAcTsSet.add(service.name);
          newAcTsSet.add(service.msg_type);
        }
      }
    }
    setTSList(Array.from(newAcTsSet));
    setTopicList(Array.from(newAcTopicSet));
  }

  function host2string(host: string | THostIp): string {
    if (Object.keys(host).includes("host")) {
      return `${(host as THostIp).host}`;
    }
    return host as string;
  }

  function host2label(host: string | THostIp): string {
    if (Object.keys(host).includes("host")) {
      const ho: THostIp = host as THostIp;
      return `${ho.host}${ho.ip ? ` | ${ho.ip}` : ""}`;
    }
    return host as string;
  }

  useEffect(() => {
    if (!rosCtx.systemInfo) return;
    if (!rosCtx.systemInfo.hosts) return;

    // update list of hosts
    const hostListLocal: THostIp[] = [];
    const hostSystem = rosCtx.systemInfo.hosts;
    // only add valid ipv4 addresses
    const ipv4format =
      /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    for (const h of hostSystem) {
      const ip = h[0];
      if (ip.match(ipv4format)) {
        const lastName = h[1].split(" ").pop();
        hostListLocal.push({ ip, host: lastName });
      }
    }
    hostListLocal.sort((a, b) => -b.host.localeCompare(a.host));
    if (hostListLocal.length === 0) {
      hostListLocal.push({ ip: "127.0.0.1", host: "localhost" });
    }
    setHostList(hostListLocal);
  }, [rosCtx.systemInfo]);

  function getRobotHosts(): string[] {
    const robotHosts: string[] = [];
    for (const h of launchCfg.params.discovery.robotHosts || []) {
      robotHosts.push(h);
    }
    // add temporal values to the list as well
    if (robotHostInputValue.length > 0) robotHosts.push(robotHostInputValue);
    return robotHosts;
  }

  function updateStartParameter(): void {
    launchCfg.params.currentRmwImpl = optionOverrideZenohConfig ? rosCtx.rosInfo?.rmwImplementation || "" : "";
    setStartCmdDaemon(launchCfg.daemonStartCmd().message);
    setStartCmdDiscovery(launchCfg.masterDiscoveryStartCmd().message);
    setStartCmdTtyd(launchCfg.terminalStartCmd().message);
    forceValuesUpdate();
  }

  const setRosVersion = useCallback(
    (rosVersion: string): void => {
      launchCfg.params.rosVersion = rosVersion;
      updateStartParameter();
    },
    [launchCfg]
  );

  function setMasterUri(masterUri: string): void {
    launchCfg.params.ros1MasterUri.uri = masterUri === "http://{HOST}:11311" ? "default" : masterUri;
    updateStartParameter();
  }

  function setName(name: string): void {
    launchCfg.params.name = name;
    updateStartParameter();
  }

  function setNetworkId(networkId: number): void {
    launchCfg.params.networkId = networkId;
    updateStartParameter();
  }

  function setHeartbeatHz(hz: number): void {
    launchCfg.params.discovery.heartbeatHz = hz;
    updateStartParameter();
  }

  function setRobotHostValues(robotHosts: string[]): void {
    launchCfg.params.discovery.robotHosts = robotHosts;
    updateStartParameter();
  }

  function setDoNotSync(doNotSync: string[]): void {
    launchCfg.params.sync.doNotSync = doNotSync;
    updateStartParameter();
  }

  function setSyncTopics(syncTopics: string[]): void {
    launchCfg.params.sync.syncTopics = syncTopics;
    updateStartParameter();
  }

  function setTerminalPath(path: string): void {
    launchCfg.params.terminal.path = path;
    updateStartParameter();
  }

  function setTerminalPort(port: number): void {
    launchCfg.params.terminal.port = port;
    updateStartParameter();
  }

  const createHostSelector = useMemo(() => {
    return (
      <Autocomplete
        id="autocomplete-host-search"
        size="small"
        fullWidth={true}
        autoHighlight
        clearOnEscape
        disableListWrap
        handleHomeEndKeys={false}
        noOptionsText="Package not found"
        options={hostList}
        getOptionLabel={(option: string | THostIp) => host2label(option)}
        freeSolo
        // This prevents warnings on invalid autocomplete values
        value={selectedHost}
        renderInput={(params) => (
          <TextField
            {...params}
            variant="standard"
            label="hostname | ip"
            placeholder=" hostname | ip "
            onBlur={(event) => {
              if (event.target.value) {
                setSelectedHost({ host: event.target.value.split(" ")[0] });
              }
            }}
          />
        )}
        renderOption={(props, option) => {
          return (
            <Stack
              {...(props as HTMLAttributes<HTMLDivElement>)}
              key={option.host}
              direction="row"
              // style={getHostStyle(option.host)}
            >
              <Typography sx={{ ml: 0 }} noWrap>
                {`${option.host} | ${option.ip}`}
              </Typography>
            </Stack>
          );
        }}
        onInputChange={(_event, newInputValue) => {
          setSelectedHost(newInputValue ? { host: newInputValue.split(" ")[0] } : null);
        }}
      />
    );
  }, [hostList, selectedHost]);

  return (
    <Stack
      pr={1}
      pl={1}
      spacing={1}
      height="100%"
      // width="100%"
      overflow="auto"
      sx={{ backgroundColor: backgroundColor }}
    >
      <Stack direction="row" spacing="1em" pt={1}>
        <Button
          type="submit"
          variant="contained"
          // display="flex"
          color="success"
          onClick={() => {
            onSave(launchCfg);
            emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(launchCfg.params.id));
          }}
          style={{ height: "3em", textAlign: "center" }}
          endIcon={<SaveIcon />}
          disabled={!launchCfg.params.host}
        >
          Save
        </Button>
        <Typography flexGrow={1} />
        <Button
          type="submit"
          variant="contained"
          // display="flex"
          color="error"
          onClick={() => {
            onDelete(launchCfg.params.id);
            emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(launchCfg.params.id));
          }}
          style={{ height: "3em", textAlign: "center" }}
          endIcon={<DeleteIcon />}
        >
          Delete
        </Button>
      </Stack>
      <Stack>
        <TextField
          id="name"
          // min={0}
          // max={99}
          label="Name"
          size="small"
          variant="outlined"
          style={{ minWidth: "9em" }}
          // fullWidth
          onChange={(e) => setName(`${e.target.value}`)}
          value={launchCfg.params.name || ""}
        />
      </Stack>
      {createHostSelector}
      <Stack direction="row" spacing="0.5em">
        <Stack flexGrow={1}>
          <Stack direction="row" alignItems="center" spacing="0.5em" margin="0.5em">
            {/* <FormControl> */}
            <RadioGroup
              row
              aria-labelledby="ros-version-select-group-label"
              name="ros-version-select-group"
              value={launchCfg.params.rosVersion}
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
              // min={0}
              // max={99}
              label="Network/Domain ID"
              size="small"
              variant="outlined"
              style={{ minWidth: "9em" }}
              InputProps={{ inputProps: { min: -1, max: 99 } }}
              // fullWidth
              onChange={(e) => setNetworkId(Number(`${e.target.value}`))}
              value={launchCfg.params.networkId}
            />
            {launchCfg.params.networkId === -1 && <Typography>Use default domain ID</Typography>}
          </Stack>

          <Stack
            direction="column"
            // divider={<Divider orientation="horizontal" />}
          >
            <FormGroup
              aria-label="position"
              row
              sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
            >
              <FormControlLabel
                control={
                  <Checkbox
                    checked={launchCfg.params.daemon.enable}
                    onChange={(event) => {
                      launchCfg.params.daemon.enable = event.target.checked;
                      updateStartParameter();
                    }}
                  />
                }
                label="Daemon Node"
                labelPlacement="end"
              />
            </FormGroup>
            {launchCfg.params.rosVersion === "2" && (
              <FormGroup aria-label="position" row>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={launchCfg.params.discovery.enable}
                      onChange={(event) => {
                        launchCfg.params.discovery.enable = event.target.checked;
                        updateStartParameter();
                      }}
                    />
                  }
                  label="Discovery Node"
                  labelPlacement="end"
                />
              </FormGroup>
            )}
            {launchCfg.params.rosVersion === "1" && (
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
                  <GridLegacy container>
                    <GridLegacy item xs={4}>
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
                              checked={launchCfg.params.discovery.enable}
                              onChange={(event) => {
                                launchCfg.params.discovery.enable = event.target.checked;
                                updateStartParameter();
                              }}
                            />
                          }
                          label="Discovery Node"
                          labelPlacement="end"
                        />
                      </FormGroup>
                    </GridLegacy>
                    <GridLegacy item xs={6} sx={{ alignSelf: "center" }}>
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
                          {`Heartbeat Hz: ${launchCfg.params.discovery.heartbeatHz}`}
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
                          {`Robot Hosts: [${getRobotHosts().join(",")}]`}
                        </Typography>
                      </Stack>
                    </GridLegacy>
                  </GridLegacy>
                </AccordionSummary>
                <AccordionDetails>
                  <Stack direction="column" divider={<Divider orientation="vertical" />}>
                    <TextField
                      type="number"
                      id="heartbeat-hz"
                      label="Heartbeat Hz"
                      size="small"
                      variant="outlined"
                      fullWidth
                      onChange={(e) => setHeartbeatHz(Number(`${e.target.value}`))}
                      value={launchCfg.params.discovery.heartbeatHz}
                      disabled={!launchCfg.params.discovery.enable}
                    />
                    <Autocomplete
                      handleHomeEndKeys={false}
                      disabled={!launchCfg.params.discovery.enable}
                      disablePortal
                      multiple
                      id="auto-complete-robot-hosts"
                      size="small"
                      options={hostList}
                      freeSolo
                      sx={{ margin: 0 }}
                      getOptionLabel={(option: string | THostIp) => host2label(option)}
                      renderInput={(params) => (
                        <TextField {...params} variant="outlined" label="Add Robot Hosts" placeholder="..." fullWidth />
                      )}
                      value={launchCfg.params.discovery.robotHosts}
                      onChange={(_event, newValue) => {
                        setRobotHostValues(newValue.map((item) => host2string(item)));
                      }}
                      inputValue={robotHostInputValue}
                      onInputChange={(_event, newInputValue) => {
                        setRobotHostInputValue(newInputValue);
                      }}
                      disableCloseOnSelect
                      renderOption={(props, option, { selected }) => (
                        <li {...props} key={`${host2label(option)}`} style={{ height: "1.5em" }}>
                          <Checkbox
                            icon={icon}
                            checkedIcon={checkedIcon}
                            // style={{ marginRight: 8 }}
                            checked={selected}
                            size="small"
                          />
                          {`${host2label(option)}`}
                        </li>
                      )}
                      slotProps={{
                        listbox: {
                          style: { maxHeight: 150 },
                        },
                        popper: {
                          placement: "top-start",
                        },
                      }}
                    />
                  </Stack>
                </AccordionDetails>
              </Accordion>
            )}
            {launchCfg.params.rosVersion === "1" && (
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
                  <GridLegacy container>
                    <GridLegacy item xs={4}>
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
                              checked={launchCfg.params.sync.enable}
                              onChange={(event) => {
                                launchCfg.params.sync.enable = event.target.checked;
                                updateStartParameter();
                              }}
                            />
                          }
                          label="Master Sync"
                          labelPlacement="end"
                        />
                      </FormGroup>
                    </GridLegacy>
                    <GridLegacy item xs={6} sx={{ alignSelf: "center" }}>
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
                          {`DoNotSync: [${launchCfg.params.sync.doNotSync.join()}]`}
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
                          {`SyncTopics: [${launchCfg.params.sync.syncTopics.join()}]`}
                        </Typography>
                      </Stack>
                    </GridLegacy>
                  </GridLegacy>
                </AccordionSummary>
                <AccordionDetails>
                  <Stack direction="column" divider={<Divider orientation="vertical" />}>
                    <Autocomplete
                      handleHomeEndKeys={false}
                      disablePortal
                      multiple
                      id="auto-complete-do-not-sync"
                      size="small"
                      options={tsList}
                      freeSolo
                      sx={{ margin: 0 }}
                      renderInput={(params) => <TextField {...params} variant="outlined" label="do not sync" />}
                      value={launchCfg.params.sync.doNotSync}
                      onChange={(_event, newValue) => {
                        setDoNotSync(newValue);
                      }}
                      // disableCloseOnSelect
                    />
                    <Autocomplete
                      handleHomeEndKeys={false}
                      disablePortal
                      multiple
                      id="auto-complete-sync-topics"
                      size="small"
                      options={topicList}
                      freeSolo
                      sx={{ margin: 0 }}
                      renderInput={(params) => <TextField {...params} variant="outlined" label="sync topics" />}
                      value={launchCfg.params.sync.syncTopics}
                      onChange={(_event, newValue) => {
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
                aria-controls="terminal_panel-content"
                id="terminal_panel-header"
                sx={{ pl: 0, margin: 0 }}
                style={{ margin: 0 }}
              >
                <GridLegacy container>
                  <GridLegacy item xs={4}>
                    <Stack
                      direction="row"
                      style={{ marginLeft: 0, paddingLeft: 0 }}
                      justifyItems="center"
                      alignItems="center"
                    >
                      <Checkbox
                        style={{ marginLeft: 0, paddingLeft: 0 }}
                        checked={launchCfg.params.terminal.enable}
                        onClick={(event) => {
                          event.stopPropagation();
                        }}
                        onChange={(event) => {
                          launchCfg.params.terminal.enable = event.target.checked;
                          updateStartParameter();
                        }}
                      />
                      <Typography>Terminal Manager</Typography>

                      <Tooltip
                        title={
                          <>
                            <Typography variant="body1">Install TTYD in the host using:</Typography>
                            <Stack mt={1} direction="row" justifyContent="center">
                              <Typography variant="body2">sudo snap install ttyd --classic</Typography>
                              <CopyButton value="sudo snap install ttyd --classic" />
                            </Stack>
                            <Link mt={2} href="https://github.com/tsl0922/ttyd" target="_blank" color="inherit">
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
                        <Stack
                          marginLeft="3px"
                          onClick={(event) => {
                            setOpenTerminalTooltip(!openTerminalTooltip);
                            event.stopPropagation();
                          }}
                        >
                          <InfoOutlinedIcon
                            sx={{
                              fontSize: "inherit",
                              color: "DodgerBlue",
                            }}
                          />
                        </Stack>
                      </Tooltip>
                    </Stack>
                  </GridLegacy>
                  <GridLegacy item xs={6} sx={{ alignSelf: "center" }}>
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
                        {`Path: ${launchCfg.params.terminal.path}`}
                      </Typography>
                      <Typography
                        variant="body2"
                        sx={{
                          color: grey[700],
                          fontWeight: "inherit",
                          flexGrow: 1,
                          ml: 0.5,
                        }}
                      >
                        {`Port: ${launchCfg.params.terminal.port}`}
                      </Typography>
                    </Stack>
                  </GridLegacy>
                </GridLegacy>
              </AccordionSummary>
              <AccordionDetails>
                <TextField
                  type="string"
                  id="terminal-path"
                  label="Path"
                  size="small"
                  variant="outlined"
                  fullWidth
                  onChange={(e) => setTerminalPath(`${e.target.value}`)}
                  value={launchCfg.params.terminal.path}
                  disabled={!launchCfg.params.terminal.enable}
                />
                <TextField
                  type="number"
                  id="terminal-port"
                  label="Port"
                  size="small"
                  variant="outlined"
                  fullWidth
                  onChange={(e) => setTerminalPort(Number(`${e.target.value}`))}
                  value={launchCfg.params.terminal.port}
                  disabled={!launchCfg.params.terminal.enable}
                />
              </AccordionDetails>
            </Accordion>
            {launchCfg.params.rosVersion === "1" && (
              <Accordion>
                <AccordionSummary
                  expandIcon={<ExpandMoreIcon />}
                  aria-controls="discovery_panel-content"
                  id="master-uri-header"
                  sx={{ pl: 0 }}
                >
                  <GridLegacy container>
                    <GridLegacy item xs={4}>
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
                              size="small"
                              checked={launchCfg.params.ros1MasterUri.enable}
                              onChange={(event) => {
                                launchCfg.params.ros1MasterUri.enable = event.target.checked;
                                updateStartParameter();
                              }}
                            />
                          }
                          label={
                            <Tooltip
                              title={"The new ROS_MASTER_URI is prefixed when system nodes are started."}
                              placement="bottom"
                              disableInteractive
                            >
                              <Typography>ROS_MASTER_URI</Typography>
                            </Tooltip>
                          }
                          labelPlacement="end"
                        />
                      </FormGroup>
                    </GridLegacy>
                    <GridLegacy item xs={6} sx={{ alignSelf: "center" }}>
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
                          {`${launchCfg.params.ros1MasterUri.uri}`}
                        </Typography>
                      </Stack>
                    </GridLegacy>
                  </GridLegacy>
                </AccordionSummary>
                <AccordionDetails>
                  <Stack direction="column" divider={<Divider orientation="vertical" />}>
                    <Tooltip
                      title={
                        <>
                          <Typography variant="body2">Enter: save entry</Typography>
                          <Typography variant="body2">Delete: remove selected entry from options list</Typography>
                        </>
                      }
                      placement="bottom"
                      disableInteractive
                    >
                      <Autocomplete
                        disableListWrap
                        handleHomeEndKeys={false}
                        autoHighlight
                        value={inputMasterUri}
                        options={optionsMasterUri}
                        isOptionEqualToValue={(option, value) =>
                          option === value || (option === "http://{HOST}:11311" && value === "default")
                        }
                        getOptionLabel={(option) => option}
                        onInputChange={(_e, newValue) => {
                          setMasterUri(newValue);
                          setInputMasterUri(newValue);
                        }}
                        size="small"
                        fullWidth
                        freeSolo
                        renderInput={(params) => (
                          <TextField
                            {...params}
                            // label="ROS_MASTER_URI"
                            variant="outlined"
                            onKeyDown={(e) => {
                              if (e.key === "Enter" && optionsMasterUri.findIndex((o) => o === inputMasterUri) === -1) {
                                // add to options
                                setOptionsMasterUri((prev) => [
                                  ...prev.filter((item) => item !== inputMasterUri),
                                  inputMasterUri,
                                ]);
                              }
                              if (e.key === "Delete") {
                                // remove entry from options
                                setOptionsMasterUri((prev) => [
                                  ...prev.filter((item) => item !== inputMasterUri || item === "http://{HOST}:11311"),
                                ]);
                              }
                            }}
                          />
                        )}
                      />
                    </Tooltip>
                  </Stack>
                </AccordionDetails>
              </Accordion>
            )}
            <Divider />
            <FormGroup
              aria-label="position"
              row
              sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
            >
              <FormControlLabel
                disabled={!rosCtx.rosInfo?.rmwImplementation}
                control={
                  <Checkbox
                    size="small"
                    checked={!!forceRmwImplementation}
                    onChange={(event) => {
                      launchCfg.params.rmwImplementation = event.target.checked
                        ? rosCtx.rosInfo?.rmwImplementation
                        : undefined;
                      setForceRmwImplementation(launchCfg.params.rmwImplementation || "");
                      updateStartParameter();
                    }}
                  />
                }
                label={`force use ${rosCtx.rosInfo?.rmwImplementation || " RMW_IMPLEMENTATION (environment variable not set)"}`}
                labelPlacement="end"
              />
            </FormGroup>

            <Tooltip
              title={
                <Typography variant="body2">
                  Prepend zenoh multicast configuration with new port (7448 + ROS_DOMAIN_ID)
                </Typography>
              }
              disableInteractive
            >
              <FormGroup
                aria-label="position"
                row
                sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
              >
                <FormControlLabel
                  control={
                    <Checkbox
                      size="small"
                      checked={!!optionOverrideZenohConfig}
                      onChange={(event) => {
                        launchCfg.params.currentRmwImpl = event.target.checked
                          ? rosCtx.rosInfo?.rmwImplementation || ""
                          : "";
                        setOptionOverrideZenohConfig(event.target.checked);
                        updateStartParameter();
                      }}
                    />
                  }
                  label={"override zenoh config"}
                  labelPlacement="end"
                />
              </FormGroup>
            </Tooltip>

            <FormGroup
              aria-label="position"
              row
              sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
            >
              <FormControlLabel
                disabled={!(launchCfg.params.daemon.enable && launchCfg.params.discovery.enable)}
                control={
                  <Checkbox
                    size="small"
                    checked={forceRestart}
                    onChange={(event) => {
                      launchCfg.params.force = { stop: event.target.checked, start: event.target.checked };
                      updateStartParameter();
                      setForceRestart(event.target.checked);
                    }}
                  />
                }
                label="force restart MAS nodes"
                labelPlacement="end"
              />
            </FormGroup>
          </Stack>
          <Divider />
          <AccordionAdv
            disabled={!window.commandExecutor}
            expanded={startCmdInfoExpanded}
            onChange={(_event, expanded) => {
              setStartCmdInfoExpanded(expanded);
            }}
          >
            <AccordionSummary
              // disabled={!startSystemNodes}
              expandIcon={<ExpandMoreIcon />}
              aria-controls="start-commands"
              id="start-commands"
              sx={{ pl: 0, paddingBottom: 0 }}
            >
              <Stack direction="row" alignItems="center" spacing="0.3em">
                <ChevronRightIcon fontSize="inherit" />
                <Typography variant="subtitle1">Start commands:</Typography>
              </Stack>
            </AccordionSummary>
            <AccordionDetails>
              <Stack
                direction="column"
                // divider={<Divider orientation="horizontal" />}
              >
                {launchCfg.params.daemon.enable && (
                  <Stack direction="row">
                    <CopyButton value={startCmdDaemon} fontSize="0.7em" />
                    <Typography
                      variant="body2"
                      component="pre"
                      sx={{
                        fontFamily: "monospace",
                        backgroundColor: "#f5f5f5",
                        padding: 1,
                        borderRadius: 1,
                        overflowWrap: "break-word",
                        wordBreak: "break-word",
                        whiteSpace: "pre-wrap",
                      }}
                    >
                      {startCmdDaemon}
                    </Typography>
                  </Stack>
                )}
                {launchCfg.params.discovery.enable && (
                  <Stack direction="row">
                    <CopyButton value={startCmdDiscovery} fontSize="0.7em" />
                    <Typography
                      variant="body2"
                      component="pre"
                      sx={{
                        fontFamily: "monospace",
                        backgroundColor: "#f5f5f5",
                        padding: 1,
                        borderRadius: 1,
                        overflowWrap: "break-word",
                        wordBreak: "break-word",
                        whiteSpace: "pre-wrap",
                      }}
                    >
                      {startCmdDiscovery}
                    </Typography>
                  </Stack>
                )}
                {launchCfg.params.terminal.enable && (
                  <Stack direction="row">
                    <CopyButton value={startCmdTtyd} fontSize="0.7em" />
                    <Typography
                      variant="body2"
                      component="pre"
                      sx={{
                        fontFamily: "monospace",
                        backgroundColor: "#f5f5f5",
                        padding: 1,
                        borderRadius: 1,
                        overflowWrap: "break-word",
                        wordBreak: "break-word",
                        whiteSpace: "pre-wrap",
                      }}
                    >
                      {startCmdTtyd}
                    </Typography>
                  </Stack>
                )}
              </Stack>
            </AccordionDetails>
          </AccordionAdv>
        </Stack>
      </Stack>{" "}
    </Stack>
  );
}
