import BookmarkBorderOutlinedIcon from "@mui/icons-material/BookmarkBorderOutlined";
import CheckBoxIcon from "@mui/icons-material/CheckBox";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import SettingsOutlinedIcon from "@mui/icons-material/SettingsOutlined";
import {
  // AccordionSummary,
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
  GridLegacy,
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
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary from "@mui/material/AccordionSummary";
import { grey } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import { ForwardedRef, forwardRef, useCallback, useContext, useEffect, useMemo, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext, getDefaultPortFromRos } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import ProviderLaunchConfiguration from "@/renderer/models/ProviderLaunchConfiguration";
import Provider from "@/renderer/providers/Provider";
import { EVENT_PROVIDER_ROS_NODES } from "@/renderer/providers/eventTypes";
import { generateUniqueId } from "@/renderer/utils";
import CopyButton from "../UI/CopyButton";
import DraggablePaper from "../UI/DraggablePaper";

const AccordionAdv = styled((props: AccordionProps) => <MuiAccordion disableGutters elevation={0} square {...props} />)(
  ({ theme }) => ({
    border: "none",
    backgroundColor: theme.palette.mode === "dark" ? "rgba(0, 0, 0, .00)" : "rgba(255, 255, 255, .00)",
    ".MuiAccordionSummary-content": { margin: 0 },
    "&:before": {
      display: "none",
    },
    paddingTop: "12px",
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
  minHeight: 32,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  paddingTop: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

type THostIp = { host: string; ip?: string };
type TSavedStartConfiguration = {
  id: string;
  hosts: THostIp[];
  params: ProviderLaunchConfiguration;
};
const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;
const DEFAULT_PARAMETER: ProviderLaunchConfiguration = new ProviderLaunchConfiguration("localhost", "2");

/**
 * Simple is object check.
 */
function isObject(item: object | unknown[] | undefined): boolean | undefined {
  return item && typeof item === "object" && !Array.isArray(item);
}

/**
 * Deep merge modifier into org and returns new object
 */
function mergeDeepConfig(
  org: ProviderLaunchConfiguration,
  modifier: ProviderLaunchConfiguration
): ProviderLaunchConfiguration {
  const result: ProviderLaunchConfiguration = {} as ProviderLaunchConfiguration;
  if (isObject(org)) {
    for (const key in org) {
      if (isObject(org[key])) {
        if (modifier && isObject(modifier[key])) {
          result[key] = mergeDeepConfig(org[key], modifier[key]);
        } else {
          result[key] = org[key];
        }
      } else {
        if (modifier && modifier[key] !== undefined) {
          if (!isObject(modifier[key])) {
            result[key] = modifier[key];
          }
        } else {
          result[key] = org[key];
        }
      }
    }
  }
  return result;
}

interface ConnectToProviderModalProps {
  onCloseDialog: () => void;
}

const ConnectToProviderModal = forwardRef<HTMLDivElement, ConnectToProviderModalProps>(
  function ConnectToProviderModal(props, ref) {
    const { onCloseDialog = (): void => {} } = props;

    const rosCtx = useContext(RosContext);
    const settingsCtx = useContext(SettingsContext);
    const logCtx = useContext(LoggingContext);

    const [open, setOpen] = useState(true);
    const [openTerminalTooltip, setOpenTerminalTooltip] = useState(false);

    const [hostList, setHostList] = useState<THostIp[]>([{ ip: "127.0.0.1", host: "localhost" }]);
    const [hostValues, setHostValues] = useState<THostIp[]>([{ host: "localhost" }]);
    const [hostInputValue, setHostInputValue] = useState<string>("");
    const [robotHostInputValue, setRobotHostInputValue] = useState("");

    const [optionNetworkId, setOptionNetworkId] = useLocalStorage<number>(
      "ConnectToProviderModal:optionNetworkId",
      DEFAULT_PARAMETER.networkId
    );

    DEFAULT_PARAMETER.networkId = import.meta.env.VITE_ROS_DOMAIN_ID
      ? Number.parseInt(import.meta.env.VITE_ROS_DOMAIN_ID)
      : Number.parseInt(`${rosCtx.rosInfo?.domainId || optionNetworkId}`);
    DEFAULT_PARAMETER.rmwImplementation = import.meta.env.VITE_RMW_IMPLEMENTATION
      ? import.meta.env.VITE_RMW_IMPLEMENTATION
      : undefined;
    DEFAULT_PARAMETER.rosVersion = import.meta.env.VITE_ROS_VERSION
      ? import.meta.env.VITE_ROS_VERSION
      : (settingsCtx.get("rosVersion") as string);
    const [startParameterDefault, setStartConfigurationsDefault] = useLocalStorage<ProviderLaunchConfiguration>(
      "ConnectToProviderModal:startParameter",
      DEFAULT_PARAMETER
    );
    const [startParameter, setStartParameter] = useState<ProviderLaunchConfiguration>(
      mergeDeepConfig(DEFAULT_PARAMETER, startParameterDefault) as ProviderLaunchConfiguration
    );
    const [startConfigurations, setStartConfigurations] = useLocalStorage<TSavedStartConfiguration[]>(
      "ConnectToProviderModal:startConfigurations",
      []
    );
    const [selectedHistory, setSelectedHistory] = useState("");
    const [forceRmwImplementation, setForceRmwImplementation] = useState<string>("");
    const [forceRestart, setForceRestart] = useState(false);

    const [inputMasterUri, setInputMasterUri] = useState(
      startParameter?.ros1MasterUri.uri ? startParameter?.ros1MasterUri.uri : "default"
    );
    const [optionsMasterUri, setOptionsMasterUri] = useLocalStorage("ConnectToProviderModal:optionsMasterUri", [
      "http://{HOST}:11311",
    ]);

    const [tsList, setTSList] = useState<string[]>([]);
    const [topicList, setTopicList] = useState<string[]>([]);

    const [startProviderDescription, setStartProviderDescription] = useState("");
    const [startProviderIsSubmitting, setStartProviderIsSubmitting] = useState(false);

    function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
      if (reason && reason === "backdropClick") return;
      setOpen(false);
      setOpenTerminalTooltip(false);
      onCloseDialog();
      // setStartProviderIsSubmitting(false);
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
      setHostValues([{ host: "localhost" }]);
    }, [rosCtx.systemInfo]);

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

    useEffect(() => {
      updateTopics();
      if (!startParameter.networkId && import.meta.env.VITE_ROS_DOMAIN_ID) {
        setNetworkId(Number.parseInt(import.meta.env.VITE_ROS_DOMAIN_ID));
      }
    }, []);

    useCustomEventListener(EVENT_PROVIDER_ROS_NODES, () => {
      updateTopics();
    });

    function host2string(host: string | THostIp): string {
      if (Object.keys(host).includes("host")) {
        return `${(host as THostIp).host}`;
      }
      return host as string;
    }

    function host2label(host: string | THostIp): string {
      if (Object.keys(host).includes("host")) {
        const ho: THostIp = host as THostIp;
        return `${ho.host}${ho.ip ? ` [${ho.ip}]` : ""}`;
      }
      return host as string;
    }

    function host2host(host: string | THostIp): THostIp {
      if (Object.keys(host).includes("host")) {
        return host as THostIp;
      }
      return { host: host } as THostIp;
    }

    function getHosts(): string[] {
      const hosts: string[] = hostValues.map((h: THostIp) => h.host);
      if (hostInputValue !== "" && !hosts.includes(hostInputValue)) {
        hosts.push(hostInputValue);
      }
      return hosts;
    }

    function getRobotHosts(): string[] {
      const robotHosts: string[] = [];
      for (const h of startParameter.discovery.robotHosts || []) {
        robotHosts.push(h);
      }
      // add temporal values to the list as well
      if (robotHostInputValue.length > 0) robotHosts.push(robotHostInputValue);
      return robotHosts;
    }

    function updateStartParameter(): void {
      setStartParameter(JSON.parse(JSON.stringify(startParameter)));
    }

    const setRosVersion = useCallback(
      (rosVersion: string): void => {
        startParameter.rosVersion = rosVersion;
        updateStartParameter();
      },
      [startParameter]
    );

    function setMasterUri(masterUri: string): void {
      startParameter.ros1MasterUri.uri = masterUri === "http://{HOST}:11311" ? "default" : masterUri;
      updateStartParameter();
    }

    function setNetworkId(networkId: number): void {
      startParameter.networkId = networkId;
      setOptionNetworkId(networkId);
      updateStartParameter();
    }

    function setHeartbeatHz(hz: number): void {
      startParameter.discovery.heartbeatHz = hz;
      updateStartParameter();
    }

    function setRobotHostValues(robotHosts: string[]): void {
      startParameter.discovery.robotHosts = robotHosts;
      updateStartParameter();
    }

    function setDoNotSync(doNotSync: string[]): void {
      startParameter.sync.doNotSync = doNotSync;
      updateStartParameter();
    }

    function setSyncTopics(syncTopics: string[]): void {
      startParameter.sync.syncTopics = syncTopics;
      updateStartParameter();
    }

    function setTerminalPath(path: string): void {
      startParameter.terminal.path = path;
      updateStartParameter();
    }

    function setTerminalPort(port: number): void {
      startParameter.terminal.port = port;
      updateStartParameter();
    }

    function stringifyStartConfig(cfg: TSavedStartConfiguration): string {
      let result = `${cfg.hosts.map((item) => host2label(item)).join()}; ros${cfg.params.rosVersion}`;
      if (cfg.params.rosVersion === "1") {
        result = `${result}; network id: ${cfg.params.networkId}`;
      } else {
        result = `${result}; domain id: ${cfg.params.networkId}`;
      }
      const robotHosts = cfg.params.discovery.robotHosts ? cfg.params.discovery.robotHosts.join(",") : [];
      if (robotHosts.length > 0) {
        result = `${result}; robotHosts: [${robotHosts}]`;
      }
      if (cfg.params.sync.enable) {
        result = `${result}; +sync: doNotSync[${cfg.params.sync.doNotSync.join(",")}], syncTopics[${cfg.params.sync.syncTopics.join(",")}]`;
      }
      if (cfg.params.terminal?.port) {
        result = `${result}; ttyd port: ${cfg.params.terminal.port}`;
      }
      return result;
    }

    function createLaunchConfigFor(host: string): ProviderLaunchConfiguration {
      const launchCfg = new ProviderLaunchConfiguration(host, startParameter.rosVersion);
      launchCfg.daemon.enable = startParameter.daemon.enable;
      launchCfg.discovery.enable = startParameter.discovery.enable;
      if (startParameter.networkId) launchCfg.networkId = startParameter.networkId;
      launchCfg.rmwImplementation = startParameter.rmwImplementation;
      launchCfg.discovery.heartbeatHz = startParameter.discovery.heartbeatHz;
      if (startParameter.discovery.robotHosts && startParameter.discovery.robotHosts.length > 0)
        launchCfg.discovery.robotHosts = startParameter.discovery.robotHosts;
      launchCfg.sync.enable = startParameter.sync.enable;
      launchCfg.sync.doNotSync = startParameter.sync.doNotSync;
      launchCfg.sync.syncTopics = startParameter.sync.syncTopics;
      launchCfg.terminal.enable = startParameter.terminal.enable;
      launchCfg.terminal.port = startParameter.terminal.port;
      launchCfg.terminal.path = startParameter.terminal.path;
      launchCfg.autoConnect = true;
      launchCfg.autostart = rosCtx.isLocalHost(host);
      launchCfg.force.stop = startParameter.force.stop;
      launchCfg.force.start = startParameter.force.start;
      if (startParameter.ros1MasterUri.enable && startParameter.ros1MasterUri.uri !== "default") {
        launchCfg.ros1MasterUri.enable = startParameter.ros1MasterUri.enable;
        launchCfg.ros1MasterUri.uri = startParameter.ros1MasterUri.uri.replace("{HOST}", host);
      }
      return launchCfg;
    }

    function createCommandsFor(launchCfg: ProviderLaunchConfiguration): string[] {
      const commands: string[] = [];
      if (launchCfg.daemon.enable) {
        commands.push(launchCfg.daemonStartCmd().message);
      }
      if (launchCfg.discovery.enable) {
        commands.push(launchCfg.masterDiscoveryStartCmd().message);
      }
      if (launchCfg.sync.enable) {
        commands.push(launchCfg.masterSyncStartCmd().message);
      }
      if (launchCfg.terminal.enable) {
        commands.push(launchCfg.terminalStartCmd().message);
      }
      return commands;
    }

    async function handleStartProvider(): Promise<void> {
      setStartProviderDescription("Starting nodes on selected hosts");
      setStartProviderIsSubmitting(true);

      const launchConfigs: ProviderLaunchConfiguration[] = [];
      const hosts: THostIp[] = hostValues;
      if (
        hostInputValue !== "" &&
        hosts.filter((h: THostIp) => h.host === hostInputValue || h.ip === hostInputValue).length === 0
      ) {
        hosts.push({ host: hostInputValue });
      }

      // update recent start configurations
      const startCfg: TSavedStartConfiguration = {
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
      for (const host of hosts) {
        const launchCfg: ProviderLaunchConfiguration = createLaunchConfigFor(host.host);
        launchConfigs.push(launchCfg);
      }

      // start provider
      let successStart = true;
      launchConfigs.map(async (launchCfg) => {
        if (!(await rosCtx.startConfig(launchCfg, null))) {
          successStart = false;
        }
      });
      if (successStart) {
        setStartProviderDescription("");
      }

      // remove loading message
      handleClose("confirmed");
      setTimeout(() => {
        setStartProviderIsSubmitting(false);
        setStartProviderDescription("");
      }, 500);
      return Promise.resolve();
    }

    async function handleJoinProvider(): Promise<void> {
      setStartProviderIsSubmitting(true);
      const hosts: THostIp[] = hostValues;
      if (
        hostInputValue !== "" &&
        hosts.filter((h: THostIp) => h.host === hostInputValue || h.ip === hostInputValue).length === 0
      ) {
        hosts.push({ host: hostInputValue });
      }
      const port = startParameter.port
        ? startParameter.port
        : getDefaultPortFromRos(
            Provider.defaultType,
            startParameter.rosVersion,
            startParameter.ros1MasterUri.uri,
            startParameter.networkId
          );
      // join each host separately
      await Promise.all(
        hosts.map(async (remoteHost: THostIp) => {
          let host: string = remoteHost.host;
          if (remoteHost.host) host = remoteHost.host;
          setStartProviderDescription(`Connecting to ${host} ...`);
          console.log(`connecting to ${host}:${port}`);
          const newProvider = new Provider(
            settingsCtx,
            host,
            startParameter.rosVersion,
            port,
            undefined,
            undefined,
            logCtx
          );
          const launchCfg = createLaunchConfigFor(host);
          newProvider.startConfiguration = launchCfg;
          await rosCtx.connectToProvider(newProvider);
        })
      );
      // update recent start configurations
      const startCfg: TSavedStartConfiguration = {
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
      // remove loading message and close dialog
      handleClose("confirmed");
      setTimeout(() => {
        setStartProviderIsSubmitting(false);
        setStartProviderDescription("");
      }, 500);
      return Promise.resolve();
    }

    useEffect(() => {
      // use system ros version
      if (rosCtx.rosInfo?.version) {
        setRosVersion(rosCtx.rosInfo?.version);
      }
      // do not include to dependency (loop!): setRosVersion
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
              backgroundColor: settingsCtx.get("backgroundColor") as string,
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
                        setStartParameter(
                          mergeDeepConfig(DEFAULT_PARAMETER, cfg.params) as ProviderLaunchConfiguration
                        );
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
                            onClick={(event) => {
                              setStartConfigurations((prev) => prev.filter((pCfg) => pCfg.id !== cfg.id));
                              event?.stopPropagation();
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

    const dialogRef = useRef(ref);

    return (
      <Dialog
        // keepMounted
        id="connect-to-ros-modal"
        scroll="paper"
        ref={dialogRef as ForwardedRef<HTMLDivElement>}
        PaperProps={{
          component: DraggablePaper,
          dialogRef: dialogRef,
        }}
        aria-labelledby="draggable-dialog-title"
        fullWidth
        maxWidth="md"
        open={open}
        onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
        // disableEscapeKeyDown
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
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
                  handleHomeEndKeys={false}
                  disablePortal
                  multiple
                  id="auto-complete-hosts"
                  size="medium"
                  options={hostList}
                  freeSolo
                  ListboxProps={{ style: { maxHeight: 150 } }}
                  sx={{ margin: 0 }}
                  getOptionLabel={(option: string | THostIp) => host2label(option)}
                  renderInput={(params) => (
                    <TextField {...params} variant="standard" label="Add Hosts" placeholder=" add " />
                  )}
                  value={hostValues}
                  onChange={(_event, newValue) => {
                    setHostValues(newValue.map((item) => host2host(item)));
                  }}
                  inputValue={hostInputValue}
                  onInputChange={(_event, newInputValue) => {
                    setHostInputValue(newInputValue);
                  }}
                  disableCloseOnSelect
                  renderOption={(props, option, { selected }) => (
                    <li {...props} key={`${host2label(option)}`} style={{ height: "1.5em" }}>
                      <Checkbox
                        icon={icon}
                        checkedIcon={checkedIcon}
                        style={{ marginRight: 8 }}
                        checked={selected}
                        size="small"
                      />
                      {`${host2label(option)}`}
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
                  // min={0}
                  // max={99}
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
                        !window.commandExecutor || (hostValues.length === 0 && hostInputValue === "")
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
                    <Tooltip
                      title={"Copy terminal commands to start nodes manually. Only for first host!"}
                      placement="right"
                      disableInteractive
                    >
                      <IconButton
                        sx={{ color: (theme) => theme.palette.text.disabled, paddingTop: 0, paddingBottom: 0 }}
                        size="small"
                        component="span"
                        onClick={() => {
                          const hosts = getHosts();
                          if (hosts.length > 0) {
                            const launchCfg = createLaunchConfigFor(hosts[0]);
                            const commands = createCommandsFor(launchCfg);
                            navigator.clipboard.writeText(commands.join("\n"));
                            logCtx.success("Commands copied!");
                          }
                        }}
                      >
                        <ContentCopyIcon sx={{ fontSize: "inherit" }} />
                      </IconButton>
                    </Tooltip>
                  </Stack>
                )}
              </Stack>
              <AccordionAdv disabled={!window.commandExecutor}>
                <AccordionSummary
                  // disabled={!startSystemNodes}
                  expandIcon={<ExpandMoreIcon />}
                  aria-controls="advanced-options"
                  id="advanced-options"
                  sx={{ pl: 0, paddingBottom: 0 }}
                >
                  <Stack direction="row" alignItems="center" spacing="0.3em">
                    <SettingsOutlinedIcon fontSize="inherit" />
                    <Typography variant="subtitle1">Advanced parameters:</Typography>
                  </Stack>
                </AccordionSummary>
                <AccordionDetails>
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
                            checked={startParameter.daemon.enable}
                            onChange={(event) => {
                              startParameter.daemon.enable = event.target.checked;
                              updateStartParameter();
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
                              checked={startParameter.discovery.enable}
                              onChange={(event) => {
                                startParameter.discovery.enable = event.target.checked;
                                updateStartParameter();
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
                                      checked={startParameter.discovery.enable}
                                      onChange={(event) => {
                                        startParameter.discovery.enable = event.target.checked;
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
                                  {`Heartbeat Hz: ${startParameter.discovery.heartbeatHz}`}
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
                              value={startParameter.discovery.heartbeatHz}
                              disabled={!startParameter.discovery.enable}
                            />
                            <Autocomplete
                              handleHomeEndKeys={false}
                              disabled={!startParameter.discovery.enable}
                              disablePortal
                              multiple
                              id="auto-complete-robot-hosts"
                              size="small"
                              options={hostList}
                              freeSolo
                              sx={{ margin: 0 }}
                              ListboxProps={{ style: { maxHeight: 150 } }}
                              getOptionLabel={(option: string | THostIp) => host2label(option)}
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
                            />
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
                                      checked={startParameter.sync.enable}
                                      onChange={(event) => {
                                        startParameter.sync.enable = event.target.checked;
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
                              value={startParameter.sync.doNotSync}
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
                              value={startParameter.sync.syncTopics}
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
                                checked={startParameter.terminal.enable}
                                onClick={(event) => {
                                  event.stopPropagation();
                                }}
                                onChange={(event) => {
                                  startParameter.terminal.enable = event.target.checked;
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
                                {`Path: ${startParameter.terminal.path}`}
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
                                {`Port: ${startParameter.terminal.port}`}
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
                          value={startParameter.terminal.path}
                          disabled={!startParameter.terminal.enable}
                        />
                        <TextField
                          type="number"
                          id="terminal-port"
                          label="Port"
                          size="small"
                          variant="outlined"
                          fullWidth
                          onChange={(e) => setTerminalPort(Number(`${e.target.value}`))}
                          value={startParameter.terminal.port}
                          disabled={!startParameter.terminal.enable}
                        />
                      </AccordionDetails>
                    </Accordion>
                    {startParameter.rosVersion === "1" && (
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
                                      checked={startParameter.ros1MasterUri.enable}
                                      onChange={(event) => {
                                        startParameter.ros1MasterUri.enable = event.target.checked;
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
                                  {`${startParameter.ros1MasterUri.uri}`}
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
                                  <Typography variant="body2">
                                    Delete: remove selected entry from options list
                                  </Typography>
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
                                      if (
                                        e.key === "Enter" &&
                                        optionsMasterUri.findIndex((o) => o === inputMasterUri) === -1
                                      ) {
                                        // add to options
                                        setOptionsMasterUri((prev) => [
                                          ...prev.filter((item) => item !== inputMasterUri),
                                          inputMasterUri,
                                        ]);
                                      }
                                      if (e.key === "Delete") {
                                        // remove entry from options
                                        setOptionsMasterUri((prev) => [
                                          ...prev.filter(
                                            (item) => item !== inputMasterUri || item === "http://{HOST}:11311"
                                          ),
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
                              startParameter.rmwImplementation = event.target.checked
                                ? rosCtx.rosInfo?.rmwImplementation
                                : undefined;
                              setForceRmwImplementation(startParameter.rmwImplementation || "");
                              updateStartParameter();
                            }}
                          />
                        }
                        label={`force use ${rosCtx.rosInfo?.rmwImplementation || " RMW_IMPLEMENTATION (environment variable not set)"}`}
                        labelPlacement="end"
                      />
                    </FormGroup>

                    <FormGroup
                      aria-label="position"
                      row
                      sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
                    >
                      <FormControlLabel
                        disabled={!(startParameter.daemon.enable && startParameter.discovery.enable)}
                        control={
                          <Checkbox
                            size="small"
                            checked={forceRestart}
                            onChange={(event) => {
                              startParameter.force = { stop: event.target.checked, start: event.target.checked };
                              updateStartParameter();
                              setForceRestart(event.target.checked);
                            }}
                          />
                        }
                        label="force restart MAS nodes"
                        labelPlacement="end"
                      />
                    </FormGroup>
                    {/* <FormGroup
                      aria-label="position"
                      row
                      sx={{ "&:hover": { backgroundColor: (theme) => theme.palette.action.hover } }}
                    >
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
                    </FormGroup> */}
                    <Box mb="0.6em">
                      <Button
                        size="small"
                        type="submit"
                        variant="outlined"
                        // display="flex"
                        color="info"
                        onClick={(event) => {
                          setStartConfigurationsDefault(
                            mergeDeepConfig(DEFAULT_PARAMETER, startParameter) as ProviderLaunchConfiguration
                          );
                          event.stopPropagation();
                        }}
                        style={{
                          height: "1.5em",
                          textTransform: "none",
                        }}
                        startIcon={<BookmarkBorderOutlinedIcon />}
                      >
                        Save parameters as default
                      </Button>
                    </Box>
                    <Box>
                      <Button
                        size="small"
                        type="submit"
                        variant="outlined"
                        // display="flex"
                        color="info"
                        onClick={(event) => {
                          setOptionNetworkId(0);
                          DEFAULT_PARAMETER.networkId = 0;
                          setStartParameter(DEFAULT_PARAMETER);
                          setForceRestart(false);
                          setSelectedHistory("");
                          window.localStorage.removeItem("ConnectToProviderModal:startParameter");
                          window.localStorage.removeItem("ConnectToProviderModal:optionNetworkId");
                          event.stopPropagation();
                        }}
                        style={{
                          height: "1.5em",
                          textTransform: "none",
                        }}
                        startIcon={<RestartAltIcon />}
                      >
                        Reset advanced parameters to default
                      </Button>
                    </Box>
                  </Stack>
                </AccordionDetails>
              </AccordionAdv>
            </Stack>
          </Stack>
        </DialogContent>
        <DialogActions>
          <Button
            color="primary"
            onClick={() => handleClose("cancel")}
            variant="text"
            // style={{ height: 40, textAlign: 'center' }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
    );
  }
);

export default ConnectToProviderModal;
