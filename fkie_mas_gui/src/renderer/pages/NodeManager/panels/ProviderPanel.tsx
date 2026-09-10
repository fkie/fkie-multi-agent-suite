import AddIcon from "@mui/icons-material/Add";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import RefreshIcon from "@mui/icons-material/Refresh";
import SettingsOutlinedIcon from "@mui/icons-material/SettingsOutlined";
import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  Stack,
  Table,
  TableBody,
  TableContainer,
  Tooltip,
  Typography,
} from "@mui/material";
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary from "@mui/material/AccordionSummary";
import { styled } from "@mui/material/styles";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "@/renderer/components/layout";
import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import { DraggablePaper } from "@/renderer/components/UI";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useAppState } from "@/renderer/hooks/useAppState";
import { useCliArgs } from "@/renderer/hooks/useCliArgs";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import { RmwSelection, TProviderLaunchParams, ZenohEnvSelection } from "@/renderer/models/ProviderLaunchConfiguration";
import { EVENT_PROVIDER_STATE } from "@/renderer/providers/eventTypes";
import Provider, { generateProviderId } from "@/renderer/providers/Provider";
import { isElectron } from "@/renderer/utils/popout";
import { emitOpenComponent } from "../../../components/layout/events";
import ProviderPanelRow from "./ProviderPanelRow";
import ProviderPanelRowCfg from "./ProviderPanelRowCfg";

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

const AccordionSummary = styled(MuiAccordionSummary)(({ theme }) => ({
  paddingTop: 0,
  margin: 0,
  minHeight: 26,
  borderTop: "1px solid rgba(0, 0, 0, .125)",
  background: theme.palette.background.default,
}));

const AccordionDetails = styled(MuiAccordionDetails)(() => ({
  paddingTop: 0,
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

export default function ProviderPanel(): JSX.Element {
  const rosCtx = useRosContext();
  const cliCtx = useCliArgs();
  const [expandedProviderCfg, setExpandedProviderCfg] = useState(true);
  const [noSourcedROS, setNoSourcedROS] = useState(false);
  const [noDomainId, setNoDomainId] = useState(false);
  const [noRosVersion, setNoRosVersion] = useState(false);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState<Provider[]>([]);
  const [filterText, setFilterText] = useState("");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [buttonLocation] = useSetting<string>("buttonLocation");

  const { value: startConfigurations } = useAppState<TProviderLaunchParams[]>("hosts", "configurations", [], {
    version: 1,
    migrateFrom: {
      localStorageKey: "Provider:startConfigurations",
    },
    migrate: (oldValue, oldVersion) => {
      if (oldVersion === undefined) {
        return oldValue as TProviderLaunchParams[];
      }
      return [];
    },
  });

  const [showStartConfigurations, setShowStartConfigurations] = useState<ProviderLaunchConfiguration[]>([]);
  const { value: openHintDialog, set: setOpenHintDialog } = useAppState<boolean>(
    "hosts",
    "show-hint-empty-configs",
    startConfigurations.length === 0,
    {
      version: 1,
      migrateFrom: {
        localStorageKey: "Provider:openHintDialog-start-configs",
      },
    }
  );

  const addButtonRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    setShowStartConfigurations(
      startConfigurations.map((cfg) => {
        // fix deprecated networkId parameter
        if (!cfg.domainId && cfg.networkId) {
          cfg.domainId = cfg.networkId;
        }
        if (!cfg.rmw.zenoh) {
          // fix for deprecated configuration parameter
          const rmw = cfg.rmw as unknown as {
            overrideZenoEnv: ZenohEnvSelection;
            remoteZenohHost: string;
            startZenohDaemon: boolean;
          };
          cfg.rmw.zenoh = {
            overrideEnv: rmw.overrideZenoEnv,
            remoteHosts: rmw.remoteZenohHost ? [rmw.remoteZenohHost] : [],
            startDaemon: rmw.startZenohDaemon,
          };
        }
        if (!cfg.rmw.zenoh.remoteHosts) {
          const oldZenoh = cfg.rmw.zenoh as unknown as {
            remoteHost: string;
          };
          cfg.rmw.zenoh.remoteHosts = oldZenoh.remoteHost ? [oldZenoh.remoteHost] : [];
        }
        if (!cfg.rmw.fastrtps) {
          cfg.rmw.fastrtps = {
            overrideEnv: "",
          };
        }
        if (!cfg.rmw.connext) {
          cfg.rmw.connext = {
            overrideEnv: "",
          };
        }
        if (!cfg.rmw.cyclone) {
          cfg.rmw.cyclone = {
            overrideEnv: "env",
            maxParticipants: "100",
            allowMulticast: "spdp",
          };
        }
        return new ProviderLaunchConfiguration(cfg);
      })
    );
  }, [startConfigurations]);

  const handleAutostart = useCallback(async (): Promise<void> => {
    if (rosCtx.providers.length !== 0) return;
    const doStart = cliCtx.getArgument("start") || false;
    const doJoin = cliCtx.getArgument("join") || false;
    // Headless electron window not needed unless --start specified
    if (isElectron() && !!cliCtx.getArgument("headless") && !doStart) return;
    const doJoinWs = (cliCtx.getArgument("join-ws") as string).split(",");
    if (doStart || doJoin) {
      const rosDomainId = Number.parseInt(`${cliCtx.getArgument("ros-domain-id") || rosCtx.rosInfo?.domainId}`);
      const rosVersion = (cliCtx.getArgument("ros-version") as string) || rosCtx.rosInfo?.version;
      let rmwImplementation: string | undefined = cliCtx.getArgument("rmw-implementation") as string;
      if (!rmwImplementation || rmwImplementation === "RMW_IMPLEMENTATION") {
        rmwImplementation = rosCtx.rosInfo?.rmwImplementation;
      }
      const hostsStr = cliCtx.getArgument("host") as string;
      const hosts = hostsStr ? hostsStr.split(",") : ["localhost"];
      if (!rosDomainId || rosDomainId < 0) {
        console.warn(`can't join: unknown ROS_DOMAIN_ID; use ros-domain-id to set domain id`);
        setNoDomainId(true);
        return;
      }
      if (!rosVersion) {
        console.warn(`can't join to ${rosDomainId}: unknown ROS_VERSION; use --ros-version to set ros version`);
        setNoRosVersion(true);
        return;
      }
      for (const host of hosts) {
        const config = new ProviderLaunchConfiguration();
        config.params.host = host;
        config.params.domainId = rosDomainId;
        config.params.rosVersion = rosVersion;
        if (
          rmwImplementation &&
          ["rmw_connextdds", "rmw_cyclonedds_cpp", "rmw_fastrtps_cpp", "rmw_zenoh_cpp"].includes(rmwImplementation)
        ) {
          config.params.rmw.current = rmwImplementation as RmwSelection;
          config.params.rmw.selected = rmwImplementation as RmwSelection;
        }
        if (doStart) {
          config.params.autostart = true;
          rosCtx.startConfig(config, null);
        } else if (doJoin) {
          console.log(`Connect to ${config.params.host}:${config.params.port}`);
          rosCtx.connect(config.params, false, "added by --join");
        }
      }
      return;
    }

    for (const ws of doJoinWs) {
      const [host, portStr] = ws.split(":");
      const port = Number.parseInt(portStr);
      if (host && port) {
        const config = new ProviderLaunchConfiguration();
        config.params.host = host;
        config.params.port = port;
        rosCtx.connect(config.params, false, "added by --join-ws");
      }
    }

    for (const startCfg of startConfigurations) {
      if (startCfg.autoConnect) {
        rosCtx.connect(startCfg, true, "Configuration autostart");
      }
    }

    if (startConfigurations.length === 0) {
      addButtonRef?.current?.focus();
    }
    // try to get local domain id from running mas processes
    if (rosCtx.rosInfo?.version) {
      try {
        const result = await window.commandExecutor?.exec(
          null, // we start the subscriber always local
          "ps aux | grep ros.fkie/screens/ | grep mas-daemon"
        );
        if (result?.result) {
          const lines = result.message.split("\n");
          let domainId = -1;
          for (const line of lines) {
            if (!line.includes("grep") && line.includes("ros.fkie/screens/") && line.includes("mas-daemon")) {
              const match = line.match(/screen\.cfg/);
              if (match) {
                domainId = 0;
              } else {
                const match = line.match(/screen_(\d+)\.cfg/);
                if (match?.[1]) {
                  domainId = Number.parseInt(match[1], 10);
                } else {
                  domainId = Number.parseInt(rosCtx.rosInfo?.domainId || "0");
                }
              }
              console.log(`found running mas-daemon with domain id ${domainId}`);
              if (domainId >= 0) {
                const newProvId = generateProviderId("localhost", 0, rosCtx.rosInfo.version, domainId);
                if (!rosCtx.getProviderById(newProvId)) {
                  const newProvider = rosCtx.createProvider(
                    "localhost",
                    rosCtx.rosInfo.version,
                    undefined,
                    domainId,
                    undefined
                  );
                  newProvider.triggeredByAutoConnect = true;
                  newProvider.additionInfo = "Discovered from process";
                  rosCtx.connectToProvider(newProvider);
                }
              }
            }
          }
        }
      } catch (error) {
        console.log(`error while lookup for running daemons: ${error} `);
      }

      if (window.commandExecutor && !(rosCtx.rosInfo?.version || cliCtx.getArgument("ros-version"))) {
        setNoSourcedROS(true);
      }
    }
  }, [
    rosCtx.providers,
    rosCtx.connect,
    rosCtx.connectToProvider,
    rosCtx.getProviderById,
    rosCtx.createProvider,
    rosCtx.rosInfo,
    cliCtx,
    startConfigurations,
  ]);

  const debouncedCallbackFilterText = useDebounceCallback((providers: Provider[], searchTerm: string) => {
    if (searchTerm.length > 1) {
      const re = new RegExp(searchTerm, "i");
      setProviderRowsFiltered(
        providers.filter((provider) => {
          const pos = provider.name().search(re);
          return pos !== -1;
        })
      );
      setShowStartConfigurations(
        startConfigurations
          .filter((item) => item.host.search(re) !== -1)
          .map((cfg) => {
            return new ProviderLaunchConfiguration(cfg);
          })
      );
    } else {
      setProviderRowsFiltered(providers);
      setShowStartConfigurations(
        startConfigurations.map((cfg) => {
          return new ProviderLaunchConfiguration(cfg);
        })
      );
    }
  }, 300);

  const editLaunchConfiguration = useCallback((config: ProviderLaunchConfiguration, title?: string) => {
    emitOpenComponent({
      id: config.params.id,
      title: title || `${config.params.host} start configuration`,
      closable: true,
      component: LAYOUT_TABS.PROVIDER_LAUNCH_CONTROL,
      toNodeId: LAYOUT_TAB_SETS.CENTER,
      config: {
        providerLaunchConfig: {
          id: config.params.id,
          config,
        },
      },
    });
  }, []);

  useCustomEventListener(EVENT_PROVIDER_STATE, () => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText]);

  useEffect(() => {
    if (!window.commandLine && cliCtx.updatedArgs === 0 && rosCtx.rosInfo) return;
    handleAutostart();
  }, [cliCtx.updatedArgs, window.commandLine, rosCtx.rosInfo]);

  useEffect(() => {
    // hide hint dialog if join or start argument was provided
    if (cliCtx.getArgument("join") || cliCtx.getArgument("start") || cliCtx.getArgument("join-ws")) {
      setOpenHintDialog(false);
    }
  }, [cliCtx.updatedArgs]);

  const createReloadButton = useMemo(() => {
    return (
      <Stack direction="row">
        <Tooltip title="Refresh hosts list" placement="bottom" disableInteractive>
          <IconButton
            edge="start"
            aria-label="refresh hosts list"
            onClick={() => rosCtx.refreshProviderList(startConfigurations)}
          >
            <RefreshIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>
      </Stack>
    );
  }, [startConfigurations]);

  const createProviderTable = useMemo(() => {
    const result = (
      // <TableContainer  height="100%" style={{ overflowX: 'scroll', flexGrow: 1 }}>
      <TableContainer style={{ flexGrow: 1 }}>
        <Table aria-label="hosts table" sx={{ display: "block" }}>
          <TableBody sx={{ display: "block" }}>
            {providerRowsFiltered.map((provider) => {
              return <ProviderPanelRow key={provider.id} provider={provider} />;
            })}
          </TableBody>
        </Table>
      </TableContainer>
    );
    return result;
  }, [providerRowsFiltered, rosCtx]);

  const createProviderStartTable = useMemo(() => {
    const result = (
      <TableContainer>
        <Table aria-label="configs table" sx={{ display: "block" }}>
          <TableBody sx={{ display: "block" }}>
            {showStartConfigurations.map((config) => {
              return (
                <ProviderPanelRowCfg
                  key={config.params.id}
                  startConfig={config}
                  editConfiguration={editLaunchConfiguration}
                />
              );
            })}
          </TableBody>
        </Table>
      </TableContainer>
    );
    return result;
  }, [showStartConfigurations, rosCtx]);

  return (
    <Stack
      spacing={1}
      height="100%"
      // width="100%"
      // overflow="auto"
      style={{ backgroundColor: backgroundColor }}
    >
      <Stack direction="row" spacing={0.5} alignItems="center">
        {buttonLocation === BUTTON_LOCATIONS.LEFT && createReloadButton}
        <SearchBar
          onSearch={(value) => {
            setFilterText(value);
          }}
          placeholder="Filter hosts"
          defaultValue={filterText}
          // fullWidth={true}
        />
        {buttonLocation === BUTTON_LOCATIONS.RIGHT && createReloadButton}
        {noSourcedROS && (
          <ConfirmModal
            title="Is ROS sourced?"
            message="The ROS version could not be determined. This indicates that setup.bash was not sourced. Please restart mas-gui after sourcing!"
            onConfirmCallback={() => {
              setNoSourcedROS(false);
            }}
            showCancelButton={false}
          />
        )}
        {noDomainId && (
          <ConfirmModal
            title={"no valid ROS domain id found"}
            message={"--start and --join require valid ROS_DOMAIN_ID. Use --ros-domain-id to set"}
            onConfirmCallback={() => {
              setNoRosVersion(false);
            }}
            showCancelButton={false}
          />
        )}
        {noRosVersion && (
          <ConfirmModal
            title={"no ROS version found"}
            message={"--start and --join require valid ROS_VERSION. Use --ros-version to set ros version"}
            onConfirmCallback={() => {
              setNoRosVersion(false);
            }}
            showCancelButton={false}
          />
        )}
      </Stack>
      <AccordionAdv
        // disabled={!window.commandExecutor}
        expanded={expandedProviderCfg}
        onChange={(_event, expanded) => {
          setExpandedProviderCfg(expanded);
        }}
        sx={{ pl: 0, padding: 0 }}
      >
        <Stack direction="row" alignItems="center" spacing="0.3em" flexGrow={1} minWidth={0}>
          <AccordionSummary
            // disabled={!startSystemNodes}
            expandIcon={<ExpandMoreIcon />}
            aria-controls="start-commands"
            id="start-commands"
            sx={{
              pl: 0,
              paddingBottom: 0,
              flexShrink: 1,
              minWidth: 0,
              "& .MuiAccordionSummary-content": {
                minWidth: 0,
                overflow: "hidden",
              },
            }}
          >
            <Stack direction="row" alignItems="center" spacing="0.3em" flexGrow={1} minWidth={0}>
              <SettingsOutlinedIcon fontSize="inherit" />
              <Typography
                variant="subtitle1"
                sx={{ flexShrink: 1, minWidth: 0, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}
              >
                {window.commandExecutor ? "Start" : "Join"} Configurations - {startConfigurations.length}
              </Typography>
            </Stack>
          </AccordionSummary>
          <Tooltip title={`Add new ${window.commandExecutor ? "start" : "join"} configuration`} disableInteractive>
            <IconButton
              component="span"
              ref={addButtonRef}
              onClick={(event) => {
                event.stopPropagation();
                const launchCfg = new ProviderLaunchConfiguration();
                editLaunchConfiguration(launchCfg, `New ${window.commandExecutor ? "start" : "join"} configuration`);
              }}
              onTouchEnd={(event) => {
                event.stopPropagation();
              }}
            >
              <AddIcon />
            </IconButton>
          </Tooltip>
        </Stack>
        <AccordionDetails sx={{ paddingBottom: 1 }}>
          <Stack
            direction="column"
            // divider={<Divider orientation="horizontal" />}
          >
            {createProviderStartTable}
          </Stack>
        </AccordionDetails>
      </AccordionAdv>
      <Stack flexGrow={1}>{createProviderTable}</Stack>
      <Dialog
        key="start-cfg-hint-dialog"
        open={openHintDialog}
        onClose={() => setOpenHintDialog(false)}
        fullWidth
        scroll="paper"
        maxWidth="sm"
        PaperComponent={DraggablePaper}
        aria-labelledby="draggable-dialog-title"
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
          Create start configuration
        </DialogTitle>

        <DialogContent dividers={true} aria-label="info">
          <Typography variant="body2">
            Create a startup configuration for each host that is used to start the MAS nodes. This allows the ROS system
            to be monitored and controlled.
          </Typography>
        </DialogContent>

        <DialogActions>
          <Button color="primary" onClick={() => setOpenHintDialog(false)}>
            Cancel
          </Button>

          <Button
            autoFocus
            color="success"
            onClick={() => {
              const launchCfg = new ProviderLaunchConfiguration();
              editLaunchConfiguration(launchCfg, "New start configuration");
              setOpenHintDialog(false);
            }}
          >
            Create
          </Button>
        </DialogActions>
      </Dialog>
    </Stack>
  );
}
