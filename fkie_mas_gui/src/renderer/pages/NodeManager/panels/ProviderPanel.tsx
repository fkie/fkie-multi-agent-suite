import AddIcon from "@mui/icons-material/Add";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import RefreshIcon from "@mui/icons-material/Refresh";
import SettingsOutlinedIcon from "@mui/icons-material/SettingsOutlined";
import { IconButton, Stack, Table, TableBody, TableContainer, Tooltip, Typography } from "@mui/material";
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary from "@mui/material/AccordionSummary";
import { styled } from "@mui/material/styles";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { ProviderLaunchConfiguration } from "@/renderer/models";
import { TProviderLaunchParams } from "@/renderer/models/ProviderLaunchConfiguration";
import { EVENT_PROVIDER_STATE } from "@/renderer/providers/eventTypes";
import Provider from "@/renderer/providers/Provider";
import { LAYOUT_TAB_SETS } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import ProviderLaunchConfigPanel from "./ProviderLaunchConfigPanel";
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
  const settingsCtx = useSettingsContext();
  const [expandedProviderCfg, setExpandedProviderCfg] = useState(true);
  const [noSourcedROS, setNoSourcedROS] = useState(false);
  const [noRosVersion, setNoRosVersion] = useState(false);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState<Provider[]>([]);
  const [filterText, setFilterText] = useState("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [buttonLocation, setButtonLocation] = useState<string>(settingsCtx.get("buttonLocation") as string);
  const [startConfigurations, setStartConfigurations] = useLocalStorage<TProviderLaunchParams[]>(
    "Provider:startConfigurations",
    []
  );
  const [showStartConfigurations, setShowStartConfigurations] = useState<ProviderLaunchConfiguration[]>([]);

  const addButtonRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setButtonLocation(settingsCtx.get("buttonLocation") as string);
  }, [settingsCtx.changed]);

  useEffect(() => {
    setShowStartConfigurations(
      startConfigurations.map((cfg) => {
        return new ProviderLaunchConfiguration(cfg);
      })
    );
  }, [startConfigurations]);

  async function getDomainId(): Promise<void> {
    if (rosCtx.providers.length === 0) {
      if (settingsCtx.getArgument("start") && window.commandExecutor) {
        return;
      }
      const rosDomainId = settingsCtx.getArgument("ros-domain-id") as number;
      // do we have a join environment parameter
      if (settingsCtx.getArgument("join") && rosDomainId >= 0) {
        if (!rosCtx.rosInfo?.version && !settingsCtx.getArgument("ros-version")) {
          console.warn(`can't join to ${rosDomainId}: unknown ROS_VERSION; use --ros-version to set ros version`);
          setNoRosVersion(true);
          return;
        }
        return;
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
              }
            }
            if (domainId >= 0) {
              console.log("CONNECT TO PROVIDER");
              const newProvider = rosCtx.createProvider(
                "localhost",
                rosCtx.rosInfo.version,
                undefined,
                domainId,
                undefined
              );
              await rosCtx.connectToProvider(newProvider);
              return;
            }
          }
        } catch (error) {
          console.log(`error while lookup for running daemons: ${error} `);
        }
      }
      console.log("HIDDEN CONNECT");
      for (const startCfg of startConfigurations) {
        if (startCfg.host === "localhost") {
          rosCtx.hiddenConnect(startCfg);
        }
      }

      if (startConfigurations.length === 0) {
        addButtonRef?.current?.focus();
      }
      if (window.commandExecutor && !(rosCtx.rosInfo?.version || settingsCtx.getArgument("ros-version"))) {
        setNoSourcedROS(true);
      }
    }
  }

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

  const saveLaunchConfiguration = useCallback(
    (config: ProviderLaunchConfiguration) => {
      setStartConfigurations((prev) => {
        const idx = prev.findIndex((c) => c.id === config.params.id);
        const next = [...prev];

        if (idx !== -1) {
          // replace
          next[idx] = config.params;
        } else {
          // add new
          next.push(config.params);
        }

        // sort
        return next.sort((a, b) => a.host.localeCompare(b.host));
      });
    },
    [setStartConfigurations]
  );

  const deleteLaunchConfiguration = useCallback(
    (configId: string) => {
      setStartConfigurations((prev) => {
        return prev.filter((c) => c.id !== configId);
      });
    },
    [setStartConfigurations]
  );

  const editLaunchConfiguration = useCallback((config: ProviderLaunchConfiguration, title?: string) => {
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        config.params.id,
        title || `${config.params.host} start configuration`,
        <ProviderLaunchConfigPanel
          config={config}
          onDelete={(configId) => {
            deleteLaunchConfiguration(configId);
          }}
          onSave={(config) => {
            saveLaunchConfiguration(config);
          }}
        />,
        true,
        LAYOUT_TAB_SETS.CENTER,
        undefined
      )
    );
  }, []);

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  useCustomEventListener(EVENT_PROVIDER_STATE, () => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText]);

  useEffect(() => {
    if (settingsCtx.updatedArgs > 0 || !window.commandLine) {
      getDomainId();
    }
  }, [settingsCtx.updatedArgs]);

  const createReloadButton = useMemo(() => {
    return (
      <Stack direction="row">
        <Tooltip
          title="Refresh hosts list"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <IconButton edge="start" aria-label="refresh hosts list" onClick={() => rosCtx.refreshProviderList()}>
            <RefreshIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>
      </Stack>
    );
  }, [tooltipDelay]);

  const createProviderTable = useMemo(() => {
    const result = (
      // <TableContainer  height="100%" style={{ overflowX: 'scroll', flexGrow: 1 }}>
      <TableContainer style={{ flexGrow: 1 }}>
        <Table aria-label="hosts table">
          <TableBody>
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
      // <TableContainer  height="100%" style={{ overflowX: 'scroll', flexGrow: 1 }}>
      <TableContainer style={{ flexGrow: 1 }}>
        <Table aria-label="configs table">
          <TableBody>
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
        {noRosVersion && (
          <ConfirmModal
            title={`can't join to ROS domain id ${settingsCtx.getArgument("ros-domain-id")}`}
            message={`--join is set to ${settingsCtx.getArgument("ros-domain-id")} but ROS_VERSION is unknown. Use --ros-version to set ros version`}
            onConfirmCallback={() => {
              setNoRosVersion(false);
            }}
            showCancelButton={false}
          />
        )}
      </Stack>
      <AccordionAdv
        disabled={!window.commandExecutor}
        expanded={expandedProviderCfg}
        onChange={(_event, expanded) => {
          setExpandedProviderCfg(expanded);
        }}
        sx={{ pl: 0, padding: 0 }}
      >
        <Stack direction="row" alignItems="center" spacing="0.3em" flexGrow={1}>
          <AccordionSummary
            // disabled={!startSystemNodes}
            expandIcon={<ExpandMoreIcon />}
            aria-controls="start-commands"
            id="start-commands"
            sx={{ pl: 0, paddingBottom: 0 }}
          >
            <Stack direction="row" alignItems="center" spacing="0.3em" flexGrow={1}>
              <SettingsOutlinedIcon fontSize="inherit" />
              <Typography variant="subtitle1" flexGrow={1}>
                Provider Configurations - {startConfigurations.length}
              </Typography>
            </Stack>
          </AccordionSummary>
          <Tooltip title="Add new start configuration" disableInteractive>
            <IconButton
              component="span"
              ref={addButtonRef}
              onClick={(event) => {
                event.stopPropagation();
                const launchCfg = new ProviderLaunchConfiguration();
                editLaunchConfiguration(launchCfg, "New start configuration");
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
    </Stack>
  );
}
