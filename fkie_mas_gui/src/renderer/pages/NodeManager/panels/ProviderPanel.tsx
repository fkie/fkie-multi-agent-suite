import RefreshIcon from "@mui/icons-material/Refresh";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import { IconButton, Stack, Table, TableBody, TableContainer, Tooltip } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { useContext, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import ConnectToProviderModal from "@/renderer/components/ConnectToProviderModal/ConnectToProviderModal";
import ConfirmModal from "@/renderer/components/SelectionModal/ConfirmModal";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { RosContext } from "@/renderer/context/RosContext";
import { BUTTON_LOCATIONS, SettingsContext } from "@/renderer/context/SettingsContext";
import { EVENT_PROVIDER_STATE } from "@/renderer/providers/eventTypes";
import Provider from "@/renderer/providers/Provider";
import { EVENT_OPEN_CONNECT } from "../layout/events";
import ProviderPanelRow from "./ProviderPanelRow";

export default function ProviderPanel(): JSX.Element {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [openConnect, setOpenConnect] = useState(false);
  const [noSourcedROS, setNoSourcedROS] = useState(false);
  const [noRosVersion, setNoRosVersion] = useState(false);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState<Provider[]>([]);
  const [filterText, setFilterText] = useState("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [buttonLocation, setButtonLocation] = useState<string>(settingsCtx.get("buttonLocation") as string);
  const [joinArg, setJoinArg] = useState<{ host: string; id: number }>({ host: "", id: -1 });

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setButtonLocation(settingsCtx.get("buttonLocation") as string);
  }, [settingsCtx.changed]);

  useCustomEventListener(EVENT_OPEN_CONNECT, () => {
    setOpenConnect(true);
  });

  async function getDomainId(): Promise<void> {
    if (rosCtx.providers.length === 0) {
      if (settingsCtx.hasArgument("start") && window.commandExecutor) {
        setOpenConnect(true);
        return;
      }
      // do we have a join environment parameter
      if (joinArg.id >= 0) {
        if (!rosCtx.rosInfo?.version && !settingsCtx.getArgument("ros-version")) {
          console.warn(
            `can't join to ${joinArg.host ? joinArg.host : "localhost"}:${joinArg.id}: unknown ROS_VERSION; use --ros-version to set ros version`
          );
          setNoRosVersion(true);
          return;
        }
        const domainId = joinArg.id;
        if (domainId >= 0) {
          const newProvider = new Provider(
            settingsCtx,
            "localhost",
            rosCtx.rosInfo?.version || settingsCtx.getArgument("ros-version"),
            undefined,
            domainId,
            undefined,
            logCtx
          );
          await rosCtx.connectToProvider(newProvider);
          return;
        }
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
              const newProvider = new Provider(
                settingsCtx,
                "localhost",
                rosCtx.rosInfo.version,
                undefined,
                domainId,
                undefined,
                logCtx
              );
              await rosCtx.connectToProvider(newProvider);
              return;
            }
          }
        } catch (error) {
          console.log(`error while lookup for running daemons: ${error} `);
        }
      }
      if (!window.commandExecutor) {
        setOpenConnect(true);
      } else if (rosCtx.rosInfo?.version || joinArg.id >= 0) {
        setOpenConnect(true);
      } else {
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
    } else {
      setProviderRowsFiltered(providers);
    }
  }, 300);

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  useCustomEventListener(EVENT_PROVIDER_STATE, () => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText]);

  useEffect(() => {
    if (window.commandExecutor && !rosCtx.rosInfo) return;
    getDomainId();
  }, [rosCtx.rosInfo, window.commandExecutor, joinArg]);

  useEffect(() => {
    // join can be localhost:1 or 1
    const splits = settingsCtx.getArgument("join").split(":");
    if (splits.length === 1) {
      const id = Number.parseInt(splits[0]);
      if (id >= 0) {
        setJoinArg({ host: "", id: id });
      }
    } else if (splits.length === 2) {
      const id = Number.parseInt(splits[1]);
      if (id >= 0) {
        setJoinArg({ host: splits[0], id: id });
      }
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
        <Tooltip title="Start system nodes" placement="bottom" disableInteractive>
          <IconButton
            color="primary"
            onClick={() => {
              setOpenConnect(true);
            }}
            size="small"
          >
            <RocketLaunchIcon fontSize="inherit" />
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
        {openConnect && (
          <ConnectToProviderModal
            defaultHost={settingsCtx.hasArgument("start") ? joinArg.host : undefined}
            defaultRosDomainId={settingsCtx.hasArgument("start") ? joinArg.id : undefined}
            startOnOpen={settingsCtx.hasArgument("start")}
            onCloseDialog={() => {
              setOpenConnect(false);
            }}
          />
        )}
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
            title={`can't join to ROS domain id ${joinArg.host ? joinArg.host : "localhost"}:${joinArg.id}`}
            message={`--join is set to ${joinArg.host ? joinArg.host : ""}:${joinArg.id} but ROS_VERSION is unknown. Use --ros-version to set ros version`}
            onConfirmCallback={() => {
              setNoRosVersion(false);
            }}
            showCancelButton={false}
          />
        )}
      </Stack>
      <Stack height="100%">{createProviderTable}</Stack>
    </Stack>
  );
}
