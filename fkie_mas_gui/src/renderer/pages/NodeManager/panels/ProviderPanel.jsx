import RefreshIcon from "@mui/icons-material/Refresh";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import { IconButton, Stack, Table, TableBody, TableContainer, Tooltip } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { useContext, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { ConnectToProviderModal, SearchBar } from "../../../components";
import { LoggingContext } from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { EVENT_PROVIDER_STATE } from "../../../providers/eventTypes";
import Provider from "../../../providers/Provider";
import { EVENT_OPEN_CONNECT } from "../layout/events";
import ProviderPanelRow from "./ProviderPanelRow";

function ProviderPanel() {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [openConnect, setOpenConnect] = useState(false);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState([]);
  const [filterText, setFilterText] = useState("");
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  useCustomEventListener(EVENT_OPEN_CONNECT, () => {
    setOpenConnect(true);
  });

  const getDomainId = async () => {
    if (rosCtx.providers.length === 0) {
      // do we have a join environment parameter
      if (import.meta.env.VITE_JOIN_ID) {
        if (!rosCtx.rosInfo?.version && !import.meta.env.VITE_ROS_VERSION) {
          console.warn(
            `can't join to ${import.meta.env.VITE_JOIN_ID}: unknown ROS_VERSION; use VITE_ROS_VERSION to set ros version`
          );
          return;
        }
        const domainId = parseInt(import.meta.env.VITE_JOIN_ID);
        if (domainId >= 0) {
          const newProvider = new Provider(
            settingsCtx,
            "localhost",
            rosCtx.rosInfo?.version || import.meta.env.VITE_ROS_VERSION,
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
            lines.forEach((line) => {
              if (!line.includes("grep") && line.includes("ros.fkie/screens/") && line.includes("mas-daemon")) {
                const match = line.match(/screen_(\d+)\.cfg/);
                if (match && match[1]) {
                  domainId = parseInt(match[1], 10);
                } else {
                  domainId = 0;
                }
              }
            });
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
      if (!window.commandExecutor || rosCtx.rosInfo?.version) {
        setOpenConnect(true);
      }
    }
  };

  const debouncedCallbackFilterText = useDebounceCallback((providers, searchTerm) => {
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
  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText, debouncedCallbackFilterText]);

  useEffect(() => {
    if (window.commandExecutor && !rosCtx.rosInfo) return;
    getDomainId();
  }, [rosCtx.rosInfo, rosCtx.providers]);

  const createProviderTable = useMemo(() => {
    const result = (
      // <TableContainer  height="100%" style={{ overflowX: 'scroll', flexGrow: 1 }}>
      <TableContainer height="100%" style={{ flexGrow: 1 }}>
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
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [providerRowsFiltered, rosCtx]);

  return (
    <Stack
      spacing={1}
      height="100%"
      // width="100%"
      // overflow="auto"
      backgroundColor={settingsCtx.get("backgroundColor")}
    >
      <Stack direction="row" spacing={0.5}>
        <SearchBar
          onSearch={(value) => {
            setFilterText(value);
          }}
          placeholder="Filter hosts"
          defaultValue={filterText}
          // fullWidth={true}
        />
        {openConnect && (
          <ConnectToProviderModal
            onCloseDialog={() => {
              setOpenConnect(false);
            }}
          />
        )}
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
      <Stack height="100%">{createProviderTable}</Stack>
    </Stack>
  );
}

ProviderPanel.propTypes = {};

export default ProviderPanel;
