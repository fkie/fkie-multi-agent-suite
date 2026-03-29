import CheckIcon from "@mui/icons-material/Check";
import HighlightOffIcon from "@mui/icons-material/HighlightOff";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import TextSnippetOutlinedIcon from "@mui/icons-material/TextSnippetOutlined";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import {
  Button,
  CircularProgress,
  IconButton,
  Link,
  Stack,
  TableCell,
  TableRow,
  Tooltip,
  Typography,
} from "@mui/material";

import { useCallback, useEffect, useMemo, useReducer, useState } from "react";

import { colorFromHostname } from "@/renderer/components/UI/Colors";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { ProviderLaunchConfiguration, RosNode } from "@/renderer/models";
import { ConnectionState, Provider } from "@/renderer/providers";
import { EMenuProvider } from "./OverflowMenuProvider";

interface ProviderPanelRowCfgProps {
  startConfig: ProviderLaunchConfiguration;
}

export default function ProviderPanelRowCfg(props: ProviderPanelRowCfgProps): JSX.Element {
  const { startConfig } = props;
  const rosCtx = useRosContext();
  const navCtx = useNavigationContext();
  const settingsCtx = useSettingsContext();
  const [updated, forceUpdate] = useReducer((x) => x + 1, 0);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx.changed]);

  const closeProviderHandler = useCallback(
    async (providerId: string) => {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        provider.close();
      }
    },
    [rosCtx]
  );

  async function showDaemonLog(provider: Provider): Promise<void> {
    await window.commandExecutor?.execTerminal(
      provider.isLocalHost ? null : { host: provider.host() },
      "'show daemon log'",
      "'ros2 run fkie_mas_daemon mas-remote-node.py --show_ros_log /mas/_daemon_{HOST}'"
    );
  }

  async function handleJoinProvider(provider: Provider): Promise<void> {
    await rosCtx.connectToProvider(provider);
  }

  const handleStartProvider = useCallback(() => {
    rosCtx.startConfig(startConfig, null);
  }, [startConfig]);

  async function onProviderMenuClick(actionType: EMenuProvider, provider: Provider): Promise<void> {
    if (actionType === EMenuProvider.INFO) {
      const nodes = Array.from(rosCtx.nodeMap)
        .filter((value: [string, RosNode]) => {
          return value[0].startsWith(provider.id);
        })
        .map((value) => {
          return value[0];
        });
      navCtx.setSelectedProviders([provider.id], true);
      navCtx.setSelectedNodes(nodes, false);
      // emitCustomEvent(
      //   EVENT_OPEN_COMPONENT,
      //   eventOpenComponent(
      //     `provider-info-${providerName}`,
      //     providerName,
      //     <SystemInformationPanel providerId={providerId} />,
      //     true,
      //     LAYOUT_TABS.HOSTS,
      //     new LayoutTabConfig(false, "info")
      //   )
      // );
      return;
    }
    if (actionType === EMenuProvider.DELETE) {
      // rosCtx.removeProvider(providerId);
    }
  }

  function generateStatusView(provider: Provider): JSX.Element {
    switch (provider.connectionState) {
      case ConnectionState.STATES.SERVER_CONNECTED:
      case ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED:
      case ConnectionState.STATES.CONNECTING:
        setTimeout(() => {
          forceUpdate();
        }, 3000);
        return (
          <Stack direction="row" alignItems="center" spacing="0.5em" paddingRight="0.5em">
            <Tooltip title="Connecting" placement="bottom" disableInteractive>
              <span style={{ color: "blue" }}>connecting</span>
            </Tooltip>

            <CircularProgress size="1em" />
            <Tooltip title="cancel" placement="bottom" disableInteractive>
              <IconButton
                onClick={() => {
                  closeProviderHandler(provider.id);
                }}
                size="small"
              >
                <HighlightOffIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        );
      case ConnectionState.STATES.STARTING:
        setTimeout(() => {
          forceUpdate();
        }, 3000);
        return (
          <Stack direction="row" alignItems="center" spacing="0.5em" paddingRight="0.5em">
            <div style={{ color: "blue" }}>{provider.connectionState}</div>
            <CircularProgress size="1em" />
          </Stack>
        );
      case ConnectionState.STATES.CONNECTED:
        return (
          <Stack direction="row" alignItems="center">
            {/* <div style={{ color: 'green' }}>{provider.connectionState}</div> */}
            <CheckIcon style={{ color: "green", fontSize: "0.6em" }} />

            <Tooltip title="Disconnect" placement="bottom" disableInteractive>
              <IconButton
                onClick={() => {
                  closeProviderHandler(provider.id);
                }}
                size="small"
              >
                <HighlightOffIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        );
      case ConnectionState.STATES.AUTHZ:
        return (
          <Stack direction="row" alignItems="center">
            <Tooltip title={`Can't access remote host! Wrong SSH credentials?`} placement="bottom" disableInteractive>
              <Button
                style={{ textTransform: "none" }}
                onClick={() => {
                  // TODO: add information panel how to configure SSH
                }}
                variant="text"
                color="error"
                size="small"
              >
                <Typography noWrap variant="body2">
                  {provider.connectionState}
                </Typography>
              </Button>
            </Tooltip>
            <Tooltip title="Start daemon" placement="bottom" disableInteractive>
              <IconButton
                color="default"
                onClick={() => {
                  handleStartProvider();
                }}
              >
                <PlayCircleOutlineIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        );
      case ConnectionState.STATES.LOST:
      case ConnectionState.STATES.UNSUPPORTED:
      case ConnectionState.STATES.UNREACHABLE:
      case ConnectionState.STATES.ERRORED: {
        // eslint-disable-next-line no-case-declarations
        let state = provider.connectionState;
        if (provider.connectionState === ConnectionState.STATES.ERRORED) {
          if (!provider.daemon) {
            state = "no daemon";
          } else if (!provider.discovery) {
            state = "no discovery";
          }
        }
        return (
          <Stack direction="row" alignItems="center" justifyContent="center">
            <Tooltip
              title={`Click to start provider! ${
                provider.errorDetails ? `${state}: ${JSON.stringify(provider.errorDetails)}` : ""
              }`}
              placement="bottom"
              disableInteractive
            >
              <span>
                {window.commandExecutor && (
                  <Button
                    style={{
                      textTransform: "none",
                    }}
                    onClick={() => {
                      handleStartProvider();
                    }}
                    variant="text"
                    color="info"
                    size="small"
                    endIcon={<PlayCircleOutlineIcon fontSize="inherit" />}
                  >
                    <div style={{ color: "red", whiteSpace: "nowrap" }}>{state}</div>
                  </Button>
                )}
                {!window.commandExecutor && <div style={{ color: "red", whiteSpace: "nowrap" }}>{state}</div>}
              </span>
            </Tooltip>

            {window.commandExecutor && rosCtx.rosInfo?.version === "2" && (
              <Tooltip title="Show daemon log" placement="bottom" disableInteractive>
                <IconButton
                  color="default"
                  onClick={() => {
                    showDaemonLog(provider);
                  }}
                >
                  <TextSnippetOutlinedIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )}

            <Tooltip title="Join to running daemon" placement="bottom" disableInteractive>
              <IconButton
                color="default"
                onClick={() => {
                  handleJoinProvider(provider);
                }}
              >
                <JoinFullIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        );
      }
      default:
        return (
          <Stack direction="row" alignItems="center">
            <div style={{ color: "grey" }}>{provider.connectionState}</div>
            {window.commandExecutor && rosCtx.rosInfo?.version === "2" && (
              <Tooltip title="Show daemon log" placement="bottom" disableInteractive>
                <IconButton
                  color="default"
                  onClick={() => {
                    showDaemonLog(provider);
                  }}
                >
                  <TextSnippetOutlinedIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )}
            {window.commandExecutor && (
              <Tooltip title="Start daemon" placement="bottom" disableInteractive>
                <IconButton
                  color="default"
                  onClick={() => {
                    handleStartProvider();
                  }}
                >
                  <PlayCircleOutlineIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )}
            <Tooltip title="Join to running daemon" placement="bottom" disableInteractive>
              <IconButton
                color="default"
                onClick={() => {
                  handleJoinProvider(provider);
                }}
              >
                <JoinFullIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        );
    }
  }

  function generateWarningsView(provider: Provider): JSX.Element {
    if (!provider.warnings) return <></>;
    const warnings = provider.warnings.filter((group) => (group.warnings || []).length > 0);
    if (warnings.length > 0) {
      return (
        <Tooltip
          title={`Provider reports warning for ${warnings.map((item) => item.id)}`}
          placement="bottom"
          disableInteractive
        >
          <IconButton
            color="default"
            onClick={() => {
              onProviderMenuClick(EMenuProvider.INFO, provider);
            }}
          >
            <WarningAmberIcon color="warning" fontSize="inherit" />
          </IconButton>
        </Tooltip>
      );
    }
    // eslint-disable-next-line react/jsx-no-useless-fragment
    return <></>;
  }

  const getHostStyle = useCallback(
    (name: string) => {
      if (settingsCtx.get("colorizeHosts")) {
        // borderLeft: `3px dashed`,
        // borderColor: colorFromHostname(provider.name()),
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(name),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  const createTableRow = useMemo(() => {
    return (
      <TableRow
        key={startConfig.id}
        style={{
          display: "block",
          padding: 0,
        }}
      >
        <TableCell
          style={{
            padding: 2,
            flexGrow: 1,
            width: "100%",
          }}
          sx={getHostStyle(startConfig.host)}
        >
          <Stack direction="row" spacing="0.5em">
            <Link
              noWrap
              href="#"
              underline="none"
              color="inherit"
              onClick={() => {
                // onProviderMenuClick(EMenuProvider.INFO, provider);
              }}
            >
              <Typography variant="body2">{startConfig.host}</Typography>
            </Link>
            {/* {provider.isLocalHost && (
              <Typography variant="body2" color="grey">
                (localhost)
              </Typography>
            )} */}
            <Tooltip title={startConfig.rosVersion === "2" ? "ROS_DOMAIN_ID" : "Network ID"} placement="right">
              <Typography color="grey" variant="body2">
                [{startConfig.networkId}]
              </Typography>
            </Tooltip>
          </Stack>
        </TableCell>
        <TableCell
          style={{
            padding: 2,
            flexGrow: 1,
            width: "100%",
          }}
        >
          <Tooltip title={"Click to start provider"} placement="bottom" disableInteractive>
            <span>
              {window.commandExecutor && (
                <Button
                  style={{
                    textTransform: "none",
                  }}
                  onClick={() => {
                    handleStartProvider();
                  }}
                  variant="text"
                  color="info"
                  size="small"
                  endIcon={<PlayCircleOutlineIcon fontSize="inherit" />}
                >
                  {/* <div style={{ color: "red", whiteSpace: "nowrap" }}>{state}</div> */}
                </Button>
              )}
              {/* {!window.commandExecutor && <div style={{ color: "red", whiteSpace: "nowrap" }}>{state}</div>} */}
            </span>
          </Tooltip>
        </TableCell>
        {/* <TableCell style={{ padding: 0 }}>
          {![
            ConnectionState.STATES.SERVER_CONNECTED,
            ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED,
            ConnectionState.STATES.STARTING,
            ConnectionState.STATES.CONNECTING,
            ConnectionState.STATES.CONNECTED,
          ].includes(provider.connectionState as string) && (
            <Tooltip title="Remove host" placement="bottom" disableInteractive>
              <IconButton
                color="error"
                onClick={() => {
                  rosCtx.removeProvider(provider.id);
                }}
                size="small"
              >
                <DeleteOutlineOutlinedIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          )}
        </TableCell> */}
      </TableRow>
    );
  }, [startConfig, updated]);

  return createTableRow;
}
