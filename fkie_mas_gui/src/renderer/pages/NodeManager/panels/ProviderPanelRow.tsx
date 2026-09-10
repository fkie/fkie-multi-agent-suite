import {
  EventProviderActivity,
  EventProviderDelay,
  EventProviderState,
  EventProviderWarnings,
} from "@/renderer/providers/events";
import CheckIcon from "@mui/icons-material/Check";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import HighlightOffIcon from "@mui/icons-material/HighlightOff";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import TextSnippetOutlinedIcon from "@mui/icons-material/TextSnippetOutlined";
import UpgradeIcon from "@mui/icons-material/Upgrade";
import VerticalAlignBottomIcon from "@mui/icons-material/VerticalAlignBottom";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import {
  Button,
  CircularProgress,
  IconButton,
  LinearProgress,
  Link,
  Stack,
  TableCell,
  TableRow,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useMemo, useReducer, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import semver from "semver";

import { LAYOUT_TABS } from "@/renderer/components/layout";
import { useAutoUpdateContext } from "@/renderer/context/AutoUpdateContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { RosNode } from "@/renderer/models";
import { ConnectionState, Provider } from "@/renderer/providers";
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_DELAY,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from "@/renderer/providers/eventTypes";
import { CmdTypes } from "@/types";
import { emitSelectTab } from "../../../components/layout/events";
import { EMenuProvider } from "./OverflowMenuProvider";

interface ProviderPanelRowProps {
  provider: Provider;
}

export default function ProviderPanelRow(props: ProviderPanelRowProps): JSX.Element {
  const { provider } = props;
  const auCtx = useAutoUpdateContext();
  const rosCtx = useRosContext();
  const navCtx = useNavigationContext();
  const settingsCtx = useSettingsContext();
  const [providersActivity, setProvidersActivity] = useState(false);
  const [updated, forceUpdate] = useReducer((x) => x + 1, 0);
  const [colorizeHosts] = useSetting<boolean>("colorizeHosts");
  const [dedicatedTabsFor] = useSetting<string>("dedicatedTabsFor");

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
      `ros2 run fkie_mas_daemon mas-remote-node.py --show_ros_log /mas/_daemon_${provider.connection.domainId}_{HOST}`
    );
  }

  async function handleJoinProvider(provider: Provider): Promise<void> {
    await rosCtx.connectToProvider(provider);
  }

  const debouncedCallbackUpdateDelay = useDebounceCallback(() => {
    forceUpdate();
  }, 1000);

  useCustomEventListener(EVENT_PROVIDER_ACTIVITY, (data: EventProviderActivity) => {
    if (data.provider.id === provider.id) {
      setProvidersActivity(data.active);
    }
  });

  useCustomEventListener(EVENT_PROVIDER_DELAY, (data: EventProviderDelay) => {
    if (data.provider.id === provider.id) {
      debouncedCallbackUpdateDelay();
    }
  });

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data: EventProviderWarnings) => {
    if (data.provider.id === provider.id) {
      forceUpdate();
    }
  });

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    if (data.provider.id === provider.id) {
      forceUpdate();
    }
  });

  async function onProviderMenuClick(actionType: EMenuProvider, provider: Provider): Promise<void> {
    if (actionType === EMenuProvider.INFO) {
      const nodes = Array.from(rosCtx.nodeMap)
        .filter((value: [string, RosNode]) => {
          return value[0].startsWith(provider.id);
        })
        .map((value) => {
          return value[0];
        });
      navCtx.setSelected("provider-panel", [provider.id, ...nodes], true);
      if (dedicatedTabsFor === "HOSTS") {
        emitSelectTab({ tabId: `${LAYOUT_TABS.DOMAIN}-${provider.id}` });
        emitSelectTab({ tabId: `${LAYOUT_TABS.NODES}-${provider.id}` });
      } else {
        emitSelectTab({ tabId: `${LAYOUT_TABS.DOMAIN}-${provider.connection.domainId}` });
        emitSelectTab({ tabId: `${LAYOUT_TABS.NODES}-${provider.connection.domainId}` });
      }
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
            {/* <Tooltip title="Start daemon" placement="bottom" disableInteractive>
              <IconButton
                color="default"
                onClick={() => {
                  handleStartProvider(provider);
                }}
              >
                <PlayCircleOutlineIcon fontSize="inherit" />
              </IconButton>
            </Tooltip> */}
          </Stack>
        );
      case ConnectionState.STATES.LOST:
      case ConnectionState.STATES.UNSUPPORTED:
      case ConnectionState.STATES.UNREACHABLE:
      case ConnectionState.STATES.ERRORED: {
        return (
          <Stack direction="row" alignItems="center" justifyContent="center">
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
    return <></>;
  }

  const getHostStyle = useCallback(
    (provider: Provider) => {
      if (colorizeHosts) {
        return {
          borderLeftStyle: "solid",
          borderLeftColor: rosCtx.providerColor(provider.id),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [colorizeHosts, rosCtx.providerColor]
  );

  const isOlderVersion = useCallback((): boolean => {
    try {
      if (provider.getDaemonReleaseVersion().indexOf("unknown") > -1) {
        return true;
      }
      return semver.gt(settingsCtx.MIN_VERSION_DAEMON, provider.getDaemonReleaseVersion());
    } catch {
      // no output on version errors
    }
    return false;
  }, [settingsCtx.MIN_VERSION_DAEMON, provider]);

  const isNewerVersion = useCallback((): boolean => {
    try {
      if (semver.major(settingsCtx.MIN_VERSION_DAEMON) < semver.major(provider.getDaemonReleaseVersion())) {
        return true;
      }
    } catch {
      // no output on version errors
    }
    return false;
  }, [settingsCtx.MIN_VERSION_DAEMON, provider]);

  const getVersionColor = useCallback((): string => {
    try {
      if (provider.getDaemonReleaseVersion().indexOf("unknown") > -1) {
        return "grey";
      }
      if (semver.major(settingsCtx.MIN_VERSION_DAEMON) !== semver.major(provider.getDaemonReleaseVersion())) {
        return "red";
      }
      if (semver.minor(settingsCtx.MIN_VERSION_DAEMON) !== semver.minor(provider.getDaemonReleaseVersion())) {
        return "HotPink";
      }
      if (semver.patch(settingsCtx.MIN_VERSION_DAEMON) !== semver.patch(provider.getDaemonReleaseVersion())) {
        return "orange";
      }
    } catch {
      // no output on version errors
    }
    return "grey";
  }, [settingsCtx.MIN_VERSION_DAEMON, provider]);

  const getDelayColor = useCallback((delay: number) => {
    if (delay < 0.1) {
      return "green";
    }
    if (delay < 0.5) {
      return "orange";
    }
    return "red";
  }, []);

  const formatDelay = useCallback((delay: number) => {
    const dp = delay > 0 ? delay : delay * -1.0;
    if (dp < 0.001) {
      return `${(dp * 1000.0).toFixed(1)}ms`;
    }
    if (dp < 0.5) {
      return `${(dp * 1000.0).toFixed(0)}ms`;
    }
    return `${dp.toFixed(0)}s`;
  }, []);

  const createTableRow = useMemo(() => {
    const domainTitle = provider.rosVersion === "2" ? "ROS_DOMAIN_ID" : "Network ID";
    return (
      <TableRow
        key={provider.id}
        style={{
          display: "flex",
          padding: 0,
        }}
      >
        <TableCell
          style={{
            padding: 2,
            flexGrow: 1,
            flexShrink: 1,
            minWidth: 20,
            overflow: "hidden",
          }}
          sx={getHostStyle(provider)}
        >
          <Stack direction="row" spacing="0.2em" alignItems="center" flexGrow={1}>
            {/* {provider.isLocalHost && <OverflowMenuExternalApps provider={provider} />} */}
            <Tooltip
              title={
                <div>
                  <Typography fontWeight="bold" fontSize="inherit">
                    {provider.name()}
                  </Typography>
                  {provider.isLocalHost && (
                    <Typography fontWeight="bold" fontSize="inherit">
                      &gt;localhost&lt;
                    </Typography>
                  )}
                  <Stack direction="row" spacing={"0.2em"}>
                    <Typography fontWeight="bold" fontSize="inherit">
                      {domainTitle}:
                    </Typography>
                    <Typography fontSize="inherit">{provider.connection.domainId}</Typography>
                  </Stack>
                  <Stack direction="row" spacing={"0.2em"}>
                    <Typography fontWeight="bold" fontSize="inherit">
                      daemon websocket port:
                    </Typography>
                    <Typography fontSize="inherit">{provider.connection.port}</Typography>
                  </Stack>
                  <Stack direction="row" spacing={"0.2em"}>
                    <Typography fontSize="inherit">{provider.additionInfo}</Typography>
                  </Stack>
                </div>
              }
              placement="bottom-start"
              disableInteractive
            >
              <Stack direction="row" spacing="0.2em" alignItems="center" flexGrow={1}>
                <Link
                  noWrap
                  href="#"
                  underline="none"
                  color="inherit"
                  onClick={() => {
                    onProviderMenuClick(EMenuProvider.INFO, provider);
                  }}
                  sx={{ flexShrink: 1, minWidth: 20, overflow: "hidden", textOverflow: "ellipsis" }}
                >
                  <Typography
                    variant="body2"
                    sx={{ flexShrink: 1, minWidth: 20, overflow: "hidden", textOverflow: "ellipsis" }}
                  >
                    {provider.name()}
                  </Typography>
                </Link>
                {provider.isLocalHost && (
                  <Typography
                    variant="body2"
                    color="grey"
                    sx={{
                      flexShrink: 1,
                      minWidth: 20,
                      overflow: "hidden",
                      textOverflow: "ellipsis",
                      whiteSpace: "nowrap",
                    }}
                  >
                    - localhost
                  </Typography>
                )}
                {providersActivity && (
                  <Stack minWidth="2em">
                    <LinearProgress sx={{ marginTop: "0.3em" }} variant="query" color="inherit" />
                  </Stack>
                )}
              </Stack>
            </Tooltip>
          </Stack>
        </TableCell>
        <TableCell
          style={{
            padding: 0,
          }}
        >
          {provider.isAvailable() && (
            <Tooltip title="websocket delay from the host" placement="bottom-start" disableInteractive>
              <Typography
                variant="body2"
                fontSize="0.8em"
                sx={{ paddingTop: "0.5em", paddingLeft: "0.5em", paddingRight: "0.5em" }}
                color={getDelayColor(provider.currentDelay)}
              >
                {formatDelay(provider.currentDelay)}
              </Typography>
            </Tooltip>
          )}
        </TableCell>
        <TableCell style={{ padding: 0 }}>
          {isOlderVersion() && (
            <Tooltip
              title={`daemon has older version ${provider.getDaemonReleaseVersion()}, required: ${settingsCtx.MIN_VERSION_DAEMON}, open terminal for update`}
              placement="bottom-start"
              disableInteractive
            >
              <IconButton
                edge="start"
                onClick={(event) => {
                  // open terminal for update
                  navCtx.openTerminal(
                    CmdTypes.CMD,
                    provider.id,
                    "",
                    "",
                    auCtx.getUpdateCli(false, true),
                    event.nativeEvent.shiftKey,
                    event.nativeEvent.ctrlKey
                  );
                }}
              >
                <UpgradeIcon sx={{ fontSize: "inherit", color: getVersionColor() }} />
              </IconButton>
            </Tooltip>
          )}
          {isNewerVersion() && (
            <Tooltip
              title={`daemon has a newer version ${provider.getDaemonReleaseVersion()} with broken changes. This GUI requires ${settingsCtx.MIN_VERSION_DAEMON}. In case of problems, please open a terminal and downgrade the daemon version.`}
              placement="bottom-start"
              disableInteractive
            >
              <IconButton
                edge="start"
                onClick={(event) => {
                  // open terminal for update
                  navCtx.openTerminal(
                    CmdTypes.TERMINAL,
                    provider.id,
                    "",
                    "",
                    "",
                    event.nativeEvent.shiftKey,
                    event.nativeEvent.ctrlKey
                  );
                }}
              >
                <VerticalAlignBottomIcon sx={{ fontSize: "inherit", color: getVersionColor() }} />
              </IconButton>
            </Tooltip>
          )}
        </TableCell>
        <TableCell style={{ padding: 0 }}>{generateWarningsView(provider)}</TableCell>
        <TableCell style={{ padding: 0 }}>{generateStatusView(provider)}</TableCell>
        <TableCell style={{ padding: 0 }}>
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
        </TableCell>
      </TableRow>
    );
  }, [provider, providersActivity, updated]);

  return createTableRow;
}
