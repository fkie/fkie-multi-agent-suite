import CheckIcon from "@mui/icons-material/Check";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import HighlightOffIcon from "@mui/icons-material/HighlightOff";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
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

import {
  EventProviderActivity,
  EventProviderDelay,
  EventProviderState,
  EventProviderWarnings,
} from "@/renderer/providers/events";
import { useCallback, useContext, useEffect, useMemo, useReducer, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import semver from "semver";

import { colorFromHostname } from "@/renderer/components/UI/Colors";
import { AutoUpdateContext } from "@/renderer/context/AutoUpdateContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { CmdType, ConnectionState, Provider } from "@/renderer/providers";
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_DELAY,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from "@/renderer/providers/eventTypes";
import { LAYOUT_TABS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import { EMenuProvider } from "./OverflowMenuProvider";
import SystemInformationPanel from "./SystemInformationPanel";

interface ProviderPanelRowProps {
  provider: Provider;
}

export default function ProviderPanelRow(props: ProviderPanelRowProps): JSX.Element {
  const { provider } = props;
  const auCtx = useContext(AutoUpdateContext);
  const rosCtx = useContext(RosContext);
  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const [providersActivity, setProvidersActivity] = useState(false);
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

  async function handleJoinProvider(provider: Provider): Promise<void> {
    await rosCtx.connectToProvider(provider);
  }

  async function handleStartProvider(provider: Provider): Promise<void> {
    await rosCtx.startProvider(provider, true);
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

  async function onProviderMenuClick(
    actionType: EMenuProvider,
    providerName: string,
    providerId: string
  ): Promise<void> {
    if (actionType === EMenuProvider.INFO) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `provider-info-${providerName}`,
          providerName,
          <SystemInformationPanel providerId={providerId} />,
          true,
          LAYOUT_TABS.HOSTS,
          new LayoutTabConfig(false, "info")
        )
      );
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
                  handleStartProvider(provider);
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
      case ConnectionState.STATES.ERRORED:
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
                      handleStartProvider(provider);
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
      default:
        return (
          <Stack direction="row" alignItems="center">
            <div style={{ color: "grey" }}>{provider.connectionState}</div>
            {window.commandExecutor && (
              <Tooltip title="Start daemon" placement="bottom" disableInteractive>
                <IconButton
                  color="default"
                  onClick={() => {
                    handleStartProvider(provider);
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
              onProviderMenuClick(EMenuProvider.INFO, provider.name(), provider.id);
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
    (provider: Provider) => {
      if (settingsCtx.get("colorizeHosts")) {
        // borderLeft: `3px dashed`,
        // borderColor: colorFromHostname(provider.name()),
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(provider.name()),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  const isOlderVersion = useCallback(
    function (): boolean {
      try {
        if (provider.getDaemonReleaseVersion().indexOf("unknown") > -1) {
          return true;
        }
        return semver.gt(settingsCtx.MIN_VERSION_DAEMON, provider.getDaemonReleaseVersion());
      } catch {
        // no output on version errors
      }
      return false;
    },
    [settingsCtx.MIN_VERSION_DAEMON, provider]
  );

  const isNewerVersion = useCallback(
    function (): boolean {
      try {
        if (semver.major(settingsCtx.MIN_VERSION_DAEMON) < semver.major(provider.getDaemonReleaseVersion())) {
          return true;
        }
      } catch {
        // no output on version errors
      }
      return false;
    },
    [settingsCtx.MIN_VERSION_DAEMON, provider]
  );

  const getVersionColor = useCallback(
    function (): string {
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
    },
    [settingsCtx.MIN_VERSION_DAEMON, provider]
  );

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
    return (
      <TableRow
        key={provider.id}
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
          sx={getHostStyle(provider)}
        >
          <Stack direction="row" spacing="0.5em">
            <Link
              noWrap
              href="#"
              underline="none"
              color="inherit"
              onClick={() => {
                onProviderMenuClick(EMenuProvider.INFO, provider.name(), provider.id);
              }}
            >
              <Typography variant="body2">{provider.name()}</Typography>
            </Link>
            {provider.isLocalHost && (
              <Typography variant="body2" color="grey">
                (localhost)
              </Typography>
            )}
            {provider.rosState.ros_domain_id !== undefined && parseInt(provider.rosState.ros_domain_id) > 0 && (
              <Tooltip title={provider.rosVersion === "2" ? "ROS_DOMAIN_ID" : "Network ID"} placement="right">
                <Typography color="grey" variant="body2">
                  [{provider.rosState.ros_domain_id}]
                </Typography>
              </Tooltip>
            )}
            {providersActivity && (
              <Stack minWidth="2em">
                <LinearProgress sx={{ marginTop: "0.5em" }} variant="query" color="inherit" />
              </Stack>
            )}
          </Stack>
        </TableCell>
        <TableCell style={{ padding: 0 }}>
          {provider.isAvailable() && (
            <Tooltip
              title="websocket delay from the host"
              placement="bottom-start"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <Typography
                variant="body2"
                fontSize="0.8em"
                sx={{ paddingLeft: "0.5em", paddingRight: "0.5em" }}
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
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                edge="start"
                onClick={(event) => {
                  // open terminal for update
                  navCtx.openTerminal(
                    CmdType.CMD,
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
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                edge="start"
                onClick={(event) => {
                  // open terminal for update
                  navCtx.openTerminal(
                    CmdType.TERMINAL,
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
        {/* <TableCell style={{ padding: 0 }}>
    <OverflowMenuProvider
      onClick={onProviderMenuClick}
      providerId={provider.id}
      providerName={provider.name()}
    />
  </TableCell> */}
      </TableRow>
    );
  }, [provider, providersActivity, updated]);

  return createTableRow;
}
