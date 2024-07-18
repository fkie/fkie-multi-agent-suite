import CheckIcon from "@mui/icons-material/Check";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import HighlightOffIcon from "@mui/icons-material/HighlightOff";
import JoinFullIcon from "@mui/icons-material/JoinFull";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import UpgradeIcon from "@mui/icons-material/Upgrade";
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
import PropTypes from "prop-types";
import { useCallback, useContext, useMemo, useReducer, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import semver from "semver";
import { colorFromHostname } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { RosNode } from "../../../models";
import { CmdType, ConnectionState } from "../../../providers";
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_DELAY,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from "../../../providers/eventTypes";
import {
  EVENT_OPEN_COMPONENT,
  EVENT_OPEN_SETTINGS,
  SETTING,
  eventOpenComponent,
  eventOpenSettings,
} from "../../../utils/events";
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import SingleTerminalPanel from "./SingleTerminalPanel";
import SystemInformationPanel from "./SystemInformationPanel";

function ProviderPanelRow({ provider }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [providersActivity, setProvidersActivity] = useState(false);
  const [updated, forceUpdate] = useReducer((x) => x + 1, 0);
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const closeProviderHandler = useCallback(
    async (providerId) => {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        provider.close();
      }
    },
    [rosCtx]
  );

  const handleJoinProvider = useCallback(
    async (provider) => {
      await rosCtx.connectToProvider(provider);
    },
    [rosCtx]
  );

  const handleStartProvider = useCallback(
    async (provider) => {
      await rosCtx.startProvider(provider, true);
    },
    [rosCtx]
  );

  const debouncedCallbackUpdateDelay = useDebounceCallback(() => {
    forceUpdate();
  }, 1000);

  useCustomEventListener(EVENT_PROVIDER_ACTIVITY, (data) => {
    if (data.provider.id === provider.id) {
      setProvidersActivity(data.active);
    }
  });

  useCustomEventListener(EVENT_PROVIDER_DELAY, (data) => {
    if (data.provider.id === provider.id) {
      debouncedCallbackUpdateDelay();
    }
  });

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data) => {
    if (data.provider.id === provider.id) {
      forceUpdate();
    }
  });

  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    if (data.provider.id === provider.id) {
      forceUpdate();
    }
  });

  const onProviderMenuClick = async (actionType, providerId, providerName) => {
    if (actionType === "INFO") {
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
    if (actionType === "DELETE") {
      // rosCtx.removeProvider(providerId);
    }
  };

  const generateStatusView = useCallback(
    (provider) => {
      switch (provider.connectionState) {
        case ConnectionState.STATES.SERVER_CONNECTED:
        case ConnectionState.STATES.SUBSCRIPTIONS_REGISTERED:
        case ConnectionState.STATES.CONNECTING:
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
              <CheckIcon style={{ color: "green" }} fontSize="0.6em" />

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
        case ConnectionState.STATES.NO_SSH_CREDENTIALS:
          return (
            <Stack direction="row" alignItems="center">
              <Tooltip
                title={`Can't access remote host! Please add SSH credentials.`}
                placement="bottom"
                disableInteractive
              >
                <Button
                  style={{ textTransform: "none" }}
                  onClick={() => {
                    emitCustomEvent(EVENT_OPEN_SETTINGS, eventOpenSettings(SETTING.IDS.SSH));
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
                  {window.CommandExecutor && (
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
                  {!window.CommandExecutor && <div style={{ color: "red", whiteSpace: "nowrap" }}>{state}</div>}
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
              {window.CommandExecutor && (
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
    },
    [closeProviderHandler, handleJoinProvider, handleStartProvider]
  );

  const generateWarningsView = useCallback((provider) => {
    if (!provider.warnings) return <></>;
    const warnings = provider.warnings.filter((group) => group.warnings.length > 0);
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
              onProviderMenuClick("INFO", provider.id, provider.name());
            }}
          >
            <WarningAmberIcon color="warning" fontSize="inherit" />
          </IconButton>
        </Tooltip>
      );
    }
    // eslint-disable-next-line react/jsx-no-useless-fragment
    return <></>;
  }, []);

  const getHostStyle = useCallback(
    (provider) => {
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
    [settingsCtx]
  );

  const isOlderVersion = useCallback(
    (provider) => {
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
    [settingsCtx.MIN_VERSION_DAEMON]
  );

  const getDelayColor = useCallback((delay) => {
    if (delay < 0.1) {
      return "green";
    }
    if (delay < 0.5) {
      return "orange";
    }
    return "red";
  }, []);

  const formatDelay = useCallback((delay) => {
    const dp = delay > 0 ? delay : delay * -1.0;
    if (dp < 0.001) {
      return `${(dp * 1000.0).toFixed(1)}ms`;
    }
    if (dp < 0.5) {
      return `${(dp * 1000.0).toFixed(0)}ms`;
    }
    return `${dp.toFixed(0)}s`;
    {
      provider.currentDelay < 0.1 ? provider.currentDelay.toFixed(3) : provider.currentDelay.toFixed(2);
    }

    if (delay < 0.1) {
      return "green";
    }
    if (delay < 0.5) {
      return "orange";
    }
    return "red";
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
                onProviderMenuClick("INFO", provider.id, provider.name());
              }}
            >
              <Typography variant="body2">{provider.name()}</Typography>
            </Link>
            {provider.isLocalHost && (
              <Typography variant="body2" color="grey">
                (localhost)
              </Typography>
            )}
            {provider.rosState.ros_domain_id !== undefined && provider.rosState.ros_domain_id > 0 && (
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
          {isOlderVersion(provider) && (
            <Tooltip
              title={`daemon has older version ${provider.getDaemonReleaseVersion()}, open terminal for update`}
              placement="bottom-start"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <IconButton
                edge="start"
                onClick={() => {
                  // open terminal for update
                  const emptyNode = new RosNode();
                  emptyNode.name = "";
                  emptyNode.providerId = provider.id;
                  emptyNode.providerName = provider.name();
                  const type = CmdType.TERMINAL;
                  const id = `${type}${emptyNode.name}@${emptyNode.providerName}`;
                  emitCustomEvent(
                    EVENT_OPEN_COMPONENT,
                    eventOpenComponent(
                      id,
                      `${emptyNode.providerName}`,
                      <SingleTerminalPanel
                        id={id}
                        type={type}
                        providerId={emptyNode.providerId}
                        node={emptyNode}
                        cmd=""
                      />,
                      true,
                      LAYOUT_TAB_SETS.BORDER_BOTTOM,
                      new LayoutTabConfig(true, type, {
                        type,
                        providerId: emptyNode.providerId,
                        nodeName: emptyNode.name,
                        cmd: "",
                      })
                    )
                  );
                }}
              >
                <UpgradeIcon sx={{ fontSize: "inherit", color: "orange" }} />
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
          ].includes(provider.connectionState) && (
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

ProviderPanelRow.propTypes = {
  provider: PropTypes.object.isRequired,
};

export default ProviderPanelRow;
