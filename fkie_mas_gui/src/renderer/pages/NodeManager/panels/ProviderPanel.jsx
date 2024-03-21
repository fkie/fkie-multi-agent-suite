import { useDebounceCallback } from '@react-hook/debounce';
import { useContext, useEffect, useReducer, useState } from 'react';
import semver from 'semver';

import DeleteOutlineOutlinedIcon from '@mui/icons-material/DeleteOutlineOutlined';
import HighlightOffIcon from '@mui/icons-material/HighlightOff';
import JoinFullIcon from '@mui/icons-material/JoinFull';
import PlayCircleOutlineIcon from '@mui/icons-material/PlayCircleOutline';
import RefreshIcon from '@mui/icons-material/Refresh';
import UpgradeIcon from '@mui/icons-material/Upgrade';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import {
  Button,
  CircularProgress,
  IconButton,
  LinearProgress,
  Link,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  Tooltip,
  Typography,
} from '@mui/material';

import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';
import {
  ConnectToProviderModal,
  SearchBar,
  colorFromHostname,
} from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { RosNode } from '../../../models';
import { CmdType, ConnectionState } from '../../../providers';
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_STATE,
  EVENT_PROVIDER_WARNINGS,
} from '../../../providers/events';
import {
  EVENT_OPEN_COMPONENT,
  EVENT_OPEN_CONNECT,
  EVENT_OPEN_SETTINGS,
  SETTING,
  eventOpenComponent,
  eventOpenSettings,
} from '../../../utils/events';
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from '../layout';
import SingleTerminalPanel from './SingleTerminalPanel';

import SystemInformationPanel from './SystemInformationPanel';

function ProviderPanel() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState([]);
  const [filterText, setFilterText] = useState('');
  const [providersActivity] = useState(new Map());
  const [, forceUpdate] = useReducer((x) => x + 1, 0);
  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  const closeProviderHandler = async (providerId) => {
    const provider = rosCtx.getProviderById(providerId);
    if (provider) {
      provider.close();
    }
  };

  const handleJoinProvider = async (provider) => {
    await rosCtx.connectToProvider(provider);
  };

  const handleStartProvider = async (provider) => {
    await rosCtx.startProvider(provider, true);
  };

  const debouncedCallbackFilterText = useDebounceCallback(
    (providers, searchTerm) => {
      if (searchTerm.length > 1) {
        const re = new RegExp(searchTerm, 'i');
        setProviderRowsFiltered(
          providers.filter((provider) => {
            const pos = provider.name().search(re);
            return pos !== -1;
          }),
        );
      } else {
        setProviderRowsFiltered(providers);
      }
    },
    300,
  );

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText, debouncedCallbackFilterText]);

  useEffect(() => {
    if (rosCtx.providers.length === 0) {
      emitCustomEvent(EVENT_OPEN_CONNECT, {});
    }
  }, [rosCtx.rosInfo, rosCtx.providers]);

  useCustomEventListener(EVENT_PROVIDER_ACTIVITY, (data) => {
    providersActivity.set(data.provider.id, data.active);
    forceUpdate();
  });

  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useCustomEventListener(EVENT_PROVIDER_WARNINGS, (data) => {
    forceUpdate();
  });

  const onProviderMenuClick = async (actionType, providerId, providerName) => {
    if (actionType === 'INFO') {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `provider-info-${providerName}`,
          providerName,
          <SystemInformationPanel providerId={providerId} />,
          true,
          LAYOUT_TABS.HOSTS,
          new LayoutTabConfig(false, 'info'),
        ),
      );
      return;
    }
    if (actionType === 'DELETE') {
      // rosCtx.removeProvider(providerId);
    }
  };

  const generateStatusView = (provider) => {
    switch (provider.connectionState) {
      case ConnectionState.STATES.CROSSBAR_CONNECTED:
      case ConnectionState.STATES.CROSSBAR_REGISTERED:
      case ConnectionState.STATES.CONNECTING:
        return (
          <Stack
            direction="row"
            alignItems="center"
            spacing="0.5em"
            paddingRight="0.5em"
          >
            <Tooltip title="Connecting" placement="bottom">
              <span style={{ color: 'blue' }}>connecting</span>
            </Tooltip>

            <CircularProgress size="1em" />
            <Tooltip title="cancel" placement="bottom">
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
          <Stack
            direction="row"
            alignItems="center"
            spacing="0.5em"
            paddingRight="0.5em"
          >
            <div style={{ color: 'blue' }}>{provider.connectionState}</div>
            <CircularProgress size="1em" />
          </Stack>
        );
      case ConnectionState.STATES.CONNECTED:
        return (
          <Stack direction="row" alignItems="center">
            <div style={{ color: 'green' }}>{provider.connectionState}</div>
            <Tooltip title="Disconnect" placement="bottom">
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
            >
              <Button
                style={{ textTransform: 'none' }}
                onClick={() => {
                  emitCustomEvent(
                    EVENT_OPEN_SETTINGS,
                    eventOpenSettings(SETTING.IDS.SSH),
                  );
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
            <Tooltip title="Start daemon" placement="bottom">
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
        let state = provider.connectionState;
        if (provider.connectionState === ConnectionState.STATES.ERRORED) {
          if (!provider.daemon) {
            state = 'no daemon';
          } else if (!provider.discovery) {
            state = 'no discovery';
          }
        }
        return (
          <Stack direction="row" alignItems="center" justifyContent="center">
            <Tooltip
              title={`Click to start provider! ${
                provider.errorDetails
                  ? `${state}: ${JSON.stringify(provider.errorDetails)}`
                  : ''
              }`}
              placement="bottom"
            >
              <span>
                {window.CommandExecutor && (
                  <Button
                    style={{
                      textTransform: 'none',
                    }}
                    onClick={() => {
                      handleStartProvider(provider);
                    }}
                    variant="text"
                    color="info"
                    size="small"
                    endIcon={<PlayCircleOutlineIcon fontSize="inherit" />}
                  >
                    <div style={{ color: 'red', whiteSpace: 'nowrap' }}>
                      {state}
                    </div>
                  </Button>
                )}
                {!window.CommandExecutor && (
                  <div style={{ color: 'red', whiteSpace: 'nowrap' }}>
                    {state}
                  </div>
                )}
              </span>
            </Tooltip>

            <Tooltip title="Join to running daemon" placement="bottom">
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
            <div style={{ color: 'grey' }}>{provider.connectionState}</div>
            <Tooltip title="Join to running daemon" placement="bottom">
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
  };

  const generateWarningsView = (provider) => {
    const warnings = provider.warnings.filter(
      (group) => group.warnings.length > 0,
    );
    if (warnings.length > 0) {
      return (
        <Tooltip
          title={`Provider reports warning for ${warnings.map(
            (item) => item.id,
          )}`}
          placement="bottom"
        >
          <IconButton
            color="default"
            onClick={() => {
              onProviderMenuClick('INFO', provider.id, provider.name());
            }}
          >
            <WarningAmberIcon color="warning" fontSize="inherit" />
          </IconButton>
        </Tooltip>
      );
    }
  };

  const getHostStyle = (provider) => {
    if (settingsCtx.get('colorizeHosts')) {
      // borderLeft: `3px dashed`,
      // borderColor: colorFromHostname(provider.name()),
      return {
        borderLeftStyle: 'solid',
        borderLeftColor: colorFromHostname(provider.name()),
        borderLeftWidth: '0.6em',
      };
    }
    return {};
  };

  const isOlderVersion = (provider) => {
    try {
      if (provider.getDaemonReleaseVersion().indexOf('unknown') > -1) {
        return true;
      }
      return semver.gt(
        settingsCtx.MIN_VERSION_DAEMON,
        provider.getDaemonReleaseVersion(),
      );
    } catch {}
    return false;
  };

  return (
    <Stack
      spacing={1}
      height="100%"
      // width="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
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
        <ConnectToProviderModal />
        <Tooltip
          title="Refresh hosts list"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
        >
          <IconButton
            edge="start"
            aria-label="refresh hosts list"
            onClick={() => rosCtx.refreshProviderList()}
          >
            <RefreshIcon sx={{ fontSize: 'inherit' }} />
          </IconButton>
        </Tooltip>
      </Stack>
      <TableContainer>
        <Table aria-label="hosts table">
          <TableBody>
            {providerRowsFiltered.map((provider) => (
              <TableRow
                key={provider.id}
                style={{
                  display: 'block',
                  padding: 0,
                }}
              >
                {/* <TableCell style={{ padding: 0 }}>
                  {getAutoConnectButton(provider)}
                </TableCell> */}
                <TableCell
                  style={{
                    padding: 2,
                    flexGrow: 1,
                    width: '100%',
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
                        onProviderMenuClick(
                          'INFO',
                          provider.id,
                          provider.name(),
                        );
                      }}
                    >
                      <Typography variant="body2">{provider.name()}</Typography>
                    </Link>
                    {provider.isLocalHost && (
                      <Typography variant="body2" color="grey">
                        (localhost)
                      </Typography>
                    )}
                    {provider.rosState.ros_domain_id !== undefined &&
                      provider.rosState.ros_domain_id > 0 && (
                        <Tooltip
                          title={
                            provider.rosVersion === '2'
                              ? 'ROS_DOMAIN_ID'
                              : 'Network ID'
                          }
                          placement="right"
                        >
                          <Typography color="grey" variant="body2">
                            [{provider.rosState.ros_domain_id}]
                          </Typography>
                        </Tooltip>
                      )}
                    {providersActivity.get(provider.id) && (
                      <Stack minWidth="2em">
                        <LinearProgress
                          sx={{ marginTop: '0.5em' }}
                          variant="query"
                          color="inherit"
                        />
                      </Stack>
                    )}
                  </Stack>
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  {isOlderVersion(provider) && (
                    <Tooltip
                      title={`daemon has older version ${provider.getDaemonReleaseVersion()}, open terminal for update`}
                      placement="bottom-start"
                      enterDelay={tooltipDelay}
                      enterNextDelay={tooltipDelay}
                    >
                      <IconButton
                        edge="start"
                        onClick={() => {
                          // open terminal for update
                          const emptyNode = new RosNode();
                          emptyNode.name = '';
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
                                cmd={''}
                              />,
                              true,
                              LAYOUT_TAB_SETS.BORDER_BOTTOM,
                              new LayoutTabConfig(true, type, {
                                type,
                                providerId: emptyNode.providerId,
                                nodeName: emptyNode.name,
                                cmd: '',
                              }),
                            ),
                          );
                        }}
                      >
                        <UpgradeIcon
                          sx={{ fontSize: 'inherit', color: 'orange' }}
                        />
                      </IconButton>
                    </Tooltip>
                  )}
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  {generateWarningsView(provider)}
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  {generateStatusView(provider)}
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  {![
                    ConnectionState.STATES.CROSSBAR_CONNECTED,
                    ConnectionState.STATES.CROSSBAR_REGISTERED,
                    ConnectionState.STATES.STARTING,
                    ConnectionState.STATES.CONNECTING,
                    ConnectionState.STATES.CONNECTED,
                  ].includes(provider.connectionState) && (
                    <Tooltip title="Remove host" placement="bottom">
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
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    </Stack>
  );
}

ProviderPanel.defaultProps = {};

ProviderPanel.propTypes = {};

export default ProviderPanel;
