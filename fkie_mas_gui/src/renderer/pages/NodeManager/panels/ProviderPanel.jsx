import { useDebounceCallback } from '@react-hook/debounce';
import { useContext, useEffect, useReducer, useState } from 'react';

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

import HighlightOffIcon from '@mui/icons-material/HighlightOff';
import JoinFullIcon from '@mui/icons-material/JoinFull';
import LinkIcon from '@mui/icons-material/Link';
import LinkOffIcon from '@mui/icons-material/LinkOff';
import PlayCircleOutlineIcon from '@mui/icons-material/PlayCircleOutline';
import RefreshIcon from '@mui/icons-material/Refresh';

import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';
import {
  ConnectToProviderModal,
  SearchBar,
  colorFromHostname,
} from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { ConnectionState } from '../../../providers';
import {
  EVENT_PROVIDER_ACTIVITY,
  EVENT_PROVIDER_STATE,
} from '../../../providers/events';
import {
  EVENT_OPEN_COMPONENT,
  EVENT_OPEN_SETTINGS,
  SETTING,
  eventOpenComponent,
  eventOpenSettings,
} from '../../../utils/events';

import { LAYOUT_TABS } from '../layout';
import OverflowMenuProvider from './OverflowMenuProvider';
import SystemInformationPanel from './SystemInformationPanel';

function ProviderPanel() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState([]);
  const [filterText, setFilterText] = useState('');
  const [providersActivity] = useState(new Map());
  // use updateActivity as trigger state to avoid copy of the providersActivity-Map
  const [updateActivity, setUpdateActivity] = useState(1);
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

  useCustomEventListener(EVENT_PROVIDER_ACTIVITY, (data) => {
    providersActivity.set(data.provider.id, data.active);
    setUpdateActivity(updateActivity * -1);
  });

  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  const onProviderMenuClick = async (actionType, providerId, providerName) => {
    if (actionType === 'INFO') {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `provider-info-${providerName}`,
          `Info - ${providerName}`,
          <SystemInformationPanel providerId={providerId} />,
          false,
          true,
          LAYOUT_TABS.PROVIDER,
        ),
      );
      return;
    }
    if (actionType === 'DELETE') {
      rosCtx.deleteProviderConfig(providerId);
    }
  };

  const changeAutoConnect = (provider, enabled) => {
    const config = rosCtx.getProviderLaunchConfig(provider.id);
    if (config) {
      config.autoConnect = enabled;
      rosCtx.saveProviderConfig(config);
      forceUpdate();
    }
  };

  const getAutoConnectButton = (provider) => {
    const config = rosCtx.getProviderLaunchConfig(provider.id);
    let autoConnect = false;
    if (config) {
      autoConnect = config.autoConnect;
      if (autoConnect) {
        return (
          <Tooltip title="Connect on start enabled" placement="bottom">
            <IconButton
              color="primary"
              onClick={() => {
                changeAutoConnect(provider, false);
              }}
              size="small"
            >
              <LinkIcon fontSize="inherit" />
            </IconButton>
          </Tooltip>
        );
      }
      return (
        <Tooltip title="Connect on start disabled" placement="bottom">
          <IconButton
            color="default"
            onClick={() => {
              changeAutoConnect(provider, true);
            }}
            size="small"
          >
            <LinkOffIcon fontSize="inherit" />
          </IconButton>
        </Tooltip>
      );
    }
    return (
      <Tooltip title="Discovered provider can't be changed" placement="bottom">
        <IconButton
          // style={{ color: 'SandyBrown' }}
          color="default"
          disabled={false}
          size="small"
        >
          <LinkIcon fontSize="inherit" />
        </IconButton>
      </Tooltip>
    );
  };

  const generateStatusView = (provider) => {
    switch (provider.connectionState) {
      case ConnectionState.STATES.CROSSBAR_CONNECTED:
      case ConnectionState.STATES.CROSSBAR_REGISTERED:
      case ConnectionState.STATES.CONNECTING:
        return (
          <Stack direction="row" alignItems="center" spacing="0.5em">
            <Tooltip title="Connecting" placement="bottom">
              <div style={{ color: 'blue' }}>connecting</div>
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
          <Stack direction="row" alignItems="center" spacing="0.5em">
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
                {provider.connectionState}
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

  const getHostStyle = (provider) => {
    if (settingsCtx.get('colorizeHosts')) {
      // borderLeft: `3px dashed`,
      // borderColor: colorFromHostname(provider.name()),
      return {
        borderLeftStyle: 'solid',
        borderLeftColor: colorFromHostname(provider.name()),
        borderLeftWidth: '10px',
      };
    }
    return {};
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
          placeholder="Filter provider"
          defaultValue={filterText}
          // fullWidth={true}
        />
        <ConnectToProviderModal />
        <Tooltip
          title="Refresh provider list"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
        >
          <IconButton
            edge="start"
            aria-label="refresh provider list"
            onClick={() => rosCtx.refreshProviderList()}
          >
            <RefreshIcon sx={{ fontSize: 'inherit' }} />
          </IconButton>
        </Tooltip>
      </Stack>
      <TableContainer>
        <Table aria-label="providers table">
          <TableBody>
            {providerRowsFiltered.map((provider) => (
              <TableRow
                key={provider.id}
                style={{
                  display: 'block',
                  padding: 0,
                }}
              >
                <TableCell style={{ padding: 0 }}>
                  {getAutoConnectButton(provider)}
                </TableCell>
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
                          <Typography color="grey">
                            [{provider.rosState.ros_domain_id}]
                          </Typography>
                        </Tooltip>
                      )}
                    {updateActivity && providersActivity.get(provider.id) && (
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
                  {generateStatusView(provider)}
                </TableCell>
                <TableCell style={{ padding: 0 }}>
                  <OverflowMenuProvider
                    onClick={onProviderMenuClick}
                    providerId={provider.id}
                    providerName={provider.name()}
                  />
                </TableCell>
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
