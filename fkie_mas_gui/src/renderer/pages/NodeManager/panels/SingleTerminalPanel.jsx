import { Alert, AlertTitle, Box } from '@mui/material';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';
import TerminalClient from '../../../components/TerminalClient/TerminalClient';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { RosNode } from '../../../models';

function SingleTerminalPanel({ type, providerId, node, screen, width, cmd }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [initialCommands, setInitialCommands] = useState([]);
  const [currentProvider, setCurrentProvider] = useState(null);
  const [lastScreenUsed, setLastScreenUsed] = useState('');

  const initializeTerminal = useDebounceCallback((newScreen = null) => {
    // get current provider
    const provider = rosCtx.getProviderById(providerId);

    if (!provider) {
      setCurrentProvider(null);
      return;
    }

    setCurrentProvider(() => provider);

    if (type === 'cmd') {
      if (cmd) {
        setInitialCommands([`${cmd} \r`]);
      }
    }

    if (type === 'screen') {
      if (newScreen && newScreen.length > 0) {
        setInitialCommands([`screen -d -r ${newScreen}  \r`]);
        setLastScreenUsed(newScreen);
      } else if (screen && screen.length > 0) {
        setInitialCommands([`screen -d -r ${screen}  \r`]);
        setLastScreenUsed(screen);
      } else {
        // search screen with node name
        let screenName = '';
        if (provider.rosState.ros_version === '1') {
          screenName = node.name
            .substring(1)
            .replaceAll('_', '__')
            .replaceAll('/', '_');
        } else if (provider.rosState.ros_version === '2') {
          screenName = node.name.substring(1).replaceAll('/', '.');
        }

        const cmdStr = `screen -d -r $(ps aux | grep "/usr/bin/SCREEN" | grep "${screenName}" | awk '{print $2}') \r`;
        setInitialCommands([cmdStr]);
        setLastScreenUsed('');
      }
    }

    if (type === 'log') {
      if (provider.getLogPaths) {
        let logPaths = [];

        const getLogPaths = async () => {
          const request = [node.name];
          logPaths = await provider.getLogPaths(request);
          if (logPaths.length > 0) {
            // `tail -f ${logPaths[0].screen_log} \r`,
            setInitialCommands(() => [
              `/usr/bin/less -fKLnQrSU ${logPaths[0].screen_log} \r`,
            ]);
          } else {
            setInitialCommands(() => [``]);
          }
        };
        getLogPaths();
      } else {
        console.error('function [getLogPaths] not available by provider');
      }
    }

    if (type === 'terminal') {
      setInitialCommands([`clear \r`]);
    }
  }, 200);

  // load commands initially
  useEffect(() => {
    initializeTerminal();
  }, [initializeTerminal]);

  // update the terminal every time the node screen changes
  useEffect(() => {
    // node changed, update the screen for the current node
    if (node) {
      rosCtx.mapProviderRosNodes.get(providerId)?.forEach((n) => {
        if (n.id === node.id) {
          // TODO: How to handle multiple screens? For now just do this for nodes with a single screen.
          if (n.screens.length === 1) {
            if (n.screens[0] !== screen && n.screens[0] !== lastScreenUsed) {
              // screen changed, reload the component
              // [lastScreenUsed] prevents unnecessary reloads
              setInitialCommands(() => []);
              initializeTerminal(n.screens[0]);
            }
          }
        }
      });
    }
  }, [
    rosCtx.mapProviderRosNodes,
    node,
    screen,
    lastScreenUsed,
    initializeTerminal,
    providerId,
  ]);

  return (
    <Box
      width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      {currentProvider && (
        <Box
          key={`${node ? node.name : 'cmd'}_${providerId}`}
          sx={{
            width: '100%',
            height: '100%',
            padding: 0,
            margin: 0,
            maxWidth: 2000,
          }}
        >
          {node && node.providerId && initialCommands.length > 0 && (
            <TerminalClient
              tokenUrl={`${node.name.replaceAll('/', '')}`}
              wsUrl={`ws://${currentProvider.host()}:7681/ws`}
              initialCommands={initialCommands}
              width={width}
              name={`${node.name}`}
              invisibleTerminal={false}
            />
          )}
          {cmd && providerId && initialCommands.length > 0 && (
            <TerminalClient
              tokenUrl={`${cmd.replaceAll('/', ' ')}`}
              wsUrl={`ws://${currentProvider.host()}:7681/ws`}
              initialCommands={initialCommands}
              width={width}
              name={`${cmd.replaceAll('/', ' ')}`}
              invisibleTerminal={false}
            />
          )}
        </Box>
      )}

      {!node && type !== 'cmd' && (
        <Alert severity="info">
          <AlertTitle>Please select a node</AlertTitle>
        </Alert>
      )}

      {!currentProvider && (
        <Alert severity="info">
          <AlertTitle>{`Invalid provider: [${providerId}]`}</AlertTitle>
          Please report this bug.
        </Alert>
      )}
    </Box>
  );
}

SingleTerminalPanel.defaultProps = {
  node: null,
  screen: null,
  providerId: '',
  width: null,
  cmd: '',
};

SingleTerminalPanel.propTypes = {
  type: PropTypes.string.isRequired,
  providerId: PropTypes.string,
  node: PropTypes.instanceOf(RosNode),
  screen: PropTypes.string,
  width: PropTypes.number,
  cmd: PropTypes.string,
};

export default SingleTerminalPanel;
