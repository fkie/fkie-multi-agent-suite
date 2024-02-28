import { Alert, AlertTitle, Box } from '@mui/material';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';
import { emitCustomEvent } from 'react-custom-events';
import TerminalClient from '../../../components/TerminalClient/TerminalClient';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { RosNode } from '../../../models';
import { CmdType } from '../../../providers';
import {
  EVENT_CLOSE_COMPONENT,
  eventCloseComponent,
} from '../../../utils/events';

function SingleTerminalPanel({
  id,
  type,
  providerId,
  node,
  screen,
  width,
  cmd,
}) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [initialCommands, setInitialCommands] = useState([]);
  const [currentProvider, setCurrentProvider] = useState(null);
  const [lastScreenUsed, setLastScreenUsed] = useState('');
  const [tokenUrl, setTokenUrl] = useState(providerId);

  const initializeTerminal = useDebounceCallback(async (newScreen = null) => {
    // get current provider
    const provider = rosCtx.getProviderById(providerId);

    if (!provider) {
      setCurrentProvider(null);
      return;
    }

    let tkUrl = `${node.name.replaceAll('/', '')}`;
    if (!tkUrl) {
      tkUrl = providerId;
    }
    setTokenUrl(tkUrl);
    setCurrentProvider(() => provider);
    const terminalCmd = await provider.cmdForType(
      type,
      node?.name,
      '',
      newScreen,
      cmd,
    );
    setInitialCommands([`${terminalCmd.cmd}\r`]);
    if (type === CmdType.SCREEN) {
      setLastScreenUsed(terminalCmd.screen);
    }
  }, 200);

  // load commands initially
  useEffect(() => {
    initializeTerminal(screen);
  }, [screen, initializeTerminal]);

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
      key={id}
      width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      {currentProvider &&
        node &&
        node.providerId &&
        initialCommands.length > 0 && (
          <TerminalClient
            key={`term-${id}`}
            tokenUrl={tokenUrl}
            wsUrl={`ws://${currentProvider.host()}:7681/ws`}
            initialCommands={initialCommands}
            width={width}
            name={`${node.name}`}
            invisibleTerminal={false}
            onCtrlD={() =>
              emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id))
            }
          />
        )}
      {currentProvider && cmd && providerId && initialCommands.length > 0 && (
        <TerminalClient
          key={`term-cmd-${id}`}
          tokenUrl={`${cmd.replaceAll('/', ' ')}`}
          wsUrl={`ws://${currentProvider.host()}:7681/ws`}
          initialCommands={initialCommands}
          width={width}
          name={`${cmd.replaceAll('/', ' ')}`}
          invisibleTerminal={false}
          onCtrlD={() =>
            emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id))
          }
        />
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
  id: PropTypes.string.isRequired,
  type: PropTypes.instanceOf(CmdType).isRequired,
  providerId: PropTypes.string,
  node: PropTypes.instanceOf(RosNode),
  screen: PropTypes.string,
  width: PropTypes.number,
  cmd: PropTypes.string,
};

export default SingleTerminalPanel;
