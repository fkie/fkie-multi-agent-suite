import { Alert, AlertTitle, Box } from "@mui/material";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import TerminalClient from "../../../components/TerminalClient/TerminalClient";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { RosNode } from "../../../models";
import { CmdType } from "../../../providers";
import { EVENT_CLOSE_COMPONENT, eventCloseComponent } from "../../../utils/events";

function SingleTerminalPanel({ id, type, providerId = "", nodeName = "", screen = "", width = null, cmd = "" }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [initialCommands, setInitialCommands] = useState([]);
  const [currentHost, setCurrentHost] = useState(null);
  const [lastScreenUsed, setLastScreenUsed] = useState("");
  const [tokenUrl, setTokenUrl] = useState(providerId);
  const [errorHighlighting, setErrorHighlighting] = useState(false);

  const initializeTerminal = useCallback(
    async (newScreen = null) => {
      // get current provider
      const provider = rosCtx.getProviderById(providerId);

      if (!provider) {
        setCurrentHost(null);
        return;
      }
      setCurrentHost(provider.host());
      let tkUrl = `${nodeName.replaceAll("/", "")}`;
      if (!tkUrl) {
        tkUrl = providerId;
      }
      setTokenUrl(tkUrl);
      const terminalCmd = await provider.cmdForType(type, nodeName, "", newScreen, cmd);
      if (terminalCmd.cmd) {
        setInitialCommands([`${terminalCmd.cmd}\r`]);
      }
      if (type === CmdType.SCREEN) {
        setLastScreenUsed(terminalCmd.screen);
      }
    },
    [cmd, nodeName, providerId, rosCtx, type]
  );

  const updateScreenName = useCallback(() => {
    // node changed, update the screen for the current node
    if (nodeName && type === CmdType.SCREEN) {
      rosCtx.mapProviderRosNodes.get(providerId)?.forEach((n) => {
        if (n.name === nodeName) {
          // TODO: How to handle multiple screens? For now just do this for nodes with a single screen.
          if (n.screens.length > 0) {
            setErrorHighlighting(false);
            if (n.screens[0] !== screen && n.screens[0] !== lastScreenUsed) {
              // screen changed, reload the component
              // [lastScreenUsed] prevents unnecessary reloads
              setInitialCommands(() => []);
              initializeTerminal(n.screens[0]);
            }
          } else if (n.screens.length === 0) {
            if (lastScreenUsed !== "") {
              // Open Log if no screen is available
              setInitialCommands(() => []);
              initializeTerminal("");
              setErrorHighlighting(true);
            }
          }
        }
      });
    }
  }, [initializeTerminal, lastScreenUsed, nodeName, providerId, rosCtx.mapProviderRosNodes, screen, type]);

  // load commands initially
  useEffect(() => {
    initializeTerminal(screen);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // update the terminal every time the node screen changes
  useEffect(() => {
    updateScreenName();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.mapProviderRosNodes]);

  const createTerminalView = useMemo(() => {
    return (
      <Box key={id} width="100%" height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
        {!nodeName && type !== CmdType.CMD && type !== CmdType.TERMINAL && (
          <Alert severity="info">
            <AlertTitle>Please select a node</AlertTitle>
          </Alert>
        )}

        {!currentHost && (
          <Alert severity="info">
            <AlertTitle>{`Invalid provider: [${providerId}]`}</AlertTitle>
            Please report this bug.
          </Alert>
        )}

        {currentHost && nodeName && initialCommands.length > 0 && (
          <TerminalClient
            key={`term-${id}`}
            tokenUrl={tokenUrl}
            wsUrl={`ws://${currentHost}:7681/ws`}
            initialCommands={initialCommands}
            width={width}
            name={`${nodeName}`}
            errorHighlighting={errorHighlighting}
            onCtrlD={() => {
              window.electronAPI?.closeTerminal(id);
              emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
            }}
          />
        )}
        {currentHost && cmd && initialCommands.length > 0 && (
          <TerminalClient
            key={`term-cmd-${id}`}
            tokenUrl={`${cmd.replaceAll("/", " ")}`}
            wsUrl={`ws://${currentHost}:7681/ws`}
            initialCommands={initialCommands}
            width={width}
            name={`${cmd.replaceAll("/", " ")}`}
            errorHighlighting={errorHighlighting}
            onCtrlD={() => {
              window.electronAPI?.closeTerminal(id);
              emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
            }}
          />
        )}
        {currentHost && type === CmdType.TERMINAL && (
          <TerminalClient
            key={`term-terminal-${id}`}
            tokenUrl={`${cmd.replaceAll("/", " ")}`}
            wsUrl={`ws://${currentHost}:7681/ws`}
            initialCommands={initialCommands}
            width={width}
            name={`bash`}
            errorHighlighting={errorHighlighting}
            onCtrlD={() => {
              window.electronAPI?.closeTerminal(id);
              emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
            }}
          />
        )}
      </Box>
    );
  }, [cmd, currentHost, id, initialCommands, nodeName, providerId, settingsCtx, tokenUrl, type, width]);

  return createTerminalView;
}

SingleTerminalPanel.propTypes = {
  id: PropTypes.string.isRequired,
  type: PropTypes.instanceOf(CmdType).isRequired,
  providerId: PropTypes.string,
  nodeName: PropTypes.string,
  screen: PropTypes.string,
  width: PropTypes.number,
  cmd: PropTypes.string,
};

export default SingleTerminalPanel;
