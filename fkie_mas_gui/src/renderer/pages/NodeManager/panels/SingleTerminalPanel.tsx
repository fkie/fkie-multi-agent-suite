import { Alert, AlertTitle, Box } from "@mui/material";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import TerminalClient from "@/renderer/components/TerminalClient/TerminalClient";
import { colorFromHostname } from "@/renderer/components/UI/Colors";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import CmdType from "@/renderer/providers/CmdType";
import Provider from "@/renderer/providers/Provider";
import { EVENT_CLOSE_COMPONENT, eventCloseComponent } from "../layout/events";

interface SingleTerminalPanelProps {
  id: string;
  type: CmdType;
  providerId: string;
  nodeName?: string;
  screen?: string;
  cmd?: string;
}

const SingleTerminalPanel = forwardRef<HTMLDivElement, SingleTerminalPanelProps>(
  function SingleTerminalPanel(props, ref) {
    const { id, type, providerId = "", nodeName = "", screen = "", cmd = "" } = props;

    const rosCtx = useContext(RosContext);
    const settingsCtx = useContext(SettingsContext);
    const [initialCommands, setInitialCommands] = useState<string[]>([]);
    const [providerName, setProviderName] = useState("");
    const [currentHost, setCurrentHost] = useState<string>();
    const [ttydPort, setTtydPort] = useState<number>(8681);
    const [lastScreenUsed, setLastScreenUsed] = useState("");
    const [tokenUrl, setTokenUrl] = useState(providerId);
    const [errorHighlighting, setErrorHighlighting] = useState(false);
    const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

    useEffect(() => {
      setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    }, [settingsCtx.changed]);

    const initializeTerminal = useCallback(
      async (newScreen: string = "") => {
        // get current provider
        const provider: Provider | undefined = rosCtx.getProviderById(providerId);

        if (!provider) {
          setCurrentHost(undefined);
          return;
        }
        setProviderName(provider.name());
        setCurrentHost(provider.host());
        let tkUrl = `${nodeName.replaceAll("/", "")}`;
        if (!tkUrl) {
          tkUrl = providerId;
        }
        setTokenUrl(tkUrl);
        const terminalCmd = await provider.cmdForType(type, nodeName, "", newScreen, cmd);
        if (type !== CmdType.SET_TIME && terminalCmd.cmd) {
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
            if (n.screens && n.screens.length > 0) {
              setErrorHighlighting(false);
              if (n.screens[0] !== screen && n.screens[0] !== lastScreenUsed) {
                // screen changed, reload the component
                // [lastScreenUsed] prevents unnecessary reloads
                setInitialCommands(() => []);
                initializeTerminal(n.screens[0]);
              }
            } else if (!n.screens || n.screens.length === 0) {
              // Open Log if no screen is available
              setInitialCommands(() => []);
              initializeTerminal("");
              setErrorHighlighting(true);
            }
          }
        });
      }
    }, [initializeTerminal, lastScreenUsed, nodeName, providerId, rosCtx.mapProviderRosNodes, screen, type]);

    const updateTTYDPort = useCallback(() => {
      const ttydNodes = rosCtx.mapProviderRosNodes.get(providerId)?.filter((n) => {
        return n.name.startsWith("/ttyd-");
      });
      if (ttydNodes && ttydNodes?.length > 0) {
        const splits = ttydNodes[0].name.split("-");
        if (splits && splits.length > 1) {
          setTtydPort(parseInt(splits[1]));
        }
      }
    }, [rosCtx.mapProviderRosNodes]);

    // load commands initially
    useEffect(() => {
      initializeTerminal(screen);
    }, []);

    // update the terminal every time the node screen changes
    useEffect(() => {
      updateScreenName();
      updateTTYDPort();
    }, [rosCtx.mapProviderRosNodes]);

    const getHostStyle = useCallback(
      function getHostStyle(): object {
        if (providerName && settingsCtx.get("colorizeHosts")) {
          return {
            flexGrow: 1,
            borderTopStyle: "solid",
            borderTopColor: colorFromHostname(providerName),
            borderTopWidth: "0.3em",
            backgroundColor: backgroundColor,
          };
        }
        return { flexGrow: 1, backgroundColor: backgroundColor };
      },
      [providerName, settingsCtx.changed]
    );

    const createTerminalView = useMemo(() => {
      return (
        <Box ref={ref} key={id} width="100%" height="100%" overflow="auto" alignItems={"center"} sx={getHostStyle()}>
          {!nodeName && type !== CmdType.CMD && type !== CmdType.TERMINAL && (
            <Alert severity="info">
              <AlertTitle>Please select a node</AlertTitle>
            </Alert>
          )}

          {!currentHost && <Alert severity="info">Wait until the provider is initialized: [${providerId}]</Alert>}

          {currentHost && nodeName && initialCommands.length > 0 && type !== CmdType.CMD && (
            <TerminalClient
              key={`term-${id}`}
              tokenUrl={tokenUrl}
              wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
              type={type}
              initialCommands={initialCommands}
              name={`${nodeName}`}
              errorHighlighting={errorHighlighting}
              onCtrlD={() => {
                window.terminalManager?.close(id);
                emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
              }}
            />
          )}
          {currentHost && cmd && initialCommands.length > 0 && (
            <TerminalClient
              key={`term-cmd-${id}`}
              tokenUrl={`${cmd.replaceAll("/", " ")}`}
              wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
              type={type}
              initialCommands={initialCommands}
              name={`${cmd.replaceAll("/", " ")}`}
              errorHighlighting={errorHighlighting}
              onCtrlD={() => {
                window.terminalManager?.close(id);
                emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
              }}
            />
          )}
          {currentHost && type === CmdType.TERMINAL && (
            <TerminalClient
              key={`term-terminal-${id}`}
              tokenUrl={`${cmd.replaceAll("/", " ")}`}
              wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
              type={type}
              initialCommands={initialCommands}
              name={`bash`}
              errorHighlighting={errorHighlighting}
              onCtrlD={() => {
                window.terminalManager?.close(id);
                emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
              }}
            />
          )}
          {currentHost && type === CmdType.SET_TIME && (
            <TerminalClient
              key={`set-time-${id}`}
              type={type}
              tokenUrl={tokenUrl}
              provider={rosCtx.getProviderById(cmd)}
              wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
              initialCommands={initialCommands}
              name={`bash`}
              errorHighlighting={errorHighlighting}
              onCtrlD={() => {
                window.terminalManager?.close(id);
                emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
              }}
            />
          )}
        </Box>
      );
    }, [cmd, currentHost, id, initialCommands, nodeName, providerId, settingsCtx, tokenUrl, type, ttydPort]);

    return createTerminalView;
  }
);

export default SingleTerminalPanel;
