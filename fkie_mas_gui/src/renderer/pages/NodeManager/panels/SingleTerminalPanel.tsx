import { Alert, AlertTitle, Box } from "@mui/material";
import { useCallback, useEffect, useMemo, useState } from "react";

import TerminalClient from "@/renderer/components/TerminalClient/TerminalClient";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import Provider from "@/renderer/providers/Provider";
import { EVENT_PROVIDER_STATE } from "@/renderer/providers/eventTypes";
import { ConnectionState, EventProviderState } from "@/renderer/providers/events";
import { CmdType, CmdTypes, TEnvEntry } from "@/types";
import { useCustomEventListener } from "react-custom-events";
import { emitCloseComponent } from "../../../components/layout/events";

interface SingleTerminalPanelProps {
  id: string;
  type: CmdType;
  provider: Provider;
  nodeName?: string;
  screen?: string;
  cmd?: string;
  env?: TEnvEntry[];
}

export default function SingleTerminalPanel(props: SingleTerminalPanelProps): JSX.Element {
  const { id, type, provider, nodeName = "", screen = "", cmd = "", env = [] } = props;

  const rosCtx = useRosContext();
  const [initialCommands, setInitialCommands] = useState<string[]>([]);
  const [providerId, setProviderId] = useState("");
  const [currentHost, setCurrentHost] = useState<string>();
  const [ttydPort, setTtydPort] = useState<number>(8681);
  const [lastScreenUsed, setLastScreenUsed] = useState("");
  const [tokenUrl, setTokenUrl] = useState(provider.id);
  const [errorHighlighting, setErrorHighlighting] = useState(false);
  const [colorizeHosts] = useSetting<boolean>("colorizeHosts");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [error, setError] = useState<string | undefined>();

  const initializeTerminal = useCallback(
    async (newScreen = "") => {
      if (!provider) {
        setCurrentHost(undefined);
        return;
      }
      setProviderId(provider.id);
      setCurrentHost(provider.host());

      let tkUrl = `${nodeName.replaceAll("/", "")}`;
      if (!tkUrl) tkUrl = provider.id;
      setTokenUrl(tkUrl);

      const terminalCmd = await provider.cmdForType(type, nodeName, "", newScreen, cmd, env);
      if (!terminalCmd.success) {
        setError(terminalCmd.error);
      }
      if (type !== CmdTypes.SET_TIME && terminalCmd.cmd) {
        setInitialCommands([`${terminalCmd.cmd}\r`]);
      }
      if (type === CmdTypes.SCREEN) {
        setLastScreenUsed(terminalCmd.screen);
        // no screen available -> log is shown instead: highlight errors
        setErrorHighlighting(!terminalCmd.screen);
      }
    },
    [cmd, nodeName, provider, type, env]
  );

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    if (data.provider.id === provider.id && data.newState === ConnectionState.STATES.CONNECTED) {
      setError("");
      initializeTerminal(screen);
    }
  });

  const updateScreenName = useCallback(() => {
    // node changed, update the screen for the current node
    if (nodeName && type === CmdTypes.SCREEN) {
      const nodes = rosCtx.mapProviderRosNodes.get(provider.id);
      const screens: string[] = [];
      if (nodes) {
        for (const n of nodes) {
          if (n.name === nodeName && n.screens) {
            screens.push(...n.screens);
          }
        }
      }
      if (!screens.includes(screen) && !screens.includes(lastScreenUsed)) {
        if (screens.length > 0) {
          // screen changed, reload the component
          // [lastScreenUsed] prevents unnecessary reloads
          setInitialCommands(() => []);
          initializeTerminal(screens[0]);
        } else if (lastScreenUsed) {
          // Open Log if no screen is available
          setInitialCommands(() => []);
          initializeTerminal("");
        }
      }
    }
  }, [initializeTerminal, lastScreenUsed, nodeName, provider, rosCtx.mapProviderRosNodes, screen, type]);

  const updateTTYDPort = useCallback(() => {
    const ttydNodes = rosCtx.mapProviderRosNodes.get(provider.id)?.filter((n) => {
      return n.name.startsWith("/ttyd-");
    });
    if (ttydNodes && ttydNodes?.length > 0) {
      const splits = ttydNodes[0].name.split("-");
      if (splits && splits.length > 1) {
        setTtydPort(Number.parseInt(splits[1]));
      }
    }
  }, [provider, rosCtx.mapProviderRosNodes]);

  // load commands initially
  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    initializeTerminal(screen);
  }, []);

  // update the terminal every time the node screen changes
  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    updateScreenName();
    updateTTYDPort();
  }, [rosCtx.mapProviderRosNodes]);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const getHostStyle = useCallback(
    function getHostStyle(): object {
      const base = {
        flexGrow: 1,
        minHeight: 0,
        // border must not add to height: 100%
        boxSizing: "border-box",
        backgroundColor: backgroundColor,
      };
      if (providerId && colorizeHosts) {
        return {
          ...base,
          borderTopStyle: "solid",
          borderTopColor: rosCtx.providerColor(providerId),
          borderTopWidth: "0.3em",
        };
      }
      return base;
    },
    [providerId, backgroundColor, colorizeHosts]
  );

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const createTerminalView = useMemo(() => {
    return (
      <Box
        key={id}
        width="100%"
        height="100%"
        display="flex"
        flexDirection="column"
        // scrolling is owned by xterm's viewport, an outer scrollbar would not match the buffer
        overflow="hidden"
        minHeight={0}
        sx={getHostStyle()}
      >
        {!nodeName && type !== CmdTypes.CMD && type !== CmdTypes.TERMINAL && (
          <Alert severity="info">
            <AlertTitle>Please select a node</AlertTitle>
          </Alert>
        )}
        {error && (
          <Alert severity="error">
            <AlertTitle>{error}</AlertTitle>
          </Alert>
        )}

        {!currentHost && <Alert severity="info">Wait until the provider is initialized: [{provider.id}]</Alert>}

        {currentHost && nodeName && initialCommands.length > 0 && type !== CmdTypes.CMD && (
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
              emitCloseComponent({ id: id });
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
              emitCloseComponent({ id: id });
            }}
          />
        )}
        {currentHost && type === CmdTypes.TERMINAL && (
          <TerminalClient
            key={`term-terminal-${id}`}
            tokenUrl={`${cmd.replaceAll("/", " ")}`}
            wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
            type={type}
            initialCommands={initialCommands}
            name={"bash"}
            errorHighlighting={errorHighlighting}
            onCtrlD={() => {
              window.terminalManager?.close(id);
              emitCloseComponent({ id: id });
            }}
          />
        )}
        {currentHost && type === CmdTypes.SET_TIME && (
          <TerminalClient
            key={`set-time-${id}`}
            type={type}
            tokenUrl={tokenUrl}
            provider={rosCtx.getProviderById(cmd)}
            remoteProvider={rosCtx.getProviderById(provider.id)}
            wsUrl={`ws://${currentHost}:${ttydPort}/ws`}
            initialCommands={initialCommands}
            name={"bash"}
            errorHighlighting={errorHighlighting}
            onCtrlD={() => {
              window.terminalManager?.close(id);
              emitCloseComponent({ id: id });
            }}
          />
        )}
      </Box>
    );
  }, [error, cmd, currentHost, id, initialCommands, nodeName, provider, tokenUrl, type, ttydPort, errorHighlighting]);

  return createTerminalView;
}
