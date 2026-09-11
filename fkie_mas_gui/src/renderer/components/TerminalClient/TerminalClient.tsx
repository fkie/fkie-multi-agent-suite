/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import { ITerminalOptions, ITheme } from "@xterm/xterm";

import { useSetting } from "@/renderer/hooks/useSetting";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import Provider from "@/renderer/providers/Provider";
import { CmdType } from "@/types";
import { Box } from "@mui/material";
import { useMemo } from "react";
import { ClientOptions, Terminal } from "./Terminal";

// TODO: Add parameter for this
const clientOptions = {
  rendererType: "canvas", // "dom" | "canvas" | "webgl"
  disableLeaveAlert: true,
  disableResizeOverlay: true,
  titleFixed: "",
} as ClientOptions;

// see:
// https://github.com/xtermjs/xterm.js/blob/master/typings/xterm.d.ts
const baseTermOptions = {
  allowProposedApi: true,
  cursorBlink: true,
  cursorStyle: "block",
  fastScrollModifier: "alt",
  scrollback: 10000,
  scrollOnUserInput: true,
  scrollOnWindowFocus: true,
  smoothScrollDuration: 0,
  // width of the overview ruler used by the search addon decorations
  overviewRulerWidth: 10,
  // keep the scrollbar visible while the mouse is inside the terminal
  windowsMode: false,
  rightClickSelectsWord: false,
  fontSize: 12,
  lineHeight: 1,
  fontFamily: "Menlo For Powerline,Consolas,Liberation Mono,Menlo,Courier,monospace",
  macOptionClickForcesSelection: true,
  macOptionIsMeta: true,
} as ITerminalOptions;

function createTerminalTheme(errorHighlighting: boolean): ITheme {
  return {
    foreground: "#d2d2d2",
    background: errorHighlighting ? "#4d0400" : "#2b2b2b",
    cursor: "#adadad",
    // scrollbar slider, alpha values keep the text readable underneath
    scrollbarSliderBackground: "#ffffff26",
    scrollbarSliderHoverBackground: "#ffffff40",
    scrollbarSliderActiveBackground: "#ffffff59",
    overviewRulerBorder: "#00000000",
    black: "#000000",
    red: "#d81e00",
    green: "#5ea702",
    yellow: "#cfae00",
    blue: "#427ab3",
    magenta: "#89658e",
    cyan: "#00a7aa",
    white: "#dbded8",
    brightBlack: "#686a66",
    brightRed: "#f54235",
    brightGreen: "#99e343",
    brightYellow: "#fdeb61",
    brightBlue: "#84b0d8",
    brightMagenta: "#bc94b7",
    brightCyan: "#37e6e8",
    brightWhite: "#f1f1f0",
  } as ITheme;
}

interface ITerminalClient {
  type: CmdType;
  initialCommands: string[];
  tokenUrl: string;
  wsUrl: string;
  name: string;
  errorHighlighting: boolean;
  provider?: Provider;
  remoteProvider?: Provider;
  onIncomingData?: (data: string) => undefined;
  onCtrlD?: (wsUrl: string, tokenUrl: string) => undefined;
}

export default function TerminalClient(props: ITerminalClient): JSX.Element {
  const {
    type,
    initialCommands,
    tokenUrl,
    remoteProvider,
    wsUrl,
    name,
    errorHighlighting,
    provider,
    onIncomingData,
    onCtrlD,
  } = props;
  const settingsCtx = useSettingsContext();
  const [buttonLocation] = useSetting<string>("buttonLocation");
  const [fontSizeTerminal] = useSetting<number>("fontSizeTerminal");

  const termOptions = useMemo<ITerminalOptions>(
    () => ({ ...baseTermOptions, fontSize: fontSizeTerminal, theme: createTerminalTheme(errorHighlighting) }),
    [fontSizeTerminal, errorHighlighting]
  );

  return (
    <Box width="100%" flexGrow={1} minHeight={0} overflow="hidden">
      <Terminal
        key={`xterm-${wsUrl}-${tokenUrl}-${JSON.stringify(initialCommands)}`}
        id={`xterm-${wsUrl}-${tokenUrl}-${JSON.stringify(initialCommands)}`}
        type={type}
        wsUrl={wsUrl}
        tokenUrl={tokenUrl}
        clientOptions={clientOptions}
        termOptions={termOptions}
        initialCommands={initialCommands}
        name={name}
        onIncomingData={onIncomingData}
        onCtrlD={onCtrlD}
        settingsCtx={settingsCtx}
        provider={provider}
        remoteProvider={remoteProvider}
        buttonLocation={buttonLocation}
      />
    </Box>
  );
}
