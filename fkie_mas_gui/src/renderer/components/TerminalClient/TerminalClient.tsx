/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import { ITerminalOptions, ITheme } from "@xterm/xterm";
import { useContext } from "react";

import { SettingsContext } from "@/renderer/context/SettingsContext";
import { CmdType } from "@/renderer/providers";
import Provider from "@/renderer/providers/Provider";
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
const termOptions = {
  allowProposedApi: true,
  cursorBlink: true,
  cursorStyle: "block",
  fastScrollModifier: "alt",
  // TODO Add settings for [scrollback]
  scrollback: 5000,
  windowsMode: false,
  rightClickSelectsWord: false,
  fontSize: 12,
  lineHeight: 1,
  fontFamily: "Menlo For Powerline,Consolas,Liberation Mono,Menlo,Courier,monospace",
  macOptionClickForcesSelection: true,
  macOptionIsMeta: true,
  theme: {
    foreground: "#d2d2d2",
    background: "#2b2b2b",
    cursor: "#adadad",
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
  } as ITheme,
} as ITerminalOptions;

interface ITerminalClient {
  type: CmdType;
  initialCommands: string[];
  tokenUrl: string;
  wsUrl: string;
  name: string;
  errorHighlighting: boolean;
  provider?: Provider;
  onIncomingData?: (data: string) => undefined;
  onCtrlD?: (wsUrl: string, tokenUrl: string) => undefined;
}

export default function TerminalClient(props: ITerminalClient): JSX.Element {
  const { type, initialCommands, tokenUrl, wsUrl, name, errorHighlighting, provider, onIncomingData, onCtrlD } = props;
  const settingsCtx = useContext(SettingsContext);

  termOptions.fontSize = settingsCtx.get("fontSizeTerminal") as number;
  termOptions.theme = {
    foreground: "#d2d2d2",
    background: errorHighlighting ? "#4d0400" : "#2b2b2b",
    cursor: "#adadad",
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

  return (
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
    />
  );
}
