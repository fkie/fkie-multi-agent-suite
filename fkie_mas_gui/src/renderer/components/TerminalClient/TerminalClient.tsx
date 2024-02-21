/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import { useContext } from 'react';

import { ITerminalOptions, ITheme } from 'xterm';
import { SettingsContext } from '../../context/SettingsContext';
import { ClientOptions, Xterm } from './terminal';

// TODO: Add parameter for this
const clientOptions = {
  rendererType: 'canvas', // "dom" | "canvas" | "webgl"
  disableLeaveAlert: true,
  disableResizeOverlay: true,
  titleFixed: '',
} as ClientOptions;

// see:
// https://github.com/xtermjs/xterm.js/blob/master/typings/xterm.d.ts
const termOptions = {
  cursorBlink: true,
  cursorStyle: 'block',
  fastScrollModifier: 'alt',
  // TODO Add settings for [scrollback]
  scrollback: 5000,
  windowsMode: false,
  rightClickSelectsWord: false,
  fontSize: 12,
  lineHeight: 1,
  fontFamily:
    'Menlo For Powerline,Consolas,Liberation Mono,Menlo,Courier,monospace',
  macOptionClickForcesSelection: true,
  macOptionIsMeta: true,
  theme: {
    foreground: '#d2d2d2',
    background: '#2b2b2b',
    cursor: '#adadad',
    black: '#000000',
    red: '#d81e00',
    green: '#5ea702',
    yellow: '#cfae00',
    blue: '#427ab3',
    magenta: '#89658e',
    cyan: '#00a7aa',
    white: '#dbded8',
    brightBlack: '#686a66',
    brightRed: '#f54235',
    brightGreen: '#99e343',
    brightYellow: '#fdeb61',
    brightBlue: '#84b0d8',
    brightMagenta: '#bc94b7',
    brightCyan: '#37e6e8',
    brightWhite: '#f1f1f0',
  } as ITheme,
} as ITerminalOptions;

interface ITerminalClient {
  initialCommands: string[];
  tokenUrl: string;
  wsUrl: string;
  name: string;
  invisibleTerminal: boolean;
  onIncomingData: (data: string) => void | null;
  onCtrlD: (wsUrl: string, tokenUrl: string) => void | null;
}

function TerminalClient(props: ITerminalClient) {
  const {
    initialCommands,
    tokenUrl,
    wsUrl,
    name,
    invisibleTerminal,
    onIncomingData,
    onCtrlD,
  } = props;
  const settingsCtx = useContext(SettingsContext);

  if (!tokenUrl || !initialCommands || !wsUrl) {
    return '';
  }

  termOptions.fontSize = settingsCtx.get('fontSizeTerminal');

  return (
    <Xterm
      key={`xterm-${wsUrl}-${tokenUrl}-${JSON.stringify(initialCommands)}`}
      id={`xterm-${wsUrl}-${tokenUrl}-${JSON.stringify(initialCommands)}`}
      wsUrl={wsUrl}
      tokenUrl={tokenUrl}
      clientOptions={clientOptions}
      termOptions={termOptions}
      initialCommands={initialCommands}
      name={name}
      invisibleTerminal={invisibleTerminal}
      onIncomingData={onIncomingData}
      onCtrlD={onCtrlD}
      fontSize={termOptions.fontSize ? termOptions.fontSize : 14}
      setFontsize={(size) => settingsCtx.set("fontSizeTerminal", size)}
    />
  );
}

export default TerminalClient;
