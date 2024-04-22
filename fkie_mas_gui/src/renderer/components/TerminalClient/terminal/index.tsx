/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import React from 'react';

import { FitAddon } from '@xterm/addon-fit';
import { ISearchOptions, SearchAddon } from '@xterm/addon-search';
import { WebLinksAddon } from '@xterm/addon-web-links';
import { WebglAddon } from '@xterm/addon-webgl';
import { ITerminalOptions, Terminal } from '@xterm/xterm';
import { bind } from 'decko';

import {
  Alert,
  AlertTitle,
  Box,
  Button,
  IconButton,
  Stack,
  TextField,
} from '@mui/material';

import FirstPageIcon from '@mui/icons-material/FirstPage';
import LastPageIcon from '@mui/icons-material/LastPage';

import SearchBar from '../../UI/SearchBar';

import { FlowControl, ZmodemAddon } from '../zmodem';
import OverlayAddon from './overlay';

import '@xterm/xterm/css/xterm.css';

const enum Command {
  // server side
  OUTPUT = '0',
  SET_WINDOW_TITLE = '1',
  SET_PREFERENCES = '2',

  // client side
  INPUT = '0',
  RESIZE_TERMINAL = '1',
  PAUSE = '2',
  RESUME = '3',
}

export interface ClientOptions {
  rendererType: 'dom' | 'canvas' | 'webgl';
  disableLeaveAlert: boolean;
  disableResizeOverlay: boolean;
  titleFixed: string;
}

interface Props {
  id: string;
  wsUrl: string;
  tokenUrl: string;
  clientOptions: ClientOptions;
  termOptions: ITerminalOptions;
  initialCommands: string[];
  invisibleTerminal: boolean | null;
  name: string;
  onIncomingData: (data: string) => void | null;
  onCtrlD: (wsUrl: string, tokenUrl: string) => void | null;
  fontSize: number;
  setFontsize: (size: number) => void | null;
}

type XtermState = {
  opened: boolean;
};

export class Xterm extends React.Component<Props, XtermState> {
  private textEncoder: TextEncoder;

  private textDecoder: TextDecoder;

  private container: HTMLElement | null = null;

  private terminal: Terminal | null = null;

  private fitAddon: FitAddon;

  private searchAddon: SearchAddon;

  private searchAddonOptions: ISearchOptions;

  private overlayAddon: OverlayAddon | null = null;

  private zModemAddon: ZmodemAddon | null = null;

  private webglAddon: WebglAddon | null = null;

  private socket: WebSocket | null = null;

  private token: string | null = null;

  private title: string = 'Terminal';

  private searchText: string = '';

  private fontSize: number;

  private resizeObserver: ResizeObserver | null = null;

  private id: string = '';

  private lastContainerSize: { width: number; height: number } = {
    width: 0,
    height: 0,
  };

  constructor(props: Props) {
    super(props);

    this.state = {
      opened: false,
    };

    this.id = props.id;
    this.textEncoder = new TextEncoder();
    this.textDecoder = new TextDecoder();
    this.fitAddon = new FitAddon();
    this.overlayAddon = new OverlayAddon();

    this.searchAddon = new SearchAddon();
    this.searchAddonOptions = {
      regex: true,
      caseSensitive: false,

      // TODO: Check why decorations does not work
      decorations: {
        matchBackground: '#797D7F',
        activeMatchBackground: '#d81e00',
        matchBorder: '#d81e00',
        matchOverviewRuler: '#d81e00',
        activeMatchColorOverviewRuler: '#d81e00',
      },
    };

    // TODO Add setting for this parameter
    this.fontSize = props.fontSize;
  }

  async componentDidMount() {
    // await this.refreshToken();
    this.openTerminal();
    this.connect();

    // show the name of the node during 300 ms at launch time
    const { overlayAddon } = this;
    const { name } = this.props;
    overlayAddon?.showOverlay(name ? name : 'empty name', 300);
    if (this.container) {
      this.resizeObserver = new ResizeObserver(() => {
        this.componentDidUpdate();
        const rect = this.container?.getBoundingClientRect();
        if (
          rect &&
          rect.width > 0 &&
          (this.lastContainerSize.width !== rect.width ||
            this.lastContainerSize.height !== rect.height)
        ) {
          this.lastContainerSize = { width: rect.width, height: rect.height };
          const { fitAddon } = this;
          fitAddon.fit();
        }
      });
      this.resizeObserver.observe(this.container);
    }
  }

  componentWillUnmount() {
    // send SIGINT on terminal close
    this.socket?.close(1000, 'Component closed');

    if (this.terminal) this.terminal.dispose();
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
  }

  componentDidUpdate() {
    const { tokenUrl } = this.props;
    if (
      this.token !== tokenUrl ||
      this.socket?.readyState === WebSocket.CLOSED
    ) {
      this.token = tokenUrl;

      this.socket?.close();
      this.connect();
    }
  }

  @bind
  private onWindowUnload(event: BeforeUnloadEvent): string {
    const { socket } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      const message = 'Close terminal? this will also terminate the command.';
      event.returnValue = message;
      return message;
    }
    event.preventDefault();
    return '';
  }

  @bind
  private onSocketOpen() {
    const { socket, textEncoder, terminal, fitAddon, overlayAddon, state } =
      this;
    const dims = fitAddon.proposeDimensions();
    this.setState({
      opened: true,
    });

    if (dims && socket) {
      socket.send(
        textEncoder.encode(
          JSON.stringify({
            AuthToken: this.token,
            columns: dims.cols,
            rows: dims.rows,
          }),
        ),
      );

      if (state.opened && terminal) {
        terminal.reset();
        terminal.resize(dims.cols, dims.rows);
        overlayAddon?.showOverlay('Reconnected', 300);
      }
    }

    // clear terminal at start
    // this.onTerminalData('cd && clear \r');

    // send initial commands to terminal
    const { initialCommands } = this.props;
    if (initialCommands) {
      initialCommands.forEach((command) => {
        this.onTerminalData(command);
      });
    }

    if (terminal) terminal.focus();
  }

  // // @bind
  // private onSocketClose(event: CloseEvent) {
  //   // console.debug(websocket connection closed`[ttyd] websocket connection closed with code: ${event.code}`);
  //   // Do we need this?
  //   // const { refreshToken, connect, doReconnect, overlayAddon } = this;
  //   // overlayAddon.showOverlay('Connection Closed', null);
  //   // // 1000: CLOSE_NORMAL
  //   // if (event.code !== 1000 && doReconnect) {
  //   //   overlayAddon.showOverlay('Reconnecting...', null);
  //   //   refreshToken().then(connect);
  //   // } else {
  //   //   const { terminal } = this;
  //   //   const keyDispose = terminal.onKey((e) => {
  //   //     const event = e.domEvent;
  //   //     if (event.key === 'Enter') {
  //   //       keyDispose.dispose();
  //   //       overlayAddon.showOverlay('Reconnecting...', null);
  //   //       refreshToken().then(connect);
  //   //     }
  //   //   });
  //   //   overlayAddon.showOverlay('Press ⏎ to Reconnect', null);
  //   // }
  // }

  @bind
  private onSocketError(_event: Event) {
    // might be fired when component is closed
    // console.error('[ttyd] websocket connection error: ', _event);
  }

  @bind
  private onSocketData(event: MessageEvent) {
    const { textDecoder, zModemAddon } = this;
    const rawData = event.data as ArrayBuffer;
    const cmd = String.fromCharCode(new Uint8Array(rawData)[0]);
    const data = rawData.slice(1);

    const { onIncomingData } = this.props;
    if (onIncomingData) onIncomingData(textDecoder.decode(data));

    switch (cmd) {
      case Command.OUTPUT:
        zModemAddon?.consume(data);
        break;
      case Command.SET_WINDOW_TITLE:
        this.title = textDecoder.decode(data);
        document.title = this.title;
        break;
      case Command.SET_PREFERENCES: {
        const preferences = JSON.parse(textDecoder.decode(data));
        const { clientOptions } = this.props;
        this.applyOptions({ ...clientOptions, ...preferences });
        break;
      }

      default:
        console.warn(`[ttyd] unknown command: ${cmd}`);
        break;
    }
  }

  @bind
  private setRendererType(value: 'webgl') {
    const { terminal } = this;

    const disposeWebglRenderer = () => {
      try {
        this.webglAddon?.dispose();
      } catch {
        // ignore
      }
      this.webglAddon = null;
    };

    switch (value) {
      case 'webgl':
        if (this.webglAddon) return;
        try {
          if (
            window.WebGL2RenderingContext &&
            document.createElement('canvas').getContext('webgl2')
          ) {
            this.webglAddon = new WebglAddon();
            this.webglAddon.onContextLoss(() => {
              disposeWebglRenderer();
            });

            if (terminal) terminal.loadAddon(this.webglAddon);
            // console.debug(`[ttyd] WebGL renderer enabled`);
          }
        } catch (e) {
          console.warn(`[ttyd] webgl2 init error`, e);
        }
        break;
      default:
        disposeWebglRenderer();
        // if (terminal) terminal.options.rendererType = value;
        break;
    }
  }

  @bind
  private openTerminal() {
    const { termOptions } = this.props;
    termOptions.allowProposedApi = true;
    this.terminal = new Terminal(termOptions);

    const { terminal, container, fitAddon, searchAddon, overlayAddon } = this;

    terminal.loadAddon(fitAddon);
    terminal.loadAddon(searchAddon);
    if (overlayAddon) terminal.loadAddon(overlayAddon);
    terminal.loadAddon(new WebLinksAddon());
    if (this.zModemAddon) terminal.loadAddon(this.zModemAddon);
    terminal.onData(this.onTerminalData);
    terminal.onResize(this.onTerminalResize);
    if (container) terminal.open(container);
    fitAddon.fit();
  }

  render() {
    const control = {
      limit: 100000,
      highWater: 10,
      lowWater: 4,
      pause: () => this.pause(),
      resume: () => this.resume(),
    } as FlowControl;

    const { invisibleTerminal } = this.props;

    // create an "invisible" terminal (visibility: 'hidden')
    // used for sending commands to a provider host
    // in combination with [onIncomingData] callback
    if (invisibleTerminal) {
      return (
        <div
          ref={(c) => {
            this.container = c;
            return this.container;
          }}
          style={{ visibility: 'hidden', height: 0 }}
        >
          <ZmodemAddon
            key={`zmodem-${this.id}`}
            id={`zmodem-${this.id}`}
            ref={(c) => {
              this.zModemAddon = c;
              return this.zModemAddon;
            }}
            sender={this.sendData}
            control={control}
          />
        </div>
      );
    }
    const { state, fitAddon } = this;
    fitAddon.fit();

    return (
      <Stack width="100%" height="100%" alignItems="center">
        {!state.opened && (
          <Alert severity="info">
            <AlertTitle>TTYD Daemon is not available</AlertTitle>
            If you want to check this terminal, please start the terminal
            manager daemon on the host.
          </Alert>
        )}

        {state.opened && (
          <Stack
            spacing={0.3}
            padding={0.5}
            width="100%"
            direction="row"
            alignContent="center"
            justifyItems="center"
            alignItems="center"
          >
            <SearchBar
              onSearch={(value: string) => {
                const { searchAddon, searchAddonOptions } = this;
                fitAddon.fit();
                this.searchText = value;
                searchAddon.findNext(value, searchAddonOptions);
              }}
              placeholder="Search Text (Supports Regular Expressions)"
              defaultValue={this.searchText}
              fullWidth
            />

            <IconButton
              size="small"
              // label="Previous"
              // leaveDelayMs={100}
              onClick={() => {
                const { searchAddon, searchAddonOptions } = this;
                fitAddon.fit();
                searchAddon.findPrevious(this.searchText, searchAddonOptions);
              }}
            >
              <FirstPageIcon />
            </IconButton>

            <IconButton
              size="small"
              // label="Next"
              // leaveDelayMs={100}
              onClick={() => {
                const { searchAddon, searchAddonOptions } = this;
                fitAddon.fit();
                searchAddon.findNext(this.searchText, searchAddonOptions);
              }}
            >
              <LastPageIcon />
            </IconButton>

            <Box sx={{ width: 100 }}>
              <TextField
                type="number"
                id="ni_font_size"
                size="small"
                variant="standard"
                defaultValue={this.fontSize}
                placeholder="Font size"
                InputProps={{
                  inputProps: {
                    min: 8,
                    max: 18,
                    style: { textAlign: 'center' },
                  },
                }}
                fullWidth
                onChange={(event) => {
                  this.fontSize = Number(event.target.value);
                  const { terminal } = this;
                  if (terminal) terminal.options.fontSize = this.fontSize;
                  const { setFontsize } = this.props;
                  if (setFontsize) setFontsize(this.fontSize);
                  fitAddon.fit();
                }}
              />
            </Box>

            <Button
              size="small"
              color="secondary"
              variant="text"
              onClick={() => {
                const { terminal } = this;
                if (terminal) terminal.clear();
              }}
            >
              Clear
            </Button>
          </Stack>
        )}

        <Box
          ref={(c) => {
            this.container = c as HTMLElement;
            return this.container;
          }}
          width="100%"
          height={state.opened ? '100%' : 0}
          visibility={state.opened ? 'visible' : 'hidden'}
          overflow="auto"
        >
          <ZmodemAddon
            key={`zmodem-${this.id}`}
            id={`zmodem-${this.id}`}
            ref={(c) => {
              this.zModemAddon = c;
              return this.zModemAddon;
            }}
            sender={this.sendData}
            control={control}
          />
        </Box>
      </Stack>
    );
  }

  // @bind
  // private async refreshToken() {
  //   const { tokenUrl } = this.props;

  //   try {
  //     const resp = await fetch(tokenUrl, { mode: 'no-cors' });
  //     if (resp.ok) {
  //       const json = await resp.json();
  //       this.token = json.token;
  //     }
  //   } catch (e) {
  //     console.error(`[ttyd] fetch ${tokenUrl}: `, e);
  //   }
  // }

  @bind
  private connect() {
    const { wsUrl } = this.props;
    this.socket = new WebSocket(wsUrl, ['tty']);

    this.socket.binaryType = 'arraybuffer';
    this.socket.onopen = this.onSocketOpen;
    this.socket.onmessage = this.onSocketData;
    // this.socket.onclose = this.onSocketClose;
    this.socket.onerror = this.onSocketError;
  }

  @bind
  private pause() {
    const { textEncoder, socket } = this;

    socket?.send(textEncoder.encode(Command.PAUSE));
  }

  @bind
  private applyOptions(options: any) {
    const { fitAddon } = this;

    Object.keys(options).forEach((key) => {
      const value = options[key];
      switch (key) {
        case 'rendererType':
          this.setRendererType(value);
          break;
        case 'disableLeaveAlert':
          if (value) {
            window.removeEventListener('beforeunload', this.onWindowUnload);
          }
          break;
        case 'disableResizeOverlay':
          if (value) {
            console.debug(`[ttyd] Resize overlay disabled`);
          }
          break;
        case 'disableReconnect':
          if (value) {
            console.debug(`[ttyd] Reconnect disabled`);
          }
          break;
        // case 'titleFixed':
        //   if (!value || value === '') return;
        //   console.debug(`[ttyd] setting fixed title: ${value}`);
        //   this.titleFixed = value;
        //   document.title = value;
        //   break;
        default:
          console.debug(`[ttyd] option: ${key}=${JSON.stringify(value)}`);
          // TODO: Check if this is required
          // if (!terminal) break;

          // if (terminal.options[key] instanceof Object) {
          //   terminal.options[key] = Object.assign(
          //     {},
          //     terminal.options[key],
          //     value
          //   );
          // } else {
          //   terminal.options[key] = value;
          // }

          if (key.indexOf('font') === 0) fitAddon.fit();

          break;
      }
    });
  }

  @bind
  private onTerminalResize(size: { cols: number; rows: number }) {
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ columns: size.cols, rows: size.rows });
      socket.send(textEncoder.encode(Command.RESIZE_TERMINAL + msg));
    }
  }

  @bind
  private onTerminalData(data: string) {
    if (data?.charCodeAt(0) === 4) {
      const { wsUrl, tokenUrl, onCtrlD } = this.props;
      if (onCtrlD) onCtrlD(wsUrl, tokenUrl);
    }
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(Command.INPUT + data));
    }
  }

  @bind
  private resume() {
    const { textEncoder, socket } = this;
    socket?.send(textEncoder.encode(Command.RESUME));
  }

  @bind
  private sendData(data: ArrayLike<number>) {
    const { socket } = this;
    const payload = new Uint8Array(data.length + 1);
    payload[0] = Command.INPUT.charCodeAt(0);
    payload.set(data, 1);
    socket?.send(payload);
  }
}
