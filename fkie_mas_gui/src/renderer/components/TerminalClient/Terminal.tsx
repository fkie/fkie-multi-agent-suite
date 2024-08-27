/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import FirstPageIcon from "@mui/icons-material/FirstPage";
import LastPageIcon from "@mui/icons-material/LastPage";
import { Alert, AlertTitle, Box, Button, IconButton, Stack, TextField } from "@mui/material";
import { FitAddon } from "@xterm/addon-fit";
import { ISearchOptions, SearchAddon } from "@xterm/addon-search";
import { Unicode11Addon } from "@xterm/addon-unicode11";
import { WebLinksAddon } from "@xterm/addon-web-links";
import { WebglAddon } from "@xterm/addon-webgl";
import { ITerminalOptions, Terminal as XTerminal } from "@xterm/xterm";
import "@xterm/xterm/css/xterm.css";
import React from "react";
import SearchBar from "../UI/SearchBar";

const enum Command {
  // server side
  OUTPUT = "0",
  SET_WINDOW_TITLE = "1",
  SET_PREFERENCES = "2",

  // client side
  INPUT = "0",
  RESIZE_TERMINAL = "1",
  PAUSE = "2",
  RESUME = "3",
}

export interface ClientOptions {
  rendererType: "dom" | "canvas" | "webgl";
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
  name: string;
  onIncomingData: (data: string) => void | null;
  onCtrlD: (wsUrl: string, tokenUrl: string) => void | null;
  fontSize: number;
  setFontsize: (size: number) => void | null;
}

type XtermState = {
  opened: boolean;
};

export class Terminal extends React.Component<Props, XtermState> {
  private textEncoder: TextEncoder;

  private textDecoder: TextDecoder;

  private container: HTMLElement | null = null;

  private terminal: XTerminal | null = null;

  private fitAddon: FitAddon;

  private searchAddon: SearchAddon = new SearchAddon();

  private searchAddonOptions: ISearchOptions;

  private webglAddon: WebglAddon = new WebglAddon();

  private unicode11Addon = new Unicode11Addon();

  private socket: WebSocket | null = null;

  private token: string | null = null;

  // private title: string = "Terminal";

  private searchText: string = "";

  private fontSize: number;

  private resizeObserver: ResizeObserver | null = null;

  private gotFocus: boolean = false;

  private lastContainerSize: { width: number; height: number } = {
    width: 0,
    height: 0,
  };

  constructor(props: Props) {
    super(props);

    this.state = {
      opened: false,
    };
    this.textEncoder = new TextEncoder();
    this.textDecoder = new TextDecoder();
    this.fitAddon = new FitAddon();

    this.searchAddonOptions = {
      regex: true,
      caseSensitive: false,

      // decoration
      decorations: {
        matchBackground: "#797D7F",
        activeMatchBackground: "#d81e00",
        matchBorder: "#d81e00",
        matchOverviewRuler: "#d81e00",
        activeMatchColorOverviewRuler: "#d81e00",
      },
    };

    // TODO Add setting for this parameter
    this.fontSize = props.fontSize;
    this.onSocketOpen = this.onSocketOpen.bind(this);
    this.onSocketError = this.onSocketError.bind(this);
    this.onSocketData = this.onSocketData.bind(this);
    this.connect = this.connect.bind(this);
    this.pause = this.pause.bind(this);
    this.onTerminalResize = this.onTerminalResize.bind(this);
    this.onTerminalData = this.onTerminalData.bind(this);
    this.resume = this.resume.bind(this);
    // this.sendData = this.sendData.bind(this);
  }

  private connect() {
    console.log(`[ttyd] connect to ${this.props.wsUrl}`);
    this.socket = new WebSocket(this.props.wsUrl, ["tty"]);
    this.socket.binaryType = "arraybuffer";
    this.socket.onopen = this.onSocketOpen;
    this.socket.onmessage = this.onSocketData;
    // this.socket.onclose = this.onSocketClose;
    this.socket.onerror = this.onSocketError;
  }

  async componentDidMount() {
    const { termOptions } = this.props;
    termOptions.allowProposedApi = true;
    this.terminal = new XTerminal(termOptions);

    const { terminal, fitAddon, searchAddon, webglAddon, unicode11Addon } = this;

    terminal.loadAddon(fitAddon);
    terminal.loadAddon(searchAddon);
    terminal.loadAddon(new WebLinksAddon());
    terminal.loadAddon(unicode11Addon);
    webglAddon.onContextLoss((_event) => {
      webglAddon.dispose();
    });
    terminal.loadAddon(webglAddon);
    terminal.onData(this.onTerminalData);
    terminal.onResize(this.onTerminalResize);
    this.connect();
    if (this.container) {
      terminal.open(this.container);
    }
    fitAddon.fit();

    // show the name of the node during 300 ms at launch time
    if (this.container) {
      this.resizeObserver = new ResizeObserver(() => {
        const rect = this.container?.getBoundingClientRect();
        if (
          rect &&
          rect.width > 0 &&
          (this.lastContainerSize.width !== rect.width || this.lastContainerSize.height !== rect.height)
        ) {
          this.lastContainerSize = { width: rect.width, height: rect.height };
          const { fitAddon } = this;
          fitAddon.fit();
        }
      });
      this.resizeObserver.observe(this.container);
    }

    const keyMap = [
      {
        key: "C",
        shiftKey: true,
        ctrlKey: true,
        callback: () => {
          navigator.clipboard.writeText(terminal.getSelection());
        },
      },
    ];

    terminal.attachCustomKeyEventHandler((ev) => {
      if (ev.type === "keydown") {
        for (let i in keyMap) {
          if (keyMap[i].key == ev.key && keyMap[i].shiftKey == ev.shiftKey && keyMap[i].ctrlKey == ev.ctrlKey) {
            keyMap[i].callback();
            return false;
          }
        }
      }
      return true;
    });
  }

  componentWillUnmount() {
    // send SIGINT on terminal close
    this.socket?.close(1000, "Component closed");
    if (this.terminal) this.terminal.dispose();
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
  }

  private onSocketOpen() {
    const { socket, textEncoder, terminal, fitAddon, state } = this;
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
          })
        )
      );

      if (state.opened && terminal) {
        terminal.reset();
        terminal.resize(dims.cols, dims.rows);
      }
    }

    // clear terminal at start
    // this.onTerminalData('cd && clear \r');

    // send initial commands to terminal
    const { initialCommands } = this.props;
    if (initialCommands) {
      initialCommands.forEach((command) => {
        this.socket?.send(textEncoder.encode(Command.INPUT + command));
        // this.onTerminalData(command);
      });
    }
  }

  private onSocketError(_event: Event) {
    // might be fired when component is closed
    console.error("[ttyd] websocket connection error: ", _event);
  }

  private onSocketData(event: MessageEvent) {
    const { textDecoder } = this;
    const rawData = event.data as ArrayBuffer;
    const cmd = String.fromCharCode(new Uint8Array(rawData)[0]);
    const data = rawData.slice(1);

    const { onIncomingData } = this.props;
    if (onIncomingData) onIncomingData(textDecoder.decode(data));

    switch (cmd) {
      case Command.OUTPUT:
        // show data in the terminal
        this.terminal?.write(textDecoder.decode(data));
        break;
      case Command.SET_WINDOW_TITLE:
        // this.title = textDecoder.decode(data);
        // document.title = this.title;
        break;
      case Command.SET_PREFERENCES: {
        break;
      }
      default:
        console.warn(`[ttyd] unknown command: ${cmd}`);
        break;
    }
    if (!this.gotFocus) {
      this.terminal?.focus();
      this.gotFocus = true;
    }
  }

  render() {
    const { state, fitAddon } = this;
    fitAddon.fit();
    return (
      <Stack width="100%" height="100%" alignItems="center">
        {!state.opened && (
          <Alert severity="info">
            <AlertTitle>TTYD Daemon is not available</AlertTitle>
            If you want to check this terminal, please start the terminal manager daemon on the host.
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
                    style: { textAlign: "center" },
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
          ref={(c: HTMLElement) => {
            this.container = c;
            return this.container;
          }}
          width="100%"
          height={state.opened ? "100%" : 0}
          visibility={state.opened ? "visible" : "hidden"}
          overflow="auto"
        ></Box>
      </Stack>
    );
  }

  private pause() {
    const { textEncoder, socket } = this;
    socket?.send(textEncoder.encode(Command.PAUSE));
  }

  private onTerminalResize(size: { cols: number; rows: number }) {
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ columns: size.cols, rows: size.rows });
      socket.send(textEncoder.encode(Command.RESIZE_TERMINAL + msg));
    }
  }

  private onTerminalData(data: string) {
    if (data?.charCodeAt(0) === 4) {
      const { wsUrl, tokenUrl, onCtrlD } = this.props;
      if (onCtrlD) {
        console.log(`CTRL+D: ${data?.charCodeAt(0)}`);
        onCtrlD(wsUrl, tokenUrl);
      }
    }
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(Command.INPUT + data));
    }
  }

  private resume() {
    const { textEncoder, socket } = this;
    socket?.send(textEncoder.encode(Command.RESUME));
  }
}
