/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import CloseIcon from "@mui/icons-material/Close";
import FirstPageIcon from "@mui/icons-material/FirstPage";
import LastPageIcon from "@mui/icons-material/LastPage";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import { Alert, AlertTitle, Box, IconButton, Link, Stack, Typography } from "@mui/material";
import { FitAddon } from "@xterm/addon-fit";
import { ISearchOptions, SearchAddon } from "@xterm/addon-search";
import { Unicode11Addon } from "@xterm/addon-unicode11";
import { WebLinksAddon } from "@xterm/addon-web-links";
import { WebglAddon } from "@xterm/addon-webgl";
import { ITerminalOptions, Terminal as XTerminal } from "@xterm/xterm";
import "@xterm/xterm/css/xterm.css";
import React from "react";

import { ISettingsContext } from "@/renderer/context/SettingsContext";
import { CmdType } from "@/renderer/providers";
import Provider from "@/renderer/providers/Provider";
import SearchBar from "../UI/SearchBar";

enum Command {
  // server side
  OUTPUT = "0",
  SET_WINDOW_TITLE = "1",
  SET_PREFERENCES = "2",
}

enum CommandClient {
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
  type: CmdType;
  onIncomingData?: (data: string) => undefined;
  onCtrlD?: (wsUrl: string, tokenUrl: string) => undefined;
  settingsCtx: ISettingsContext;
  provider?: Provider;
}

type XtermState = {
  opened: boolean;
  showSearchBar: boolean;
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

  private settingsCtx: ISettingsContext | null = null;

  private fontSize = 14;

  private searchText = "";

  private resizeObserver: ResizeObserver | null = null;

  private gotFocus = false;

  private lastContainerSize: { width: number; height: number } = {
    width: 0,
    height: 0,
  };

  private type: CmdType;

  private provider: Provider | undefined;

  constructor(props: Props) {
    super(props);

    this.state = {
      opened: false,
      showSearchBar: false,
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
    this.type = props.type;
    this.provider = props.provider;
    this.settingsCtx = props.settingsCtx;
    this.fontSize = this.settingsCtx?.get("fontSizeTerminal") as number;
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

  private connect(): void {
    console.log(`[ttyd] connect to ${this.props.wsUrl}`);
    this.socket = new WebSocket(this.props.wsUrl, ["tty"]);
    this.socket.binaryType = "arraybuffer";
    this.socket.onopen = this.onSocketOpen;
    this.socket.onmessage = this.onSocketData;
    // this.socket.onclose = this.onSocketClose;
    this.socket.onerror = this.onSocketError;
  }

  async componentDidMount(): Promise<void> {
    const { termOptions } = this.props;
    termOptions.allowProposedApi = true;
    this.terminal = new XTerminal(termOptions);

    const { terminal, fitAddon, searchAddon, webglAddon, unicode11Addon } = this;

    terminal.loadAddon(fitAddon);
    terminal.loadAddon(searchAddon);
    terminal.loadAddon(new WebLinksAddon());
    terminal.loadAddon(unicode11Addon);
    webglAddon.onContextLoss((/*event*/) => {
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
        altKey: false,
        callback: (): void => {
          navigator.clipboard.writeText(terminal.getSelection());
        },
      },
      {
        key: "+",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.fontSize = this.fontSize + 1;
          const { terminal } = this;
          if (terminal) terminal.options.fontSize = this.fontSize;
          this.settingsCtx?.set("fontSizeTerminal", this.fontSize);
          fitAddon.fit();
        },
      },
      {
        key: "-",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.fontSize = this.fontSize - 1;
          const { terminal } = this;
          if (terminal) terminal.options.fontSize = this.fontSize;
          this.settingsCtx?.set("fontSizeTerminal", this.fontSize);
          fitAddon.fit();
        },
      },
      {
        key: "0",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.fontSize = this.settingsCtx?.getDefault("fontSizeTerminal") as number;
          const { terminal } = this;
          if (terminal) terminal.options.fontSize = this.fontSize;
          this.settingsCtx?.set("fontSizeTerminal", this.fontSize);
          fitAddon.fit();
        },
      },
      {
        key: "Backspace",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          const { terminal } = this;
          if (terminal) terminal.clear();
        },
      },
      {
        key: "f",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.setState({ opened: this.state.opened, showSearchBar: !this.state.showSearchBar });
        },
      },
    ];

    terminal.attachCustomKeyEventHandler((ev) => {
      if (ev.type === "keydown") {
        for (const i in keyMap) {
          if (
            keyMap[i].key === ev.key &&
            keyMap[i].shiftKey === ev.shiftKey &&
            keyMap[i].ctrlKey === ev.ctrlKey &&
            keyMap[i].altKey === ev.altKey
          ) {
            keyMap[i].callback();
            ev.preventDefault();
            ev.stopPropagation();
            return false;
          }
        }
      }
      return true;
    });
  }

  componentWillUnmount(): void {
    // send SIGINT on terminal close
    this.socket?.close(1000, "Component closed");
    if (this.terminal) this.terminal.dispose();
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
  }

  private async onSocketOpen(): Promise<void> {
    const { socket, textEncoder, terminal, fitAddon, state } = this;
    const dims = fitAddon.proposeDimensions();
    this.setState({
      opened: true,
      showSearchBar: this.state.showSearchBar,
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
      for (const command of initialCommands) {
        this.socket?.send(textEncoder.encode(CommandClient.INPUT + command));
        // this.onTerminalData(command);
      }
    }
    if (this.type === CmdType.SET_TIME && this.provider) {
      if (await this.provider.updateTimeDiff()) {
        this.socket?.send(
          textEncoder.encode(
            `${CommandClient.INPUT}sudo /bin/date -s ${new Date(this.provider.timestamp).toISOString()}\n\r`
          )
        );
      }
    }
  }

  private onSocketError(_event: Event): void {
    // might be fired when component is closed
    console.error("[ttyd] websocket connection error: ", _event);
  }

  private onSocketData(event: MessageEvent): void {
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

  render(): JSX.Element {
    const { state, fitAddon } = this;
    fitAddon.fit();
    return (
      <Stack width="100%" height="100%" alignItems="center">
        {!state.opened && (
          <Alert severity="info">
            <AlertTitle>TTYD Daemon on {this.props.wsUrl} is not available</AlertTitle>
            <Typography>
              If you want to check this terminal, please start the terminal manager daemon on the host using{" "}
              <RocketLaunchIcon fontSize="inherit" /> in 'Hosts' panel or manually:
            </Typography>
            <Typography sx={{ ml: "1em" }}>ttyd --writable --port 8681 bash</Typography>
            <Typography sx={{ mt: "1em" }}>
              Install instructions:{" "}
              <Link mt={2} href="https://github.com/tsl0922/ttyd" target="_blank" color="inherit">
                https://github.com/tsl0922/ttyd
              </Link>
            </Typography>
          </Alert>
        )}

        {state.opened && state.showSearchBar && (
          <Stack
            spacing={0.3}
            // padding={0.5}
            width="100%"
            direction="row"
            alignContent="center"
            justifyItems="center"
            alignItems="center"
          >
            <SearchBar
              onSearch={(value: string) => {
                const { searchAddon, searchAddonOptions } = this;
                // fitAddon.fit();
                this.searchText = value;
                searchAddon.findNext(value, searchAddonOptions);
              }}
              onCloseRequest={() => {
                this.searchText = "";
                const { searchAddon } = this;
                searchAddon.clearDecorations();
                this.setState({ opened: this.state.opened, showSearchBar: !this.state.showSearchBar });
                this.terminal?.focus();
                fitAddon.fit();
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
              <FirstPageIcon fontSize="inherit" />
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
              <LastPageIcon fontSize="inherit" />
            </IconButton>

            <IconButton
              size="small"
              onClick={() => {
                this.searchText = "";
                const { searchAddon } = this;
                searchAddon.clearDecorations();
                this.setState({ opened: this.state.opened, showSearchBar: !this.state.showSearchBar });
                this.terminal?.focus();
              }}
            >
              <CloseIcon fontSize="inherit" />
            </IconButton>
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
        />
      </Stack>
    );
  }

  private pause(): void {
    const { textEncoder, socket } = this;
    socket?.send(textEncoder.encode(CommandClient.PAUSE));
  }

  private onTerminalResize(size: { cols: number; rows: number }): void {
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ columns: size.cols, rows: size.rows });
      socket.send(textEncoder.encode(CommandClient.RESIZE_TERMINAL + msg));
    }
  }

  private onTerminalData(data: string): void {
    if (data?.charCodeAt(0) === 4) {
      const { wsUrl, tokenUrl, onCtrlD } = this.props;
      if (onCtrlD) {
        console.log(`CTRL+D: ${data?.charCodeAt(0)}`);
        onCtrlD(wsUrl, tokenUrl);
      }
    }
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(CommandClient.INPUT + data));
    }
  }

  private resume(): void {
    const { textEncoder, socket } = this;
    socket?.send(textEncoder.encode(CommandClient.RESUME));
  }
}
