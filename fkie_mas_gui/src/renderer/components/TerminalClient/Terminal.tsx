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
import Provider from "@/renderer/providers/Provider";
import { CmdType, CmdTypes } from "@/types";
import SearchBar from "../UI/SearchBar";
import { LineNumberHighlighter } from "./LineNumberHighlighter";

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

/** Debounce delay for the line number highlighting */
const HIGHLIGHT_DEBOUNCE_MS = 150;

/** Background color used to highlight line numbers */
const LINE_NUMBER_COLOR = "#004C99";

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
  remoteProvider?: Provider;
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

  /**
   * Only ONE SearchAddon instance is allowed per terminal.
   * Two instances share the same marker/decoration lifecycle inside xterm and
   * crash on dispose ("Cannot read properties of undefined (reading '_isDisposed')").
   */
  private searchAddon: SearchAddon = new SearchAddon();

  private searchAddonOptions: ISearchOptions;

  /** Line numbers are highlighted with own markers, independent from SearchAddon */
  private lineNumberHighlighter: LineNumberHighlighter | null = null;

  private webglAddon: WebglAddon = new WebglAddon();

  private webglDisposed = false;

  private unicode11Addon = new Unicode11Addon();

  private socket: WebSocket | null = null;

  private token: string | null = null;

  // private title: string = "Terminal";

  private settingsCtx: ISettingsContext | null = null;

  private fontSize = 14;

  private searchText = "";

  private resizeObserver: ResizeObserver | null = null;

  private gotFocus = false;

  private isUnmounting = false;

  private highlightTimer: number | undefined = undefined;

  private lastContainerSize: { width: number; height: number } = {
    width: 0,
    height: 0,
  };

  private type: CmdType;

  private provider: Provider | undefined;

  private remoteProvider: Provider | undefined;

  private sudoRequested: boolean = false;

  private timeSyncIterations: number = 0;

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

    this.type = props.type;
    this.provider = props.provider;
    this.remoteProvider = props.remoteProvider;
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

    webglAddon.onContextLoss(() => {
      // guard against double dispose of the webgl addon
      if (this.webglDisposed) return;
      this.webglDisposed = true;
      try {
        webglAddon.dispose();
      } catch (error) {
        console.warn("[ttyd] webgl addon dispose failed:", error);
      }
    });
    terminal.loadAddon(webglAddon);

    terminal.onData(this.onTerminalData);
    terminal.onResize(this.onTerminalResize);

    this.connect();

    if (this.container) {
      terminal.open(this.container);
    }

    // own line number highlighting, independent from SearchAddon
    this.lineNumberHighlighter = new LineNumberHighlighter(terminal, LINE_NUMBER_COLOR);

    this.safeFit();

    if (this.container) {
      this.resizeObserver = new ResizeObserver(() => {
        if (this.isUnmounting || !this.terminal) return;
        const rect = this.container?.getBoundingClientRect();
        if (
          rect &&
          rect.width > 0 &&
          (this.lastContainerSize.width !== rect.width || this.lastContainerSize.height !== rect.height)
        ) {
          this.lastContainerSize = { width: rect.width, height: rect.height };
          this.safeFit();
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
          this.setFontSize(this.fontSize + 1);
        },
      },
      {
        key: "-",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.setFontSize(this.fontSize - 1);
        },
      },
      {
        key: "0",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.setFontSize(this.settingsCtx?.getDefault("fontSizeTerminal") as number);
        },
      },
      {
        key: "Backspace",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.clearSearchDecorations();
          this.lineNumberHighlighter?.clear();
          this.terminal?.clear();
        },
      },
      {
        key: "f",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.setState((prev) => ({ ...prev, showSearchBar: !prev.showSearchBar }));
        },
      },
    ];

    terminal.attachCustomKeyEventHandler((ev) => {
      if (ev.type === "keydown") {
        for (const entry of keyMap) {
          if (
            entry.key === ev.key &&
            entry.shiftKey === ev.shiftKey &&
            entry.ctrlKey === ev.ctrlKey &&
            entry.altKey === ev.altKey
          ) {
            entry.callback();
            ev.preventDefault();
            ev.stopPropagation();
            return false;
          }
        }
      }
      return true;
    });
  }

  componentDidUpdate(): void {
    // fit() must never be called from render(): it is a side effect and may run
    // on an already disposed terminal
    this.safeFit();
  }

  componentWillUnmount(): void {
    this.isUnmounting = true;

    // 1) stop pending highlight work
    if (this.highlightTimer !== undefined) {
      window.clearTimeout(this.highlightTimer);
      this.highlightTimer = undefined;
    }

    // 2) detach socket handlers FIRST, close() is async and messages may still arrive
    if (this.socket) {
      this.socket.onopen = null;
      this.socket.onmessage = null;
      this.socket.onerror = null;
      this.socket.onclose = null;
      try {
        this.socket.close(1000, "Component closed");
      } catch (error) {
        console.warn("[ttyd] socket close failed:", error);
      }
      this.socket = null;
    }

    // 3) stop resize observer before touching any addon
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
      this.resizeObserver = null;
    }

    // 4) drop own decorations and search decorations while terminal is still alive
    this.lineNumberHighlighter?.dispose();
    this.lineNumberHighlighter = null;
    this.clearSearchDecorations();

    // 5) dispose the terminal; it disposes its own addons - never dispose them manually
    try {
      this.terminal?.dispose();
    } catch (error) {
      console.warn("[ttyd] terminal dispose failed:", error);
    }
    this.terminal = null;
  }

  private connect(): void {
    console.log(`[ttyd] connect to ${this.props.wsUrl}`);
    this.socket = new WebSocket(this.props.wsUrl, ["tty"]);
    this.socket.binaryType = "arraybuffer";
    this.socket.onopen = this.onSocketOpen;
    this.socket.onmessage = this.onSocketData;
    this.socket.onerror = this.onSocketError;
  }

  private setFontSize(size: number): void {
    if (!size || Number.isNaN(size)) return;
    this.fontSize = size;
    if (this.terminal) this.terminal.options.fontSize = this.fontSize;
    this.settingsCtx?.set("fontSizeTerminal", this.fontSize);
    this.safeFit();
  }

  private safeFit(): void {
    if (this.isUnmounting || !this.terminal || !this.container) return;
    try {
      this.fitAddon.fit();
    } catch (error) {
      console.warn("[ttyd] fit failed:", error);
    }
  }

  /** Clears only the search decorations, line numbers stay visible */
  private clearSearchDecorations(): void {
    try {
      this.searchAddon.clearDecorations();
    } catch (error) {
      console.warn("[ttyd] clearDecorations failed:", error);
    }
  }

  /** Scans new buffer lines for line numbers, debounced */
  private scheduleLineNumberHighlight(): void {
    if (this.isUnmounting || !this.terminal || !this.lineNumberHighlighter) return;
    if (this.highlightTimer !== undefined) return;

    this.highlightTimer = window.setTimeout(() => {
      this.highlightTimer = undefined;
      if (this.isUnmounting || !this.terminal || !this.lineNumberHighlighter) return;
      try {
        this.lineNumberHighlighter.scan();
      } catch (error) {
        console.warn("[ttyd] line number highlight failed:", error);
      }
    }, HIGHLIGHT_DEBOUNCE_MS);
  }

  private runSearch(direction: "next" | "previous"): void {
    if (this.isUnmounting || !this.terminal || !this.searchText) return;
    try {
      if (direction === "next") {
        this.searchAddon.findNext(this.searchText, this.searchAddonOptions);
      } else {
        this.searchAddon.findPrevious(this.searchText, this.searchAddonOptions);
      }
    } catch (error) {
      console.warn("[ttyd] search failed:", error);
    }
  }

  private closeSearchBar(): void {
    this.searchText = "";
    this.clearSearchDecorations();
    this.setState((prev) => ({ ...prev, showSearchBar: false }));
    this.terminal?.focus();
    this.safeFit();
  }

  private async onSocketOpen(): Promise<void> {
    if (this.isUnmounting) return;

    const { socket, textEncoder, terminal, fitAddon } = this;
    const wasOpened = this.state.opened;

    let dims: { cols: number; rows: number } | undefined;
    try {
      dims = fitAddon.proposeDimensions();
    } catch (error) {
      console.warn("[ttyd] proposeDimensions failed:", error);
    }

    this.setState((prev) => ({ ...prev, opened: true }));

    if (dims && socket && socket.readyState === WebSocket.OPEN) {
      socket.send(
        textEncoder.encode(
          JSON.stringify({
            AuthToken: this.token,
            columns: dims.cols,
            rows: dims.rows,
          })
        )
      );

      if (wasOpened && terminal) {
        terminal.reset();
        terminal.resize(dims.cols, dims.rows);
        this.lineNumberHighlighter?.clear();
      }
    }

    // send initial commands to terminal
    const { initialCommands } = this.props;
    if (initialCommands) {
      for (const command of initialCommands) {
        this.socket?.send(textEncoder.encode(CommandClient.INPUT + command));
      }
    }
    if (this.type === CmdTypes.SET_TIME && this.provider) {
      this.sendTimeSync();
    }
  }

  private async sendTimeSync(): Promise<void> {
    if (this.isUnmounting) return;
    if (!this.provider || !this.remoteProvider || this.timeSyncIterations > 5) return;
    if (await this.provider.updateTimeDiff()) {
      if (await this.remoteProvider?.updateTimeDiff()) {
        if (this.isUnmounting) return;
        let diff = 0;
        let diffInfoStr = ` - current difference ${this.remoteProvider.timeDiff.toFixed(0)}ms`;
        if (this.timeSyncIterations > 1) {
          if (Math.abs(this.remoteProvider.timeDiff) < 100) {
            return;
          }
          diff = this.remoteProvider.timeDiff;
          diffInfoStr = ` - add difference ${diff.toFixed(0)}ms`;
        }
        this.socket?.send(
          this.textEncoder.encode(
            `${CommandClient.INPUT}sudo /bin/date -s ${new Date(
              this.provider.timestamp - diff
            ).toISOString()} && echo "date set ok${diffInfoStr}"\n`
          )
        );
        this.timeSyncIterations += 1;
      }
    }
  }

  private onSocketError(event: Event): void {
    // might be fired when component is closed
    if (this.isUnmounting) return;
    console.error("[ttyd] websocket connection error: ", event);
  }

  private onSocketData(event: MessageEvent): void {
    // guard: messages can still arrive while the component is going away
    if (this.isUnmounting || !this.terminal) return;

    const { textDecoder } = this;
    const rawData = event.data as ArrayBuffer;
    const cmd = String.fromCharCode(new Uint8Array(rawData)[0]);
    const data = rawData.slice(1);

    const { onIncomingData } = this.props;
    if (onIncomingData) onIncomingData(textDecoder.decode(data));

    switch (cmd) {
      case Command.OUTPUT:
        // scan only after the chunk has been parsed, cursor position is valid then
        this.terminal.write(textDecoder.decode(data), () => {
          this.scheduleLineNumberHighlight();
        });
        break;
      case Command.SET_WINDOW_TITLE:
        // this.title = textDecoder.decode(data);
        break;
      case Command.SET_PREFERENCES:
        break;
      default:
        console.warn(`[ttyd] unknown command: ${cmd}`);
        break;
    }

    if (!this.gotFocus) {
      this.terminal.focus();
      this.gotFocus = true;
    }

    if (this.type === CmdTypes.SET_TIME && this.provider) {
      const decodedData = textDecoder.decode(data);
      if (decodedData.startsWith("sudo ")) {
        this.sudoRequested = true;
      } else if (this.sudoRequested) {
        if (decodedData.startsWith("date set ok")) {
          this.sudoRequested = false;
          this.sendTimeSync();
        }
      }
    }
  }

  render(): JSX.Element {
    const { state } = this;
    return (
      <Stack width="100%" height="100%" alignItems="center">
        {!state.opened && (
          <Alert severity="info">
            <AlertTitle>TTYD Daemon on {this.props.wsUrl} is not available</AlertTitle>
            <Typography>
              If you want to check this terminal, please start the terminal manager daemon on the host using{" "}
              <RocketLaunchIcon fontSize="inherit" /> in &apos;Hosts&apos; panel or manually:
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
            width="100%"
            direction="row"
            alignContent="center"
            justifyItems="center"
            alignItems="center"
          >
            <SearchBar
              onSearch={(value: string) => {
                this.searchText = value;
                if (!value) {
                  this.clearSearchDecorations();
                  return;
                }
                this.runSearch("next");
              }}
              onCloseRequest={() => {
                this.closeSearchBar();
              }}
              placeholder="Search Text (Supports Regular Expressions)"
              defaultValue={this.searchText}
              fullWidth
            />

            <IconButton
              size="small"
              onClick={() => {
                this.runSearch("previous");
              }}
            >
              <FirstPageIcon fontSize="inherit" />
            </IconButton>

            <IconButton
              size="small"
              onClick={() => {
                this.runSearch("next");
              }}
            >
              <LastPageIcon fontSize="inherit" />
            </IconButton>

            <IconButton
              size="small"
              onClick={() => {
                this.closeSearchBar();
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
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(CommandClient.PAUSE));
    }
  }

  private resume(): void {
    const { textEncoder, socket } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(CommandClient.RESUME));
    }
  }

  private onTerminalResize(size: { cols: number; rows: number }): void {
    if (this.isUnmounting) return;
    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ columns: size.cols, rows: size.rows });
      socket.send(textEncoder.encode(CommandClient.RESIZE_TERMINAL + msg));
    }
  }

  private onTerminalData(data: string): void {
    if (this.isUnmounting) return;

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
}
