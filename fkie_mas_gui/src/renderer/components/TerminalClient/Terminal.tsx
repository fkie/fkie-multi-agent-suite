/*
Based on: https://github.com/tsl0922/ttyd

MIT License
Copyright (c) 2016 Shuanglei Tao <tsl0922@gmail.com>
*/

import CloseIcon from "@mui/icons-material/Close";
import FirstPageIcon from "@mui/icons-material/FirstPage";
import LastPageIcon from "@mui/icons-material/LastPage";
import MoneyIcon from "@mui/icons-material/Money";
import RocketLaunchIcon from "@mui/icons-material/RocketLaunch";
import SearchIcon from "@mui/icons-material/Search";
import { Alert, AlertTitle, Box, IconButton, Link, Stack, ToggleButton, Tooltip, Typography } from "@mui/material";
import { FitAddon } from "@xterm/addon-fit";
import { ISearchOptions, SearchAddon } from "@xterm/addon-search";
import { Unicode11Addon } from "@xterm/addon-unicode11";
import { WebLinksAddon } from "@xterm/addon-web-links";
import { WebglAddon } from "@xterm/addon-webgl";
import { IDisposable, ITerminalOptions, Terminal as XTerminal } from "@xterm/xterm";
import "@xterm/xterm/css/xterm.css";
import React from "react";

import { BUTTON_LOCATIONS, ISettingsContext } from "@/renderer/context/SettingsContext";
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

/** Flow control: pause the server once this many unprocessed bytes are buffered */
const FLOW_CONTROL_HIGH_WATER = 100_000;

/** Flow control: resume once the backlog drops below this value */
const FLOW_CONTROL_LOW_WATER = 10_000;

/** Reconnect backoff */
const RECONNECT_BASE_DELAY_MS = 1_000;
const RECONNECT_MAX_DELAY_MS = 15_000;
const RECONNECT_MAX_ATTEMPTS = 10;

/** Duration the resize overlay stays visible */
const RESIZE_OVERLAY_MS = 800;

/** Maximum number of time sync iterations */
const TIME_SYNC_MAX_ITERATIONS = 5;

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
  onIncomingData?: (data: string) => void;
  onCtrlD?: (wsUrl: string, tokenUrl: string) => void;
  onTitleChange?: (title: string) => void;
  settingsCtx: ISettingsContext;
  provider?: Provider;
  remoteProvider?: Provider;
  buttonLocation: string;
}

type XtermState = {
  opened: boolean;
  showSearchBar: boolean;
  highlightLineNumbers: boolean;
  /** null hides the overlay */
  resizeOverlay: string | null;
};

export class Terminal extends React.Component<Props, XtermState> {
  private textEncoder: TextEncoder;

  /** Streaming decoder for OUTPUT payloads, keeps state across chunks */
  private textDecoder: TextDecoder;

  /** Separate decoder for control payloads, must not disturb the streaming state */
  private controlDecoder: TextDecoder;

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

  /** Created lazily, only when clientOptions.rendererType requests webgl */
  private webglAddon: WebglAddon | null = null;

  private webglDisposed = false;

  private unicode11Addon = new Unicode11Addon();

  private socket: WebSocket | null = null;

  private token: string | null = null;

  private title: string;

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

  private highlightPending = false;

  /** Unprocessed bytes handed to terminal.write() */
  private pendingBytes = 0;

  private flowPaused = false;

  private reconnectAttempts = 0;

  private reconnectTimer: number | undefined = undefined;

  private resizeOverlayTimer: number | undefined = undefined;

  private leaveAlertHandler: ((event: BeforeUnloadEvent) => void) | null = null;

  /** xterm listeners, disposed BEFORE terminal.dispose() */
  private disposables: IDisposable[] = [];

  constructor(props: Props) {
    super(props);

    this.state = {
      opened: false,
      showSearchBar: false,
      highlightLineNumbers: true,
      resizeOverlay: null,
    };
    this.textEncoder = new TextEncoder();
    this.textDecoder = new TextDecoder();
    this.controlDecoder = new TextDecoder();
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
    this.title = props.clientOptions?.titleFixed || props.name;

    this.onSocketOpen = this.onSocketOpen.bind(this);
    this.onSocketError = this.onSocketError.bind(this);
    this.onSocketData = this.onSocketData.bind(this);
    this.onSocketClose = this.onSocketClose.bind(this);
    this.connect = this.connect.bind(this);
    this.pause = this.pause.bind(this);
    this.onTerminalResize = this.onTerminalResize.bind(this);
    this.onTerminalData = this.onTerminalData.bind(this);
    this.resume = this.resume.bind(this);
  }

  private toggleHighlightLineNumbers(): void {
    this.setState((prev) => ({ ...prev, highlightLineNumbers: !prev.highlightLineNumbers }));
    if (!this.state.highlightLineNumbers) {
      try {
        this.lineNumberHighlighter?.scan();
      } catch (error) {
        console.warn("[ttyd] line number highlight failed:", error);
      }
    } else {
      this.clearSearchDecorations();
      this.lineNumberHighlighter?.clear();
    }
  }

  async componentDidMount(): Promise<void> {
    const { termOptions, clientOptions } = this.props;

    // never mutate props: xterm keeps a reference to the options object
    this.terminal = new XTerminal({
      ...termOptions,
      allowProposedApi: true,
      fontSize: this.fontSize || termOptions.fontSize,
    });

    const { terminal, fitAddon, searchAddon, unicode11Addon } = this;

    terminal.loadAddon(fitAddon);
    terminal.loadAddon(searchAddon);
    terminal.loadAddon(new WebLinksAddon());
    terminal.loadAddon(unicode11Addon);
    // the addon stays inert until its version is activated
    terminal.unicode.activeVersion = "11";

    this.disposables.push(terminal.onData(this.onTerminalData));
    this.disposables.push(terminal.onResize(this.onTerminalResize));

    // a fixed title always wins over the escape sequence
    this.disposables.push(
      terminal.onTitleChange((title) => {
        if (this.isUnmounting) return;
        this.title = this.props.clientOptions?.titleFixed || title;
        this.props.onTitleChange?.(this.title);
      })
    );

    // re-scan when the user scrolls back: reflow or in-place overwrites may have
    // invalidated decorations of rows that are outside the tail window
    this.disposables.push(
      terminal.onScroll(() => {
        this.scheduleLineNumberHighlight();
      })
    );

    this.disposables.push(
      terminal.buffer.onBufferChange((buffer) => {
        this.lineNumberHighlighter?.clear();
        if (buffer.type !== "normal") return;
        this.scheduleLineNumberHighlight();
      })
    );

    if (this.container) {
      terminal.open(this.container);
    }

    // WebGL must be loaded AFTER open() and only when requested
    if (clientOptions?.rendererType === "webgl") {
      this.loadWebglAddon(terminal);
    }

    // own line number highlighting, independent from SearchAddon
    this.lineNumberHighlighter = new LineNumberHighlighter(terminal, LINE_NUMBER_COLOR);

    if (this.container) {
      const rect = this.container.getBoundingClientRect();
      this.lastContainerSize = { width: rect.width, height: rect.height };
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
        key: "l",
        shiftKey: false,
        ctrlKey: true,
        altKey: false,
        callback: (): void => {
          this.toggleHighlightLineNumbers();
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

    if (!clientOptions?.disableLeaveAlert) {
      this.leaveAlertHandler = (event: BeforeUnloadEvent): void => {
        event.preventDefault();
        event.returnValue = "Close terminal? this will also terminate the command.";
      };
      window.addEventListener("beforeunload", this.leaveAlertHandler);
    }

    // size the pty correctly BEFORE the shell starts
    this.safeFit();
    await this.connect();
  }

  componentDidUpdate(prevProps: Props): void {
    const { termOptions, clientOptions } = this.props;
    if (this.terminal && prevProps.termOptions.theme !== termOptions.theme) {
      // reassigning the theme forces the renderer to refresh its color cache
      this.terminal.options.theme = termOptions.theme;
    }
    if (clientOptions?.titleFixed && prevProps.clientOptions?.titleFixed !== clientOptions.titleFixed) {
      this.title = clientOptions.titleFixed;
      this.props.onTitleChange?.(this.title);
    }
    this.safeFit();
  }

  componentWillUnmount(): void {
    this.isUnmounting = true;
    this.highlightPending = false;

    // 0) cancel reconnect/overlay timers and the leave alert
    if (this.reconnectTimer !== undefined) {
      window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = undefined;
    }
    if (this.resizeOverlayTimer !== undefined) {
      window.clearTimeout(this.resizeOverlayTimer);
      this.resizeOverlayTimer = undefined;
    }
    if (this.leaveAlertHandler) {
      window.removeEventListener("beforeunload", this.leaveAlertHandler);
      this.leaveAlertHandler = null;
    }

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
    this.flowPaused = false;
    this.pendingBytes = 0;

    // 3) stop resize observer before touching any addon
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
      this.resizeObserver = null;
    }

    // 4) drop own decorations and search decorations while terminal is still alive
    for (const disposable of this.disposables) {
      try {
        disposable.dispose();
      } catch (error) {
        console.warn("[ttyd] listener dispose failed:", error);
      }
    }
    this.disposables = [];

    this.lineNumberHighlighter?.dispose();
    this.lineNumberHighlighter = null;
    this.clearSearchDecorations();

    // 5) dispose the terminal; it disposes its own addons - never dispose them manually
    try {
      this.terminal?.dispose();
    } catch {
      // } catch (error) {
      //   console.warn("[ttyd] terminal dispose failed:", error);
    }
    this.terminal = null;
    try {
      this.webglAddon?.dispose();
    } catch {
      //
    }
    this.webglAddon = null;
  }

  /** Creates and loads the WebGL renderer, falls back to the DOM renderer on failure */
  private loadWebglAddon(terminal: XTerminal): void {
    try {
      const addon = new WebglAddon();
      addon.onContextLoss(() => {
        // guard against double dispose of the webgl addon
        if (this.webglDisposed) return;
        this.webglDisposed = true;
        try {
          addon.dispose();
        } catch (error) {
          console.warn("[ttyd] webgl addon dispose failed:", error);
        }
        this.webglAddon = null;
      });
      terminal.loadAddon(addon);
      this.webglAddon = addon;
    } catch (error) {
      console.warn("[ttyd] webgl not available, falling back to dom renderer:", error);
      this.webglAddon = null;
    }
  }

  private async fetchToken(): Promise<string> {
    const { tokenUrl } = this.props;
    if (!tokenUrl) return "";
    try {
      const response = await fetch(tokenUrl);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const json = (await response.json()) as { token?: string };
      return json.token ?? "";
    } catch (error) {
      console.warn("[ttyd] token request failed:", error);
      return "";
    }
  }

  private async connect(): Promise<void> {
    if (this.isUnmounting) return;
    if (this.socket !== null && this.socket.readyState !== WebSocket.CLOSED) return;

    // token must be available BEFORE the auth message is sent
    this.token = await this.fetchToken();
    if (this.isUnmounting) return;

    console.log(`[ttyd] connect to ${this.props.wsUrl}`);
    this.socket = new WebSocket(this.props.wsUrl, ["tty"]);
    this.socket.binaryType = "arraybuffer";
    this.socket.onopen = this.onSocketOpen;
    this.socket.onmessage = this.onSocketData;
    this.socket.onerror = this.onSocketError;
    this.socket.onclose = this.onSocketClose;
  }

  private onSocketClose(event: CloseEvent): void {
    if (this.isUnmounting) return;
    console.log(`[ttyd] socket closed: code=${event.code} reason=${event.reason}`);

    this.socket = null;
    this.flowPaused = false;
    this.pendingBytes = 0;
    this.setState((prev) => ({ ...prev, opened: false }));

    // 1000 means the peer closed intentionally, do not fight it
    if (event.code === 1000) return;
    this.scheduleReconnect();
  }

  private scheduleReconnect(): void {
    if (this.isUnmounting || this.reconnectTimer !== undefined) return;
    if (this.reconnectAttempts >= RECONNECT_MAX_ATTEMPTS) {
      console.warn("[ttyd] giving up reconnect after max attempts");
      return;
    }

    const delay = Math.min(RECONNECT_BASE_DELAY_MS * 2 ** this.reconnectAttempts, RECONNECT_MAX_DELAY_MS);
    this.reconnectAttempts += 1;
    console.log(`[ttyd] reconnect attempt ${this.reconnectAttempts} in ${delay}ms`);

    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = undefined;
      void this.connect();
    }, delay);
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
      // renderer dimensions can be undefined while the layout is in transition
      console.warn("[ttyd] fit failed:", error);
    }
  }

  /** Shows the current terminal size for a short moment */
  private showResizeOverlay(cols: number, rows: number): void {
    if (this.props.clientOptions?.disableResizeOverlay) return;
    if (this.resizeOverlayTimer !== undefined) {
      window.clearTimeout(this.resizeOverlayTimer);
    }
    this.setState((prev) => ({ ...prev, resizeOverlay: `${cols}x${rows}` }));
    this.resizeOverlayTimer = window.setTimeout(() => {
      this.resizeOverlayTimer = undefined;
      if (this.isUnmounting) return;
      this.setState((prev) => ({ ...prev, resizeOverlay: null }));
    }, RESIZE_OVERLAY_MS);
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
    if (!this.state.highlightLineNumbers) return;
    if (this.isUnmounting || !this.terminal || !this.lineNumberHighlighter) return;
    if (this.highlightTimer !== undefined) {
      // remember that more data arrived while the timer was pending
      this.highlightPending = true;
      return;
    }

    this.highlightTimer = window.setTimeout(() => {
      this.highlightTimer = undefined;
      if (this.isUnmounting || !this.terminal || !this.lineNumberHighlighter) return;
      try {
        this.lineNumberHighlighter.scan();
      } catch (error) {
        console.warn("[ttyd] line number highlight failed:", error);
      }
      // trailing run for data that arrived during the throttle window
      if (this.highlightPending) {
        this.highlightPending = false;
        this.scheduleLineNumberHighlight();
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
    const { socket, textEncoder, terminal } = this;
    if (!terminal || !socket || socket.readyState !== WebSocket.OPEN) return;

    const wasOpened = this.state.opened;
    if (wasOpened) {
      terminal.reset();
      this.lineNumberHighlighter?.clear();
    }

    // connection is healthy again, reset the backoff and the flow control state
    this.reconnectAttempts = 0;
    this.flowPaused = false;
    this.pendingBytes = 0;

    this.setState((prev) => ({ ...prev, opened: true }));

    // use the real terminal size, fit() already ran before connect()
    const cols = terminal.cols;
    const rows = terminal.rows;
    // never send null, ttyd expects a string token
    socket.send(textEncoder.encode(JSON.stringify({ AuthToken: this.token ?? "", columns: cols, rows: rows })));

    for (const command of this.props.initialCommands ?? []) {
      this.socket?.send(textEncoder.encode(CommandClient.INPUT + command));
    }
    if (this.type === CmdTypes.SET_TIME && this.provider) this.sendTimeSync();
    this.safeFit();
  }

  private async sendTimeSync(): Promise<void> {
    if (this.isUnmounting) return;
    if (!this.provider || !this.remoteProvider) return;
    if (this.timeSyncIterations >= TIME_SYNC_MAX_ITERATIONS) {
      console.warn("[ttyd] time sync aborted: max iterations reached");
      return;
    }
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
    // show the hint overlay again, onclose handles the reconnect
    this.setState((prev) => ({ ...prev, opened: false }));
  }

  private onSocketData(event: MessageEvent): void {
    // guard: messages can still arrive while the component is going away
    if (this.isUnmounting || !this.terminal) return;

    const rawData = event.data as ArrayBuffer;
    // empty frames would decode to command "\0"
    if (!rawData || rawData.byteLength === 0) return;

    const cmd = String.fromCharCode(new Uint8Array(rawData, 0, 1)[0]);
    const payload = rawData.slice(1);

    switch (cmd) {
      case Command.OUTPUT: {
        // streaming decode, multi byte sequences may be split across frames
        const decoded = this.textDecoder.decode(payload, { stream: true });
        this.props.onIncomingData?.(decoded);
        this.handleOutput(decoded, payload.byteLength);
        break;
      }
      case Command.SET_WINDOW_TITLE: {
        const title = this.controlDecoder.decode(payload);
        this.title = this.props.clientOptions?.titleFixed || title;
        this.props.onTitleChange?.(this.title);
        break;
      }
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
  }

  /** Writes output and throttles the server while the write queue grows */
  private handleOutput(decoded: string, byteLength: number): void {
    this.pendingBytes += byteLength;
    if (!this.flowPaused && this.pendingBytes > FLOW_CONTROL_HIGH_WATER) {
      this.flowPaused = true;
      this.pause();
    }

    // scan only after the chunk has been parsed, cursor position is valid then
    this.terminal?.write(decoded, () => {
      this.pendingBytes = Math.max(0, this.pendingBytes - byteLength);
      if (this.flowPaused && this.pendingBytes < FLOW_CONTROL_LOW_WATER) {
        this.flowPaused = false;
        this.resume();
      }
      this.scheduleLineNumberHighlight();
      this.handleTimeSyncEcho(decoded);
    });
  }

  /** Time sync state machine, driven by the shell echo */
  private handleTimeSyncEcho(decoded: string): void {
    if (this.isUnmounting) return;
    if (this.type !== CmdTypes.SET_TIME || !this.provider) return;

    if (decoded.startsWith("sudo ")) {
      this.sudoRequested = true;
      return;
    }
    if (this.sudoRequested && decoded.startsWith("date set ok")) {
      this.sudoRequested = false;
      this.sendTimeSync();
    }
  }

  private createAdvButtons(): JSX.Element {
    return (
      <Stack direction="row" alignItems="center" flexShrink={0}>
        <Tooltip
          title={
            <Stack spacing={0.5}>
              <Typography variant="body2" fontWeight="bold" fontSize="inherit">
                Toggle Search Bar (Ctrl+F)
              </Typography>
            </Stack>
          }
          placement="bottom"
          disableInteractive
        >
          <ToggleButton
            size="small"
            value="showExplorer"
            selected={this.state.showSearchBar}
            sx={{ height: "1em" }}
            onChange={() => {
              this.setState((prev) => ({ ...prev, showSearchBar: !prev.showSearchBar }));
            }}
          >
            <SearchIcon sx={{ fontSize: "inherit" }} fontSize="inherit" />
          </ToggleButton>
        </Tooltip>
        <Tooltip
          title={
            <Stack spacing={0.5}>
              <Typography variant="body2" fontWeight="bold" fontSize="inherit">
                Toggle line number highlighting (Ctrl+L)
              </Typography>
            </Stack>
          }
          placement="bottom"
          disableInteractive
        >
          <ToggleButton
            size="small"
            value="showExplorer"
            selected={this.state.highlightLineNumbers}
            sx={{ height: "1em" }}
            onChange={() => {
              this.toggleHighlightLineNumbers();
            }}
          >
            <MoneyIcon sx={{ fontSize: "inherit" }} fontSize="inherit" />
          </ToggleButton>
        </Tooltip>
      </Stack>
    );
  }

  render(): JSX.Element {
    const { state } = this;
    return (
      <Stack width="100%" height="100%" sx={{ minHeight: 0, overflow: "hidden", position: "relative" }}>
        {!state.opened && (
          <Box sx={{ position: "absolute", inset: 0, zIndex: 2 }}>
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
          </Box>
        )}

        {state.resizeOverlay && (
          <Box
            sx={{
              position: "absolute",
              top: 8,
              right: 8,
              zIndex: 3,
              px: 1,
              py: 0.25,
              borderRadius: 1,
              pointerEvents: "none",
              backgroundColor: "rgba(0,0,0,0.6)",
              color: "#fff",
              fontFamily: "monospace",
              fontSize: "0.75rem",
            }}
          >
            {state.resizeOverlay}
          </Box>
        )}

        <Stack direction="row" alignItems="center" flexShrink={0}>
          {this.props.buttonLocation === BUTTON_LOCATIONS.LEFT && this.createAdvButtons()}
          <Typography
            variant="body2"
            flexGrow={1}
            // noWrap
            sx={{
              color: "text.secondary",
              pl: 0.5,
              fontSize: "0.7em",
              fontFamily: "monospace",
              borderLeftColor: "#2b2b2b",
              borderLeftStyle: "solid",
            }}
          >
            {this.props.initialCommands[this.props.initialCommands.length - 1]?.split(";").slice(-1)[0] || ""}
          </Typography>
          {this.props.buttonLocation === BUTTON_LOCATIONS.RIGHT && this.createAdvButtons()}
        </Stack>

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
          ref={(c: HTMLElement | null) => {
            this.container = c;
          }}
          width="100%"
          flexGrow={1}
          // minHeight:0 is required so flexbox may shrink this item below its content height
          minHeight={0}
          // scrolling is handled by xterm itself (.xterm-viewport)
          overflow="hidden"
          sx={{
            // "& .xterm": { padding: "0 2px" },
            // style only, never touch overflow: xterm keeps scrollTop in sync itself
            "& .xterm-viewport": {
              scrollbarWidth: "thin",
              scrollbarColor: "rgba(255,255,255,0.25) transparent",
            },
            // keep the search decorations of the overview ruler clickable
            "& .xterm-decoration-overview-ruler": { zIndex: 10 },
          }}
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

    // reflow moves text between lines, markers keep their old index:
    // all decorations must be rebuilt
    this.lineNumberHighlighter?.clear();
    this.scheduleLineNumberHighlight();
    this.showResizeOverlay(size.cols, size.rows);

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
        console.log("[ttyd] CTRL+D intercepted, closing session");
        onCtrlD(wsUrl, tokenUrl);
        // handled by the callback, do not forward the EOT byte
        return;
      }
    }

    const { socket, textEncoder } = this;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(textEncoder.encode(CommandClient.INPUT + data));
    }
  }
}
