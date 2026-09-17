import { join } from "node:path";
import ioIcon from "@public/google_terminal.png?asset";
import logIcon from "@public/google_text_snippet.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import { TerminalCloseCallback, TerminalManagerEvents, TTerminalManager } from "@/types";
import { TTerminalConfig } from "@/types/TerminalManager";
import windowStateKeeper from "../windowStateKeeper";
import { openUrl } from ".";
import { ROSInfo } from "./ROSInfo";

type TTerminal = {
  window: BrowserWindow;
};

/**
 * Class TerminalManager: Allows to create terminal objects to interact with console
 */
export default class TerminalManager implements TTerminalManager {
  rosInfo: ROSInfo;

  instances: { [id: string]: TTerminal } = {};

  constructor() {
    this.rosInfo = new ROSInfo();
  }

  onClose: (callback: TerminalCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers(): void {
    ipcMain.handle(TerminalManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(TerminalManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(TerminalManagerEvents.open, (_event: Electron.IpcMainInvokeEvent, props: TTerminalConfig) => {
      return this.open(props);
    });
  }

  public has: (id: string) => Promise<boolean> = async (id) => {
    if (this.instances[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public close: (id: string) => Promise<boolean> = async (id) => {
    if (this.instances[id]) {
      this.instances[id].window.destroy();
      delete this.instances[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public open: (props: TTerminalConfig) => Promise<string | null> = async (props) => {
    // if (isDebug) {
    //   await installExtensions()
    // }
    if (this.instances[props.id]) {
      this.instances[props.id].window.restore();
      this.instances[props.id].window.focus();
      return Promise.resolve(null);
    }
    const editorWindowStateKeeper = await windowStateKeeper("editor");

    const window = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: editorWindowStateKeeper.x,
      y: editorWindowStateKeeper.y,
      width: editorWindowStateKeeper.width,
      height: editorWindowStateKeeper.height,
      icon: `${props.cmdType}` === "log" ? logIcon : ioIcon,
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[props.id] = { window: window };
    // Track window state
    editorWindowStateKeeper.track(window);

    window.on("ready-to-show", () => {
      if (!window) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        window.minimize();
      } else {
        window.show();
      }
    });

    window.on("close", async (e) => {
      // send close request to the renderer
      if (this.instances[props.id]) {
        e.preventDefault();
        this.instances[props.id].window.webContents.send(TerminalManagerEvents.onClose, props.id);
      }
    });

    window.on("closed", () => {
      delete this.instances[props.id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    openUrl(window, "terminal", props);
    return Promise.resolve(null);
  };
}
