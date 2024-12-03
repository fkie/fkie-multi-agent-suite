import { TerminalCloseCallback, TerminalManagerEvents, TTerminalManager } from "@/types";
import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "path";
import windowStateKeeper from "../windowStateKeeper";
import { ROSInfo } from "./ROSInfo";

type TTerminal = {
  window: BrowserWindow;
};

/**
 * Class TerminalManager: Allows to create terminal objects to interact with console
 */
class TerminalManager implements TTerminalManager {
  rosInfo: ROSInfo;

  instances: { [id: string]: TTerminal } = {};

  constructor() {
    this.rosInfo = new ROSInfo();
  }

  onClose: (callback: TerminalCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers: () => void = () => {
    ipcMain.handle(TerminalManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(TerminalManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(
      TerminalManagerEvents.open,
      (
        _event: Electron.IpcMainInvokeEvent,
        id: string,
        host: string,
        port: number,
        info: string,
        node: string,
        screen: string,
        cmd: string
      ) => {
        return this.open(id, host, port, info, node, screen, cmd);
      }
    );
  };

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

  public open: (
    id: string,
    host: string,
    port: number,
    info: string,
    node: string,
    screen: string,
    cmd: string
  ) => Promise<string | null> = async (id, host, port, info, node, screen, cmd) => {
    // if (isDebug) {
    //   await installExtensions()
    // }
    if (this.instances[id]) {
      this.instances[id].window.focus();
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
      icon: join(
        __dirname,
        `${info}` === "log" ? "../../icon/crystal_clear_show_log.png" : "../../icon/crystal_clear_show_io.png"
      ),
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[id] = { window: window };
    // Track window state
    editorWindowStateKeeper.track(window);

    window.on("ready-to-show", () => {
      if (!window) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        window.minimize();
      } else {
        if (editorWindowStateKeeper.isMaximized) window.maximize();
        window.show();
      }
    });

    window.on("close", async (e) => {
      // send close request to the renderer
      if (this.instances[id]) {
        e.preventDefault();
        this.instances[id].window.webContents.send(TerminalManagerEvents.onClose, id);
      }
    });

    window.on("closed", () => {
      delete this.instances[id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    if (is.dev && process.env.ELECTRON_RENDERER_URL) {
      const nodeStr = node ? `&node=${node}` : "";
      const screenStr = screen ? `&screen=${screen}` : "";
      const cmdStr = cmd ? `&cmd=${cmd}` : "";
      window.loadURL(
        `${process.env.ELECTRON_RENDERER_URL}/terminal.html?id=${id}&host=${host}&port=${port}&info=${info}${nodeStr}${screenStr}${cmdStr}`
      );
    } else {
      window.loadFile(join(__dirname, `../renderer/terminal.html`), {
        query: {
          id: id,
          host: host,
          port: `${port}`,
          info: `${info}`,
          node: node,
          screen: screen,
          cmd: cmd,
        },
      });
    }
    return Promise.resolve(null);
  };
}

export default TerminalManager;
