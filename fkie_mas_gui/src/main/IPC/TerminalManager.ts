import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import log from "electron-log";
import { join } from "path";
import { TerminalManagerEvents, ITerminalManager } from "@/types";
import { ARGUMENTS, getArgument, hasArgument } from "../CommandLineInterface";
import { ICredential } from "../models/ICredential";
import windowStateKeeper from "../windowStateKeeper";
import CommandExecutor from "./CommandExecutor";
import { ROSInfo } from "./ROSInfo";

interface ITerminal {
  window: BrowserWindow;
}

/**
 * Class TerminalManager: Allows to create terminal objects to interact with console
 */
class TerminalManager implements ITerminalManager {
  commandExecutor: CommandExecutor;

  rosInfo: ROSInfo;

  instances: { [id: string]: ITerminal } = {};

  constructor() {
    this.commandExecutor = new CommandExecutor();
    this.rosInfo = new ROSInfo();
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(TerminalManagerEvents.has, this.handleHasTerminal);
    ipcMain.handle(TerminalManagerEvents.open, this.handleOpenTerminal);
    ipcMain.handle(TerminalManagerEvents.close, this.handleCloseTerminal);
  };

  public handleHasTerminal: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.instances[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleCloseTerminal: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.instances[id]) {
      this.instances[id].window.destroy();
      delete this.instances[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleOpenTerminal: (
    _event: Electron.IpcMainInvokeEvent,
    id: string,
    host: string,
    port: number,
    info: string,
    node: string,
    screen: string,
    cmd: string
  ) => Promise<string | null> = async (_event, id, host, port, info, node, screen, cmd) => {
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
      const cmdStr = screen ? `&cmd=${cmd}` : "";
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

  /**
   * Creates a new terminal using given credentials and execute bash
   *
   * @param {ICredential} credential - Credentials to connect with
   * @param {number} port - TTYD Port
   * @return {string} Returns result
   */
  public spawnTerminal: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    port?: number
  ) => Promise<{ result: boolean; message: string }> = (rosVersion = null, credential = null, port = undefined) => {
    let version = rosVersion;
    if (!version) {
      version = this.rosInfo.version === "1" ? "1" : "2";
      log.debug(`use ROS version: ${version}`);
    }
    const portNumber = port || getArgument(ARGUMENTS.TTYD_PORT);

    const ttydCmd = `${this.getPathTTY()} --writable --port ${portNumber} bash`;

    let cmd = "";
    if (version === "1") {
      cmd = `rosrun fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else if (version === "2") {
      cmd = `ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else {
      return Promise.resolve({
        result: false,
        message: "Could not start [ttyd], ROS is not available",
      });
    }

    log.info(`Starting [ttyd-${portNumber}]: [${cmd}]`);
    return this.commandExecutor.exec(credential, cmd);
  };

  /**
   * Returns the path of the suitable executable for the current platform
   *
   * @return {string} Path to the executable
   */
  private getPathTTY: () => string = () => {
    if (hasArgument(ARGUMENTS.TTYD_PATH)) {
      const path = getArgument(ARGUMENTS.TTYD_PATH);
      if (path) return path;
    }

    // TODO Add executables and paths for windows and MAC
    return "ttyd";
  };
}

export default TerminalManager;
