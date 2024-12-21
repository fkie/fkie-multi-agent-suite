import {
  EditorCloseCallback,
  EditorManagerEvents,
  FileRangeCallback,
  TEditorManager,
  TFileRange,
  TLaunchArg,
} from "@/types";
import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "path";
import windowStateKeeper from "../windowStateKeeper";

type TEditor = {
  window: BrowserWindow;
  changed: string[];
};

/**
 * Class EditorManager: handle communication with external editor
 */
class EditorManager implements TEditorManager {
  editors: { [id: string]: TEditor } = {};

  constructor() {}

  public onFileRange: (callback: FileRangeCallback) => void = () => {
    // implemented in preload script
  };
  public onClose: (callback: EditorCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers: () => void = () => {
    ipcMain.handle(EditorManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(
      EditorManagerEvents.open,
      (
        _event: Electron.IpcMainInvokeEvent,
        id: string,
        host: string,
        port: number,
        rootLaunch: string,
        launchFile: string,
        fileRange: TFileRange,
        launchArgs: TLaunchArg[]
      ) => {
        return this.open(id, host, port, rootLaunch, launchFile, fileRange, launchArgs);
      }
    );
    ipcMain.handle(EditorManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(
      EditorManagerEvents.changed,
      (_event: Electron.IpcMainInvokeEvent, id: string, launchFile: string, changed: boolean) => {
        return this.changed(id, launchFile, changed);
      }
    );
    ipcMain.handle(
      EditorManagerEvents.emitFileRange,
      (
        _event: Electron.IpcMainInvokeEvent,
        id: string,
        launchFile: string,
        fileRange: TFileRange,
        launchArgs: TLaunchArg[]
      ) => {
        return this.emitFileRange(id, launchFile, fileRange, launchArgs);
      }
    );
  };

  public has: (id: string) => Promise<boolean> = async (id) => {
    if (this.editors[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public emitFileRange: (
    id: string,
    path: string,
    fileRange: TFileRange,
    launchArgs: TLaunchArg[]
  ) => Promise<boolean> = async (id, path, fileRange, launchArgs) => {
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send(EditorManagerEvents.onFileRange, id, path, fileRange, launchArgs);
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public close: (id: string) => Promise<boolean> = async (id) => {
    if (this.editors[id]) {
      this.editors[id].window.destroy();
      delete this.editors[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public changed: (id: string, path: string, changed: boolean) => Promise<boolean> = async (id, path, changed) => {
    if (this.editors[id]) {
      if (this.editors[id].changed.includes(path)) {
        if (!changed) {
          this.editors[id].changed = this.editors[id].changed.filter((item) => item !== path);
        }
      } else if (changed) {
        this.editors[id].changed.push(path);
      }
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public open: (
    id: string,
    host: string,
    port: number,
    path: string,
    launchFile: string,
    fileRange: TFileRange,
    launchArgs: TLaunchArg[]
  ) => Promise<string | null> = async (id, host, port, path, launchFile, fileRange, launchArgs) => {
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send(EditorManagerEvents.onFileRange, id, path, fileRange, launchArgs);
      return Promise.resolve(null);
    }

    const editorWindowStateKeeper = await windowStateKeeper("editor");

    const editorWindow = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: editorWindowStateKeeper.x,
      y: editorWindowStateKeeper.y,
      width: editorWindowStateKeeper.width,
      height: editorWindowStateKeeper.height,
      icon: join(__dirname, "../../icon/crystal_clear_edit_launch.png"),
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.editors[id] = { window: editorWindow, changed: [] };
    // Track window state
    editorWindowStateKeeper.track(editorWindow);

    editorWindow.on("ready-to-show", () => {
      if (!editorWindow) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        editorWindow.minimize();
      } else {
        if (editorWindowStateKeeper.isMaximized) editorWindow.maximize();
        editorWindow.show();
      }
    });

    editorWindow.on("close", async (e) => {
      // send close request to the renderer
      e.preventDefault();
      this.editors[id].window.webContents.send(EditorManagerEvents.onClose, id);
    });

    editorWindow.on("closed", () => {
      delete this.editors[id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    if (is.dev && process.env.ELECTRON_RENDERER_URL) {
      // const fileRangeStr=`&sl=${fileRange.startLineNumber}&el=${fileRange.endLineNumber}&sc=${fileRange.startColumn}&ec=${fileRange.endColumn}`
      const fileRangeStr = fileRange ? `&range=${JSON.stringify(fileRange)}` : "";
      const launchArgsStr = launchArgs ? `&launchArgs=${JSON.stringify(launchArgs)}` : "";
      editorWindow.loadURL(
        `${process.env.ELECTRON_RENDERER_URL}/editor.html?id=${id}&host=${host}&port=${port}&root=${path}&path=${launchFile}${fileRangeStr}${launchArgsStr}`
      );
    } else {
      editorWindow.loadFile(join(__dirname, `../renderer/editor.html`), {
        query: {
          id: id,
          host: host,
          port: `${port}`,
          root: path,
          path: launchFile,
          range: `${JSON.stringify(fileRange)}`,
          launchArgs: `${JSON.stringify(launchArgs)}`,
        },
      });
    }
    return Promise.resolve(null);
  };
}

export default EditorManager;
