import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "path";
import windowStateKeeper from "../windowStateKeeper";

interface IEditor {
  window: BrowserWindow;
  changed: string[];
}

/**
 * Class EditorManager: handle communication with external editor
 */
class EditorManager {
  editors: { [id: string]: IEditor } = {};

  constructor() {}

  public registerHandlers: () => void = () => {
    ipcMain.handle("editor:has", this.handleHasEditor);
    ipcMain.handle("editor:open", this.handleEditorOpen);
    ipcMain.handle("editor:close", this.handleEditorClose);
    ipcMain.handle("editor:changed", this.handleEditorChanged);
    ipcMain.handle("editor:emitFileRange", this.handleEditorFileRange);
  };

  public handleHasEditor: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.editors[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleEditorFileRange: (
    _event: Electron.IpcMainInvokeEvent,
    id: string,
    launchFile: string,
    fileRange: { startLineNumber: number; endLineNumber: number; startColumn: number; endColumn: number }
  ) => Promise<null> = async (_event, id, launchFile, fileRange) => {
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send("editor:onFileRange", id, launchFile, fileRange);
    }
    return Promise.resolve(null);
  };

  public handleEditorClose: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.editors[id]) {
      this.editors[id].window.destroy();
      delete this.editors[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleEditorChanged: (
    _event: Electron.IpcMainInvokeEvent,
    id: string,
    launchFile: string,
    changed: boolean
  ) => Promise<boolean> = async (_event, id, launchFile, changed) => {
    if (this.editors[id]) {
      if (this.editors[id].changed.includes(launchFile)) {
        if (!changed) {
          this.editors[id].changed = this.editors[id].changed.filter((item) => item !== launchFile);
        }
      } else if (changed) {
        this.editors[id].changed.push(launchFile);
      }
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleEditorOpen: (
    _event: Electron.IpcMainInvokeEvent,
    id: string,
    host: string,
    port: number,
    rootLaunch: string,
    launchFile: string,
    fileRange: { startLineNumber: number; endLineNumber: number; startColumn: number; endColumn: number }
  ) => Promise<string | null> = async (_event, id, host, port, rootLaunch, launchFile, fileRange) => {
    // if (isDebug) {
    //   await installExtensions()
    // }
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send("editor:onFileRange", id, launchFile, fileRange);
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
      this.editors[id].window.webContents.send("editor:onClose", id);
    });

    editorWindow.on("closed", () => {
      delete this.editors[id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    if (is.dev && process.env.ELECTRON_RENDERER_URL) {
      editorWindow.loadURL(
        fileRange
          ? `${process.env.ELECTRON_RENDERER_URL}/editor.html?id=${id}&host=${host}&port=${port}&root=${rootLaunch}&path=${launchFile}&sl=${fileRange.startLineNumber}&el=${fileRange.endLineNumber}&sc=${fileRange.startColumn}&ec=${fileRange.endColumn}`
          : `${process.env.ELECTRON_RENDERER_URL}/editor.html?id=${id}&host=${host}&port=${port}&root=${rootLaunch}&path=${launchFile}`
      );
    } else {
      editorWindow.loadFile(join(__dirname, `../renderer/editor.html`), {
        query: fileRange
          ? {
              id: id,
              host: host,
              port: `${port}`,
              root: rootLaunch,
              path: launchFile,
              sl: `${fileRange.startLineNumber}`,
              el: `${fileRange.endLineNumber}`,
              sc: `${fileRange.startColumn}`,
              ec: `${fileRange.endColumn}`,
            }
          : {
              id: id,
              host: host,
              port: `${port}`,
              root: rootLaunch,
              path: launchFile,
            },
      });
    }
    return Promise.resolve(null);
  };

}

export default EditorManager;
