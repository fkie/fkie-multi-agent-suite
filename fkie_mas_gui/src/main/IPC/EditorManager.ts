import { is } from "@electron-toolkit/utils";
import { IEditorManager, IEditor, EditorManagerEvents, TFileRange, TLaunchArgs } from "@/types";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "path";
import windowStateKeeper from "../windowStateKeeper";

/**
 * Class EditorManager: handle communication with external editor
 */
class EditorManager implements IEditorManager {
  editors: { [id: string]: IEditor } = {};

  constructor() {}

  public registerHandlers: () => void = () => {
    ipcMain.handle(EditorManagerEvents.has, this.handleHasEditor);
    ipcMain.handle(EditorManagerEvents.open, this.handleEditorOpen);
    ipcMain.handle(EditorManagerEvents.close, this.handleEditorClose);
    ipcMain.handle(EditorManagerEvents.changed, this.handleEditorChanged);
    ipcMain.handle(EditorManagerEvents.emitFileRange, this.handleEditorFileRange);
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
    fileRange: TFileRange,
    launchArgs: TLaunchArgs
  ) => Promise<null> = async (_event, id, launchFile, fileRange, launchArgs) => {
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send(EditorManagerEvents.onFileRange, id, launchFile, fileRange, launchArgs);
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
    fileRange: TFileRange,
    launchArgs: TLaunchArgs
  ) => Promise<string | null> = async (_event, id, host, port, rootLaunch, launchFile, fileRange, launchArgs) => {
    if (this.editors[id]) {
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send(EditorManagerEvents.onFileRange, id, launchFile, fileRange, launchArgs);
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
        `${process.env.ELECTRON_RENDERER_URL}/editor.html?id=${id}&host=${host}&port=${port}&root=${rootLaunch}&path=${launchFile}${fileRangeStr}${launchArgsStr}`
      );
    } else {
      editorWindow.loadFile(join(__dirname, `../renderer/editor.html`), {
        query: {
          id: id,
          host: host,
          port: `${port}`,
          root: rootLaunch,
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
