import { join } from "node:path";
import editorIcon from "@public/google_edit_document.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import {
  EditorCloseCallback,
  EditorManagerEvents,
  FileRangeCallback,
  TEditorConfig,
  TEditorManager,
  TFileRange,
  TLaunchArg,
  TParameterRequest,
} from "@/types";
import windowStateKeeper from "../windowStateKeeper";
import { openUrl } from ".";

type TEditor = {
  window: BrowserWindow;
  changed: string[];
};

/**
 * Class EditorManager: handle communication with external editor
 */
export default class EditorManager implements TEditorManager {
  editors: { [id: string]: TEditor } = {};

  public onFileRange: (callback: FileRangeCallback) => void = () => {
    // implemented in preload script
  };
  public onClose: (callback: EditorCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers(): void {
    ipcMain.handle(EditorManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(EditorManagerEvents.open, (_event: Electron.IpcMainInvokeEvent, props: TEditorConfig) => {
      return this.open(props);
    });
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
        launchArgs: TLaunchArg[],
        selectParameter?: TParameterRequest
      ) => {
        return this.emitFileRange(id, launchFile, fileRange, launchArgs, selectParameter);
      }
    );
  }

  public has: (id: string) => Promise<boolean> = async (id) => {
    if (this.editors[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public emitFileRange: (
    id: string,
    path: string,
    fileRange: TFileRange | null,
    launchArgs: TLaunchArg[],
    selectParameter?: TParameterRequest
  ) => Promise<boolean> = async (id, path, fileRange, launchArgs, selectParameter) => {
    if (this.editors[id]) {
      this.editors[id].window.restore();
      this.editors[id].window.focus();
      this.editors[id].window.webContents.send(
        EditorManagerEvents.onFileRange,
        id,
        path,
        fileRange,
        launchArgs,
        selectParameter
      );
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

  public open: (props: TEditorConfig) => Promise<string | null> = async (props) => {
    if (this.editors[props.id]) {
      this.editors[props.id].window.restore();
      this.editors[props.id].window.focus();
      this.editors[props.id].window.webContents.send(
        EditorManagerEvents.onFileRange,
        props.id,
        props.path,
        props.fileRange,
        props.launchArgs,
        props.selectParameter
      );
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
      icon: editorIcon,
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.editors[props.id] = { window: editorWindow, changed: [] };
    // Track window state
    editorWindowStateKeeper.track(editorWindow);

    editorWindow.on("ready-to-show", () => {
      if (!editorWindow) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        editorWindow.minimize();
      } else {
        editorWindow.show();
      }
    });

    editorWindow.on("close", async (e) => {
      // send close request to the renderer
      e.preventDefault();
      this.editors[props.id].window.webContents.send(EditorManagerEvents.onClose, props.id);
    });

    editorWindow.on("closed", () => {
      delete this.editors[props.id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    openUrl(editorWindow, "editor", props);
    return Promise.resolve(null);
  };
}
