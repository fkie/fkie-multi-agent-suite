// import { electronAPI } from "@electron-toolkit/preload";
import {
  AuCheckingForUpdateCallback,
  AuDownloadProgressCallback,
  AutoUpdateManagerEvents,
  AuUpdateAvailableCallback,
  AuUpdateDownloadedCallback,
  AuUpdateErrorCallback,
  CommandExecutorEvents,
  DialogManagerEvents,
  EditorCloseCallback,
  EditorManagerEvents,
  FileRangeCallback,
  ShutdownManagerEvents,
  SubscriberCloseCallback,
  SubscriberManagerEvents,
  TAutoUpdateManager,
  TCommandExecutor,
  TEditorManager,
  TerminalCloseCallback,
  TerminalManagerEvents,
  TerminateCallback,
  TFileRange,
  TLaunchArgs,
  TRosInfo,
  TShutdownManager,
  TSubscriberManager,
  TSystemInfo,
  TTerminalManager,
} from "@/types";
import { contextBridge, ipcRenderer } from "electron";
import { ConnectConfig } from "ssh2";

// Custom APIs for renderer
// const api = {};

// Use `contextBridge` APIs to expose Electron APIs to
// renderer only if context isolation is enabled, otherwise
// just add to the DOM global.
if (process.contextIsolated) {
  try {
    // contextBridge.exposeInMainWorld("electron", electronAPI);
    // contextBridge.exposeInMainWorld("api", api);

    // Expose protected methods that allow the renderer process to use
    // the ipcRenderer without exposing the entire object
    contextBridge.exposeInMainWorld("autoUpdate", {
      checkForUpdate: () => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.checkForUpdate);
      },
      quitAndInstall: () => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.quitAndInstall);
      },
      onCheckingForUpdate: (callback: AuCheckingForUpdateCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onCheckingForUpdate, (_event, state) => {
          callback(state);
        }),
      onUpdateAvailable: (callback: AuUpdateAvailableCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onUpdateAvailable, (_event, info) => {
          callback(info);
        }),
      onUpdateNotAvailable: (callback: AuUpdateAvailableCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onUpdateNotAvailable, (_event, info) => {
          callback(info);
        }),
      onDownloadProgress: (callback: AuDownloadProgressCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onDownloadProgress, (_event, info) => {
          callback(info);
        }),
      onUpdateDownloaded: (callback: AuUpdateDownloadedCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onUpdateDownloaded, (_event, info) => {
          callback(info);
        }),
      onUpdateError: (callback: AuUpdateErrorCallback) =>
        ipcRenderer.on(AutoUpdateManagerEvents.onUpdateError, (_event, message) => {
          callback(message);
        }),
    } as TAutoUpdateManager);

    // Register Command Executor
    contextBridge.exposeInMainWorld("commandExecutor", {
      exec: (credential: ConnectConfig, command: string) =>
        ipcRenderer.invoke(CommandExecutorEvents.exec, credential, command),

      execTerminal: (credential: ConnectConfig, title: string, command: string) =>
        ipcRenderer.invoke(CommandExecutorEvents.execTerminal, credential, title, command),
    } as TCommandExecutor);

    // Register ROS Info
    contextBridge.exposeInMainWorld("rosInfo", {
      getInfo: () => ipcRenderer.invoke("rosInfo:getInfo"),
    } as TRosInfo);

    // Register System Info
    contextBridge.exposeInMainWorld("systemInfo", {
      getInfo: () => ipcRenderer.invoke("systemInfo:getInfo"),
    } as TSystemInfo);

    // register shutdown interface
    contextBridge.exposeInMainWorld("shutdownManager", {
      emitCloseAppRequest: () => ipcRenderer.invoke(ShutdownManagerEvents.emitCloseAppRequest),
      onCloseAppRequest: (callback: TerminateCallback) =>
        ipcRenderer.on(ShutdownManagerEvents.onCloseAppRequest, () => callback()),
      cancelCloseTimeout: () => ipcRenderer.invoke(ShutdownManagerEvents.cancelCloseTimeout),
      quitGui: () => ipcRenderer.invoke(ShutdownManagerEvents.quitGui),
    } as TShutdownManager);

    // register editor interface
    contextBridge.exposeInMainWorld("editorManager", {
      open: (
        id: string,
        host: string,
        port: number,
        path: string,
        rootLaunch: string,
        fileRange: TFileRange,
        launchArgs: TLaunchArgs
      ) => {
        return ipcRenderer.invoke(EditorManagerEvents.open, id, host, port, rootLaunch, path, fileRange, launchArgs);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.close, id);
      },
      changed: (id: string, path: string, changed: boolean) => {
        return ipcRenderer.invoke(EditorManagerEvents.changed, id, path, changed);
      },
      emitFileRange: (id: string, path: string, fileRange: TFileRange, launchArgs: TLaunchArgs) => {
        return ipcRenderer.invoke(EditorManagerEvents.emitFileRange, id, path, fileRange, launchArgs);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.has, id);
      },
      onFileRange: (callback: FileRangeCallback) =>
        ipcRenderer.on(EditorManagerEvents.onFileRange, (_event, id, launchFile, fileRange, launchArgs) => {
          callback(id, launchFile, fileRange, launchArgs);
        }),
      onClose: (callback: EditorCloseCallback) =>
        ipcRenderer.on(EditorManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TEditorManager);

    contextBridge.exposeInMainWorld("subscriberManager", {
      // subscriber interface
      open: (id: string, host: string, port: number, topic: string, showOptions: boolean, noData: boolean) => {
        return ipcRenderer.invoke(SubscriberManagerEvents.open, id, host, port, topic, showOptions, noData);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(SubscriberManagerEvents.close, id);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(SubscriberManagerEvents.has, id);
      },
      onClose: (callback: SubscriberCloseCallback) =>
        ipcRenderer.on(SubscriberManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TSubscriberManager);

    contextBridge.exposeInMainWorld("terminalManager", {
      // terminal interface
      open: (id: string, host: string, port: number, info: string, node: string, screen: string, cmd: string) => {
        return ipcRenderer.invoke(TerminalManagerEvents.open, id, host, port, info, node, screen, cmd);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(TerminalManagerEvents.close, id);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(TerminalManagerEvents.has, id);
      },
      onClose: (callback: TerminalCloseCallback) =>
        ipcRenderer.on(TerminalManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TTerminalManager);

    contextBridge.exposeInMainWorld("dialogManager", {
      openFile: (path: string) => {
        return ipcRenderer.invoke(DialogManagerEvents.openFile, path);
      },
    });
  } catch (error) {
    console.error(error);
  }
} else {
  // window.electron = electronAPI;
  // window.api = api;
}
