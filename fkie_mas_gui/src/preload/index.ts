// import { electronAPI } from "@electron-toolkit/preload";
import {
  AuCheckingForUpdateCallback,
  AuDownloadProgressCallback,
  AutoUpdateManagerEvents,
  AuUpdateAvailableCallback,
  AuUpdateDownloadedCallback,
  AuUpdateErrorCallback,
  CommandExecutorEvents,
  CommandLineEvents,
  DialogManagerEvents,
  EditorCloseCallback,
  EditorManagerEvents,
  FileRangeCallback,
  PublishCloseCallback,
  PublishManagerEvents,
  ServiceCloseCallback,
  ServiceManagerEvents,
  ShutdownManagerEvents,
  SubscriberCloseCallback,
  SubscriberManagerEvents,
  TAutoUpdateManager,
  TCommandExecutor,
  TCommandLine,
  TEditorConfig,
  TEditorManager,
  TerminalCloseCallback,
  TerminalManagerEvents,
  TerminateCallback,
  TFileRange,
  TLaunchArg,
  TParameterRequest,
  TPublisherConfig,
  TPublishManager,
  TRosInfo,
  TServiceManager,
  TShutdownManager,
  TSubscriberConfig,
  TSubscriberManager,
  TSystemInfo,
  TTerminalManager,
} from "@/types";
import { TServiceConfig } from "@/types/ServiceManager";
import { TTerminalConfig } from "@/types/TerminalManager";
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

    contextBridge.exposeInMainWorld("ttydApi", {
      fetchToken: (url: string): Promise<string> => ipcRenderer.invoke("ttyd:fetchToken", url),
    });

    contextBridge.exposeInMainWorld("autoUpdate", {
      checkForUpdate: () => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.checkForUpdate);
      },
      quitAndInstall: () => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.quitAndInstall);
      },

      setChannel: (channelType: "prerelease" | "release") => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.setChannel, channelType);
      },

      isAppImage: () => {
        return ipcRenderer.invoke(AutoUpdateManagerEvents.isAppImage);
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

    // Register Command Line
    contextBridge.exposeInMainWorld("commandLine", {
      getArgument: (name: string) => ipcRenderer.invoke(CommandLineEvents.getArgument, name),
    } as TCommandLine);

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
      open: (props: TEditorConfig) => {
        return ipcRenderer.invoke(EditorManagerEvents.open, props);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.close, id);
      },
      changed: (id: string, path: string, changed: boolean) => {
        return ipcRenderer.invoke(EditorManagerEvents.changed, id, path, changed);
      },
      emitFileRange: (
        id: string,
        path: string,
        fileRange: TFileRange,
        launchArgs: TLaunchArg[],
        selectParameter: TParameterRequest
      ) => {
        return ipcRenderer.invoke(EditorManagerEvents.emitFileRange, id, path, fileRange, launchArgs, selectParameter);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.has, id);
      },
      onFileRange: (callback: FileRangeCallback) =>
        ipcRenderer.on(
          EditorManagerEvents.onFileRange,
          (_event, id, launchFile, fileRange, launchArgs, selectParameter) => {
            callback(id, launchFile, fileRange, launchArgs, selectParameter);
          }
        ),
      onClose: (callback: EditorCloseCallback) =>
        ipcRenderer.on(EditorManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TEditorManager);

    contextBridge.exposeInMainWorld("publishManager", {
      // publisher interface
      start: (props: TPublisherConfig) => {
        return ipcRenderer.invoke(PublishManagerEvents.start, props);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(PublishManagerEvents.close, id);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(PublishManagerEvents.has, id);
      },
      onClose: (callback: PublishCloseCallback) =>
        ipcRenderer.on(PublishManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TPublishManager);

    contextBridge.exposeInMainWorld("subscriberManager", {
      // subscriber interface
      open: (props: TSubscriberConfig) => {
        return ipcRenderer.invoke(SubscriberManagerEvents.open, props);
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

    contextBridge.exposeInMainWorld("serviceManager", {
      // service interface
      start: (props: TServiceConfig) => {
        return ipcRenderer.invoke(ServiceManagerEvents.start, props);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(ServiceManagerEvents.close, id);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(ServiceManagerEvents.has, id);
      },
      onClose: (callback: ServiceCloseCallback) =>
        ipcRenderer.on(ServiceManagerEvents.onClose, (_event, id) => {
          return callback(id);
        }),
    } as TServiceManager);

    contextBridge.exposeInMainWorld("terminalManager", {
      // terminal interface
      open: (props: TTerminalConfig) => {
        return ipcRenderer.invoke(TerminalManagerEvents.open, props);
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
      openDirectory: (path: string) => {
        return ipcRenderer.invoke(DialogManagerEvents.openDirectory, path);
      },
    });
  } catch (error) {
    console.error(error);
  }
} else {
  // window.electron = electronAPI;
  // window.api = api;
}
