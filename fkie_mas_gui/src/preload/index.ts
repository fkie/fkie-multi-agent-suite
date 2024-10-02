// import { electronAPI } from "@electron-toolkit/preload";
import {
  CommandExecutorEvents,
  DialogManagerEvents,
  EditorCloseCallback,
  EditorManagerEvents,
  TFileRange,
  FileRangeCallback,
  LaunchManagerEvents,
  PasswordManagerEvents,
  ShutdownManagerEvents,
  SubscriberCloseCallback,
  SubscriberManagerEvents,
  TCommandExecutor,
  TCredential,
  TEditorManager,
  TerminalCloseCallback,
  TerminalManagerEvents,
  TerminateCallback,
  TLaunchManager,
  TPasswordManager,
  TRosInfo,
  TShutdownManager,
  TSubscriberManager,
  TSystemInfo,
  TTerminalManager,
  TLaunchArgs,
} from "@/types";
import { contextBridge, ipcRenderer } from "electron";

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
    // TODO: create autoUpdate type object
    contextBridge.exposeInMainWorld("autoUpdate", {
      send: (channel: string, data: unknown) => {
        // whitelist channels
        const validChannels = ["check-for-updates", "quit-and-install"];
        if (validChannels.includes(channel)) {
          ipcRenderer.send(channel, data);
        }
      },
      receive: (channel: string, func) => {
        const validChannels = [
          "checking-for-update",
          "update-available",
          "update-not-available",
          "download-progress",
          "update-downloaded",
          "update-error",
        ];
        if (validChannels.includes(channel)) {
          // Deliberately strip event as it includes `sender`
          ipcRenderer.on(channel, (_event, ...args) => {
            func(...args);
          });
        }
      },
    });

    // Register Password Manager
    contextBridge.exposeInMainWorld("passwordManager", {
      setPassword: (service: string, account: string, password: string) =>
        ipcRenderer.invoke(PasswordManagerEvents.setPassword, service, account, password),

      deletePassword: (service: string, account: string) =>
        ipcRenderer.invoke(PasswordManagerEvents.deletePassword, service, account),
    } as TPasswordManager);

    // Register Command Executor
    contextBridge.exposeInMainWorld("commandExecutor", {
      exec: (credential: TCredential, command: string) =>
        ipcRenderer.invoke(CommandExecutorEvents.exec, credential, command),

      execTerminal: (credential: TCredential, title: string, command: string) =>
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

    // Register launch Manager
    contextBridge.exposeInMainWorld("launchManager", {
      startTerminalManager: (rosVersion: string, credential: TCredential, port?: number) =>
        ipcRenderer.invoke(LaunchManagerEvents.startTerminalManager, rosVersion, credential, port),

      startDaemon: (
        rosVersion: string,
        credential: TCredential,
        name?: string,
        networkId?: number,
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          LaunchManagerEvents.startDaemon,
          rosVersion,
          credential,
          name,
          networkId,
          ros1MasterUri,
          forceStart
        ),

      startMasterDiscovery: (
        rosVersion: string,
        credential: TCredential,
        name?: string,
        networkId?: number,
        group?: string,
        heartbeatHz?: number,
        robotHosts?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          LaunchManagerEvents.startMasterDiscovery,
          rosVersion,
          credential,
          name,
          networkId,
          group,
          heartbeatHz,
          robotHosts,
          ros1MasterUri,
          forceStart
        ),

      startMasterSync: (
        rosVersion: string,
        credential: TCredential,
        name?: string,
        doNotSync?: string[],
        syncTopics?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          LaunchManagerEvents.startMasterSync,
          rosVersion,
          credential,
          name,
          doNotSync,
          syncTopics,
          ros1MasterUri,
          forceStart
        ),

      startDynamicReconfigureClient: (name: string, rosMasterUri: string, credential?: TCredential | null) =>
        ipcRenderer.invoke(LaunchManagerEvents.startDynamicReconfigureClient, name, rosMasterUri, credential),
    } as TLaunchManager);

    // register shutdown interface
    contextBridge.exposeInMainWorld("shutdownManager", {
      onTerminateSubprocesses: (callback: TerminateCallback) =>
        ipcRenderer.on(ShutdownManagerEvents.terminateSubprocesses, () => callback()),
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
