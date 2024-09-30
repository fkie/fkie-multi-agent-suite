// import { electronAPI } from "@electron-toolkit/preload";
import {
  EditorManagerEvents,
  FileRange,
  ShutdownManagerEvents,
  TEditorManager,
  TerminateCallback,
  TShutdownManager,
  FileRangeCallback,
  EditorCloseCallback,
  TSubscriberManager,
  SubscriberCloseCallback,
  SubscriberManagerEvents,
} from "@/types";
import { contextBridge, ipcRenderer } from "electron";
import { ICredential } from "../main/models/ICredential";

// Custom APIs for renderer
// const api = {};

// Use `contextBridge` APIs to expose Electron APIs to
// renderer only if context isolation is enabled, otherwise
// just add to the DOM global.
if (process.contextIsolated) {
  try {
    // contextBridge.exposeInMainWorld("electron", electronAPI);
    // contextBridge.exposeInMainWorld("api", api);
    // Register Password Manager
    contextBridge.exposeInMainWorld("PasswordManager", {
      setPassword: (service: string, account: string, password: string) =>
        ipcRenderer.invoke("PasswordManager:setPassword", service, account, password),

      deletePassword: (service: string, account: string) =>
        ipcRenderer.invoke("PasswordManager:deletePassword", service, account),
    });

    // TODO remove SFTP if websocket ros.file.get and ros.file.save works
    // Register SFTP Manager
    // contextBridge.exposeInMainWorld('FileManagerWrapper', {
    //   checkPassword: (credential: ICredential) =>
    //     ipcRenderer.invoke('FileManagerWrapper:checkPassword', credential),

    //   exist: (credential: ICredential, path: string) =>
    //     ipcRenderer.invoke('FileManagerWrapper:exist', credential, path),

    //   stat: (credential: ICredential, path: string) =>
    //     ipcRenderer.invoke('FileManagerWrapper:stat', credential, path),

    //   get: (credential: ICredential, path: string) =>
    //     ipcRenderer.invoke('FileManagerWrapper:get', credential, path),

    //   put: (credential: ICredential, content: string, path: string) =>
    //     ipcRenderer.invoke('FileManagerWrapper:put', credential, content, path),
    // });

    // Register Command Executor
    contextBridge.exposeInMainWorld("CommandExecutor", {
      exec: (credential: ICredential, command: string) =>
        ipcRenderer.invoke("CommandExecutor:exec", credential, command),

      execTerminal: (credential: ICredential, title: string, command: string) =>
        ipcRenderer.invoke("CommandExecutor:execTerminal", credential, title, command),
    });

    // Register ROS Info
    contextBridge.exposeInMainWorld("ROSInfo", {
      getInfo: () => ipcRenderer.invoke("ROSInfo:getInfo"),
    });

    // Register System Info
    contextBridge.exposeInMainWorld("SystemInfo", {
      getInfo: () => ipcRenderer.invoke("SystemInfo:getInfo"),
    });

    // Register Multimaster Manager
    //    Validate first if ROS is available
    // if (['1', '2'].includes(`${sMultimasterManagerPreload.rosInfo.version}`)) {
    contextBridge.exposeInMainWorld("MultimasterManager", {
      startTerminalManager: (rosVersion: string, credential: ICredential, port?: number) =>
        ipcRenderer.invoke("MultimasterManager:startTerminalManager", rosVersion, credential, port),

      startMultimasterDaemon: (
        rosVersion: string,
        credential: ICredential,
        name?: string,
        networkId?: number,
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          "MultimasterManager:startMultimasterDaemon",
          rosVersion,
          credential,
          name,
          networkId,
          ros1MasterUri,
          forceStart
        ),

      startMasterDiscovery: (
        rosVersion: string,
        credential: ICredential,
        name?: string,
        networkId?: number,
        group?: string,
        heartbeatHz?: number,
        robotHosts?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          "MultimasterManager:startMasterDiscovery",
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
        credential: ICredential,
        name?: string,
        doNotSync?: string[],
        syncTopics?: string[],
        ros1MasterUri?: string,
        forceStart?: boolean
      ) =>
        ipcRenderer.invoke(
          "MultimasterManager:startMasterSync",
          rosVersion,
          credential,
          name,
          doNotSync,
          syncTopics,
          ros1MasterUri,
          forceStart
        ),

      startDynamicReconfigureClient: (name: string, rosMasterUri: string, credential?: ICredential | null) =>
        ipcRenderer.invoke("MultimasterManager:startDynamicReconfigureClient", name, rosMasterUri, credential),
    });

    // Expose protected methods that allow the renderer process to use
    // the ipcRenderer without exposing the entire object
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

    // register shutdown interface
    contextBridge.exposeInMainWorld("shutdownManager", {
      onTerminateSubprocesses: (callback: TerminateCallback) =>
        ipcRenderer.on(ShutdownManagerEvents.terminateSubprocesses, () => callback()),
      quitGui: () => ipcRenderer.invoke(ShutdownManagerEvents.quitGui),
    } as TShutdownManager);

    // register editor interface
    contextBridge.exposeInMainWorld("editorManager", {
      open: (id: string, host: string, port: number, path: string, rootLaunch: string, fileRange: FileRange) => {
        return ipcRenderer.invoke(EditorManagerEvents.open, id, host, port, rootLaunch, path, fileRange);
      },
      close: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.close, id);
      },
      changed: (id: string, path: string, changed: boolean) => {
        return ipcRenderer.invoke(EditorManagerEvents.changed, id, path, changed);
      },
      emitFileRange: (id: string, path: string, fileRange: FileRange) => {
        return ipcRenderer.invoke(EditorManagerEvents.emitFileRange, id, path, fileRange);
      },
      has: (id: string) => {
        return ipcRenderer.invoke(EditorManagerEvents.has, id);
      },
      onFileRange: (callback: FileRangeCallback) =>
        ipcRenderer.on(EditorManagerEvents.onFileRange, (_event, id, launchFile, fileRange) => {
          callback(id, launchFile, fileRange);
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

    contextBridge.exposeInMainWorld("electronAPI", {
      openFile: (path: string) => {
        return ipcRenderer.invoke("dialog:openFile", path);
      },
      // terminal interface
      openTerminal: (
        id: string,
        host: string,
        port: number,
        info: string,
        node: string,
        screen: string,
        cmd: string
      ) => {
        return ipcRenderer.invoke("terminal:open", id, host, port, info, node, screen, cmd);
      },
      closeTerminal: (id: string) => {
        return ipcRenderer.invoke("terminal:close", id);
      },
      hasTerminal: (id: string) => {
        return ipcRenderer.invoke("terminal:has", id);
      },
      onTerminalClose: (callback: (tabId: string) => Promise<boolean>) =>
        ipcRenderer.on("terminal:onClose", (_event, id) => {
          return callback(id);
        }),
    });
  } catch (error) {
    console.error(error);
  }
} else {
  // window.electron = electronAPI;
  // window.api = api;
}
