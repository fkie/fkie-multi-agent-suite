import { contextBridge, ipcRenderer } from 'electron';
import { ICredential } from './models/ICredential';

// import MultimasterManager from './IPC/MultimasterManager';
// const sMultimasterManagerPreload = new MultimasterManager();

// Register Password Manager
contextBridge.exposeInMainWorld('PasswordManager', {
  setPassword: (service: string, account: string, password: string) =>
    ipcRenderer.invoke(
      'PasswordManager:setPassword',
      service,
      account,
      password,
    ),

  deletePassword: (service: string, account: string) =>
    ipcRenderer.invoke('PasswordManager:deletePassword', service, account),
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
contextBridge.exposeInMainWorld('CommandExecutor', {
  exec: (credential: ICredential, command: string) =>
    ipcRenderer.invoke('CommandExecutor:exec', credential, command),

  execTerminal: (credential: ICredential, title: string, command: string) =>
    ipcRenderer.invoke(
      'CommandExecutor:execTerminal',
      credential,
      title,
      command,
    ),
});

// Register ROS Info
contextBridge.exposeInMainWorld('ROSInfo', {
  getInfo: () => ipcRenderer.invoke('ROSInfo:getInfo'),
});

// Register System Info
contextBridge.exposeInMainWorld('SystemInfo', {
  getInfo: () => ipcRenderer.invoke('SystemInfo:getInfo'),
});

// Register Multimaster Manager
//    Validate first if ROS is available
// if (['1', '2'].includes(`${sMultimasterManagerPreload.rosInfo.version}`)) {
contextBridge.exposeInMainWorld('MultimasterManager', {
  startTerminalManager: (
    rosVersion: string,
    credential: ICredential,
    port?: number,
  ) =>
    ipcRenderer.invoke(
      'MultimasterManager:startTerminalManager',
      rosVersion,
      credential,
      port,
    ),

  startMultimasterDaemon: (
    rosVersion: string,
    credential: ICredential,
    name?: string,
    networkId?: number,
  ) =>
    ipcRenderer.invoke(
      'MultimasterManager:startMultimasterDaemon',
      rosVersion,
      credential,
      name,
      networkId,
    ),

  startMasterDiscovery: (
    rosVersion: string,
    credential: ICredential,
    name?: string,
    port?: number,
    group?: string,
    heartbeatHz?: number,
    robotHosts?: string[],
  ) =>
    ipcRenderer.invoke(
      'MultimasterManager:startMasterDiscovery',
      rosVersion,
      credential,
      name,
      port,
      group,
      heartbeatHz,
      robotHosts,
    ),

  startMasterSync: (
    rosVersion: string,
    credential: ICredential,
    name?: string,
    doNotSync?: string[],
    syncTopics?: string[],
  ) =>
    ipcRenderer.invoke(
      'MultimasterManager:startMasterSync',
      rosVersion,
      credential,
      name,
      doNotSync,
      syncTopics,
    ),
});

// Expose protected methods that allow the renderer process to use
// the ipcRenderer without exposing the entire object
contextBridge.exposeInMainWorld('autoUpdate', {
  send: (channel: string, data: any) => {
    // whitelist channels
    const validChannels = ['check-for-updates', 'quit-and-install'];
    if (validChannels.includes(channel)) {
      ipcRenderer.send(channel, data);
    }
  },
  receive: (channel: string, func) => {
    const validChannels = [
      'checking-for-update',
      'update-available',
      'update-not-available',
      'download-progress',
      'update-downloaded',
      'update-error',
    ];
    if (validChannels.includes(channel)) {
      // Deliberately strip event as it includes `sender`
      ipcRenderer.on(channel, (event, ...args) => {
        func(...args);
      });
    }
  },
});

contextBridge.exposeInMainWorld('ShutdownInterface', {
  onTerminateSubprocesses: (callback: Function) =>
    ipcRenderer.on('ShutdownInterface:terminateSubprocesses', () => callback()),
  quitGui: () => ipcRenderer.invoke('ShutdownInterface:quitGui'),
});

contextBridge.exposeInMainWorld('electronAPI', {
  openFile: (path: string) => {
    return ipcRenderer.invoke('dialog:openFile', path);
  }
});
