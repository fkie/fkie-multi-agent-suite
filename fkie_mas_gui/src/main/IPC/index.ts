import { dialog, ipcMain } from 'electron'
import { ICredential } from '../models/ICredential'
import AutoUpdateManager from './AutoUpdateManager'
import CommandExecutor from './CommandExecutor'
import DialogManager from './DialogManager'
import MultimasterManager from './MultimasterManager'
import PasswordManager from './PasswordManager'
import { IROSInfo, ROSInfo } from './ROSInfo'
import ShutdownInterface from './ShutdownInterface'
import { ISystemInfo, SystemInfo } from './SystemInfo'
import TerminalManager from './TerminalManager'

const sPasswordManager = new PasswordManager()
const sCommandExecutor = new CommandExecutor()
const sMultimasterManager = new MultimasterManager()

export const registerHandlers = (): void => {
  // Password Manager
  ipcMain.handle(
    'PasswordManager:setPassword',
    (_event, service: string, account: string, password: string) => {
      return sPasswordManager.setPassword(service, account, password)
    }
  )

  ipcMain.handle('PasswordManager:deletePassword', (_event, service: string, account: string) => {
    return sPasswordManager.deletePassword(service, account)
  })

  // SSH Manager
  ipcMain.handle('CommandExecutor:exec', (_event, credential: ICredential, command: string) => {
    return sCommandExecutor.exec(credential, command)
  })

  // SSH Manager
  ipcMain.handle(
    'CommandExecutor:execTerminal',
    (_event, credential: ICredential, title: string, command: string) => {
      return sCommandExecutor.execTerminal(credential, title, command)
    }
  )

  // ROSInfo
  ipcMain.handle('ROSInfo:getInfo', () => {
    return new ROSInfo().getInfo()
  })

  // ROSInfo
  ipcMain.handle('SystemInfo:getInfo', () => {
    return new SystemInfo().getInfo()
  })

  // Multimaster IPC methods
  //    Validate if ROS is available
  // if (['1', '2'].includes(`${sMultimasterManager.rosInfo.version}`)) {
  ipcMain.handle(
    'MultimasterManager:startTerminalManager',
    (_event, rosVersion: string, credential: ICredential, port?: number) => {
      return sMultimasterManager.startTerminalManager(rosVersion, credential, port)
    }
  )

  ipcMain.handle(
    'MultimasterManager:startMultimasterDaemon',
    (_event, rosVersion: string, credential: ICredential, name?: string, networkId?: number) => {
      return sMultimasterManager.startMultimasterDaemon(rosVersion, credential, name, networkId)
    }
  )

  ipcMain.handle(
    'MultimasterManager:startMasterDiscovery',
    (
      _event,
      rosVersion: string,
      credential: ICredential,
      name?: string,
      port?: number,
      group?: string,
      heartbeatHz?: number,
      robotHosts?: string[]
    ) => {
      return sMultimasterManager.startMasterDiscovery(
        rosVersion,
        credential,
        name,
        port,
        group,
        heartbeatHz,
        robotHosts
      )
    }
  )

  ipcMain.handle(
    'MultimasterManager:startMasterSync',
    (
      _event,
      rosVersion: string,
      credential: ICredential,
      name?: string,
      doNotSync?: string[],
      syncTopics?: string[]
    ) => {
      return sMultimasterManager.startMasterSync(
        rosVersion,
        credential,
        name,
        doNotSync,
        syncTopics
      )
    }
  )

  async function handleFileOpen(
    _event: Electron.IpcMainInvokeEvent,
    path: string
  ): Promise<string | null> {
    const { canceled, filePaths } = await dialog.showOpenDialog({
      defaultPath: path
    })
    if (!canceled) {
      return filePaths[0]
    }
    return null
  }
  ipcMain.handle('dialog:openFile', handleFileOpen)
}

export {
  AutoUpdateManager,
  DialogManager,
  MultimasterManager,
  PasswordManager,
  ROSInfo,
  ShutdownInterface,
  TerminalManager
}
export type { IROSInfo, ISystemInfo }
