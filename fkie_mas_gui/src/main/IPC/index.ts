import { ipcMain } from 'electron';
import { ICredential } from '../models/ICredential';
import CommandExecutor from './CommandExecutor';
// import FileManagerWrapper from './FileManagerWrapper';
import MultimasterManager from './MultimasterManager';
import PasswordManager from './PasswordManager';
import { IROSInfo, ROSInfo } from './ROSInfo';
import { ISystemInfo, SystemInfo } from './SystemInfo';
import TerminalManager from './TerminalManager';

const sPasswordManager = new PasswordManager();
// const fileManagerWrapper = new FileManagerWrapper();
const sCommandExecutor = new CommandExecutor();
const sMultimasterManager = new MultimasterManager();

export const registerHandlers = () => {
  // Password Manager
  ipcMain.handle(
    'PasswordManager:setPassword',
    (event, service: string, account: string, password: string) => {
      return sPasswordManager.setPassword(service, account, password);
    },
  );

  ipcMain.handle(
    'PasswordManager:deletePassword',
    (event, service: string, account: string) => {
      return sPasswordManager.deletePassword(service, account);
    },
  );

  // TODO remove SFTP if crossbar ros.file.get and ros.file.save works
  // // SFTP Manager
  // ipcMain.handle(
  //   'FileManagerWrapper:checkPassword',
  //   (event, credential: ICredential) => {
  //     return fileManagerWrapper.checkPassword(credential);
  //   }
  // );

  // ipcMain.handle(
  //   'FileManagerWrapper:exist',
  //   (event, credential: ICredential, path: string) => {
  //     return fileManagerWrapper.exist(credential, path);
  //   }
  // );

  // ipcMain.handle(
  //   'FileManagerWrapper:stat',
  //   (event, credential: ICredential, path: string) => {
  //     return fileManagerWrapper.stat(credential, path);
  //   }
  // );

  // ipcMain.handle(
  //   'FileManagerWrapper:get',
  //   (event, credential: ICredential, path: string) => {
  //     return fileManagerWrapper.get(credential, path);
  //   }
  // );

  // ipcMain.handle(
  //   'FileManagerWrapper:put',
  //   (event, credential: ICredential, content: string, path: string) => {
  //     return fileManagerWrapper.put(credential, content, path);
  //   }
  // );

  // SSH Manager
  ipcMain.handle(
    'CommandExecutor:exec',
    (event, credential: ICredential, command: string) => {
      return sCommandExecutor.exec(credential, command);
    },
  );

  // ROSInfo
  ipcMain.handle('ROSInfo:getInfo', () => {
    return new ROSInfo().getInfo();
  });

  // ROSInfo
  ipcMain.handle('SystemInfo:getInfo', () => {
    return new SystemInfo().getInfo();
  });

  // Multimaster IPC methods
  //    Validate if ROS is available
  // if (['1', '2'].includes(`${sMultimasterManager.rosInfo.version}`)) {
  ipcMain.handle(
    'MultimasterManager:startTerminalManager',
    (event, rosVersion: string, credential: ICredential, port?: number) => {
      return sMultimasterManager.startTerminalManager(
        rosVersion,
        credential,
        port,
      );
    },
  );

  ipcMain.handle(
    'MultimasterManager:startMultimasterDaemon',
    (event, rosVersion: string, credential: ICredential, name?: string) => {
      return sMultimasterManager.startMultimasterDaemon(
        rosVersion,
        credential,
        name,
      );
    },
  );

  ipcMain.handle(
    'MultimasterManager:startMasterDiscovery',
    (
      event,
      rosVersion: string,
      credential: ICredential,
      name?: string,
      port?: number,
      group?: string,
      heartbeatHz?: number,
      robotHosts?: string[],
    ) => {
      return sMultimasterManager.startMasterDiscovery(
        rosVersion,
        credential,
        name,
        port,
        group,
        heartbeatHz,
        robotHosts,
      );
    },
  );

  ipcMain.handle(
    'MultimasterManager:startMasterSync',
    (
      event,
      rosVersion: string,
      credential: ICredential,
      name?: string,
      doNotSync?: string[],
      syncTopics?: string[],
    ) => {
      return sMultimasterManager.startMasterSync(
        rosVersion,
        credential,
        name,
        doNotSync,
        syncTopics,
      );
    },
  );
};
// };
//  FileManagerWrapper,

export {
  IROSInfo,
  ISystemInfo,
  MultimasterManager,
  PasswordManager,
  ROSInfo,
  TerminalManager,
};
