import { TCredential } from "@/types";
import { ipcMain } from "electron";
import AutoUpdateManager from "./AutoUpdateManager";
import CommandExecutor from "./CommandExecutor";
import DialogManager from "./DialogManager";
import EditorManager from "./EditorManager";
import MultimasterManager from "./MultimasterManager";
import PasswordManager from "./PasswordManager";
import { ROSInfo } from "./ROSInfo";
import ShutdownManager from "./ShutdownManager";
import SubscriberManager from "./SubscriberManager";
import { SystemInfo } from "./SystemInfo";
import TerminalManager from "./TerminalManager";

const sPasswordManager = new PasswordManager();
const sCommandExecutor = new CommandExecutor();
const sMultimasterManager = new MultimasterManager();
const editorManager = new EditorManager();
const subscriberManager = new SubscriberManager();
const terminalManager = new TerminalManager();

export const registerHandlers = (): void => {
  editorManager.registerHandlers();
  subscriberManager.registerHandlers();
  terminalManager.registerHandlers();

  // Password Manager
  ipcMain.handle("PasswordManager:setPassword", (_event, service: string, account: string, password: string) => {
    return sPasswordManager.setPassword(service, account, password);
  });

  ipcMain.handle("PasswordManager:deletePassword", (_event, service: string, account: string) => {
    return sPasswordManager.deletePassword(service, account);
  });

  // SSH Manager
  ipcMain.handle("CommandExecutor:exec", (_event, credential: TCredential, command: string) => {
    return sCommandExecutor.exec(credential, command);
  });

  // SSH Manager
  ipcMain.handle("CommandExecutor:execTerminal", (_event, credential: TCredential, title: string, command: string) => {
    return sCommandExecutor.execTerminal(credential, title, command);
  });

  // ROSInfo
  ipcMain.handle("rosInfo:getInfo", () => {
    return new ROSInfo().getInfo();
  });

  // ROSInfo
  ipcMain.handle("systemInfo:getInfo", () => {
    return new SystemInfo().getInfo();
  });

  // Multimaster IPC methods
  //    Validate if ROS is available
  // if (['1', '2'].includes(`${sMultimasterManager.rosInfo.version}`)) {
  ipcMain.handle(
    "MultimasterManager:startTerminalManager",
    (_event, rosVersion: string, credential: TCredential, port?: number) => {
      return sMultimasterManager.startTerminalManager(rosVersion, credential, port);
    }
  );

  ipcMain.handle(
    "MultimasterManager:startMultimasterDaemon",
    (
      _event,
      rosVersion: string,
      credential: TCredential,
      name?: string,
      networkId?: number,
      ros1MasterUri?: string,
      forceStart?: boolean
    ) => {
      return sMultimasterManager.startMultimasterDaemon(
        rosVersion,
        credential,
        name,
        networkId,
        ros1MasterUri,
        forceStart
      );
    }
  );

  ipcMain.handle(
    "MultimasterManager:startMasterDiscovery",
    (
      _event,
      rosVersion: string,
      credential: TCredential,
      name?: string,
      port?: number,
      group?: string,
      heartbeatHz?: number,
      robotHosts?: string[],
      ros1MasterUri?: string,
      forceStart?: boolean
    ) => {
      return sMultimasterManager.startMasterDiscovery(
        rosVersion,
        credential,
        name,
        port,
        group,
        heartbeatHz,
        robotHosts,
        ros1MasterUri,
        forceStart
      );
    }
  );

  ipcMain.handle(
    "MultimasterManager:startMasterSync",
    (
      _event,
      rosVersion: string,
      credential: TCredential,
      name?: string,
      doNotSync?: string[],
      syncTopics?: string[],
      ros1MasterUri?: string,
      forceStart?: boolean
    ) => {
      return sMultimasterManager.startMasterSync(
        rosVersion,
        credential,
        name,
        doNotSync,
        syncTopics,
        ros1MasterUri,
        forceStart
      );
    }
  );

  ipcMain.handle(
    "MultimasterManager:startDynamicReconfigureClient",
    (_event, name: string, rosMasterUri: string, credential?: TCredential | null) => {
      return sMultimasterManager.startDynamicReconfigureClient(name, rosMasterUri, credential);
    }
  );
};

export {
  AutoUpdateManager,
  DialogManager,
  EditorManager,
  MultimasterManager,
  PasswordManager,
  ROSInfo,
  ShutdownManager,
  SubscriberManager,
  TerminalManager,
};
