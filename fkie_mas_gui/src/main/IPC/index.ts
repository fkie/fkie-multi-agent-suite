import { TCredential } from "@/types";
import { ipcMain } from "electron";
import AutoUpdateManager from "./AutoUpdateManager";
import CommandExecutor from "./CommandExecutor";
import DialogManager from "./DialogManager";
import EditorManager from "./EditorManager";
import LaunchManager from "./LaunchManager";
import PasswordManager from "./PasswordManager";
import { ROSInfo } from "./ROSInfo";
import ShutdownManager from "./ShutdownManager";
import SubscriberManager from "./SubscriberManager";
import { SystemInfo } from "./SystemInfo";
import TerminalManager from "./TerminalManager";

const sPasswordManager = new PasswordManager();
const sCommandExecutor = new CommandExecutor();
const launchManager = new LaunchManager();
const editorManager = new EditorManager();
const subscriberManager = new SubscriberManager();
const terminalManager = new TerminalManager();

export const registerHandlers = (): void => {
  editorManager.registerHandlers();
  launchManager.registerHandlers();
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


};

export {
  AutoUpdateManager,
  DialogManager,
  EditorManager,
  LaunchManager,
  PasswordManager,
  ROSInfo,
  ShutdownManager,
  SubscriberManager,
  TerminalManager,
};
