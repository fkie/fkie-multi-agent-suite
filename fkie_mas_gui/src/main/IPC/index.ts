import { ipcMain } from "electron";
import AutoUpdateManager from "./AutoUpdateManager";
import CommandExecutor from "./CommandExecutor";
import DialogManager from "./DialogManager";
import EditorManager from "./EditorManager";
import PasswordManager from "./PasswordManager";
import { ROSInfo } from "./ROSInfo";
import ShutdownManager from "./ShutdownManager";
import SubscriberManager from "./SubscriberManager";
import { SystemInfo } from "./SystemInfo";
import TerminalManager from "./TerminalManager";

const passwordManager = new PasswordManager();
const commandExecutor = new CommandExecutor();
const editorManager = new EditorManager();
const subscriberManager = new SubscriberManager();
const terminalManager = new TerminalManager();

export const registerHandlers = (): void => {
  commandExecutor.registerHandlers();
  editorManager.registerHandlers();
  passwordManager.registerHandlers();
  subscriberManager.registerHandlers();
  terminalManager.registerHandlers();

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
  PasswordManager,
  ROSInfo,
  ShutdownManager,
  SubscriberManager,
  TerminalManager,
};
