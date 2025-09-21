import { ipcMain } from "electron";
import AutoUpdateManager from "./AutoUpdateManager";
import CommandExecutor from "./CommandExecutor";
import CommandLine from "./CommandLine";
import DialogManager from "./DialogManager";
import EditorManager from "./EditorManager";
import PublishManager from "./PublishManager";
import { ROSInfo } from "./ROSInfo";
import ShutdownManager from "./ShutdownManager";
import SubscriberManager from "./SubscriberManager";
import { SystemInfo } from "./SystemInfo";
import TerminalManager from "./TerminalManager";

const commandExecutor = new CommandExecutor();
const commandLine = new CommandLine();
const editorManager = new EditorManager();
const publishManager = new PublishManager();
const subscriberManager = new SubscriberManager();
const terminalManager = new TerminalManager();

export const registerHandlers = (): void => {
  commandExecutor.registerHandlers();
  commandLine.registerHandlers();
  editorManager.registerHandlers();
  publishManager.registerHandlers();
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
  PublishManager,
  ROSInfo,
  ShutdownManager,
  SubscriberManager,
  TerminalManager
};

