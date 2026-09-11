import { PopoutParams } from "@/types";
import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "node:path";

import AutoUpdateManager from "./AutoUpdateManager";
import DialogManager from "./DialogManager";
import EditorManager from "./EditorManager";
import PublishManager from "./PublishManager";
import { ROSInfo } from "./ROSInfo";
import ServiceManager from "./ServiceManager";
import ShutdownManager from "./ShutdownManager";
import SubscriberManager from "./SubscriberManager";
import { SystemInfo } from "./SystemInfo";
import TerminalManager from "./TerminalManager";
import { registerTtydTokenHandler } from "./ttydTokenHandler";

const editorManager = new EditorManager();
const publishManager = new PublishManager();
const serviceManager = new ServiceManager();
const subscriberManager = new SubscriberManager();
const terminalManager = new TerminalManager();

export const registerHandlers = (): void => {
  registerTtydTokenHandler();
  editorManager.registerHandlers();
  publishManager.registerHandlers();
  serviceManager.registerHandlers();
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

export const openUrl = (window: BrowserWindow, site: string, params: PopoutParams): void => {
  const search = new URLSearchParams();
  for (const [k, v] of Object.entries(params)) {
    if (v === undefined) continue;

    if (Array.isArray(v) || (typeof v === "object" && v !== null)) {
      search.set(k, JSON.stringify(v));
    } else {
      search.set(k, String(v));
    }
  }

  if (is.dev && process.env.ELECTRON_RENDERER_URL) {
    const url = `${process.env.ELECTRON_RENDERER_URL}/${site}.html?${search.toString()}`;
    window.loadURL(url);
  } else {
    window.loadFile(join(__dirname, `../renderer/${site}.html`), {
      query: Object.fromEntries(search.entries()),
    });
  }
};

export {
  AutoUpdateManager,
  DialogManager,
  EditorManager,
  PublishManager,
  ROSInfo,
  ServiceManager,
  ShutdownManager,
  SubscriberManager,
  TerminalManager
};

