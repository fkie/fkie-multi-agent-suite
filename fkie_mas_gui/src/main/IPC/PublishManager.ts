import { PublishCloseCallback, PublishManagerEvents, TPublishManager } from "@/types";
import { is } from "@electron-toolkit/utils";
import pubIcon from "@public/sekkyumu_topic_pub.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "node:path";
import windowStateKeeper from "../windowStateKeeper";

type TPublisher = {
  window: BrowserWindow;
};

/**
 * Class SubscriberManager: handle communication with external echo window
 */
export default class PublishManager implements TPublishManager {
  instances: { [id: string]: TPublisher } = {};

  onClose: (callback: PublishCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers(): void {
    ipcMain.handle(PublishManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(PublishManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(
      PublishManagerEvents.start,
      (
        _event: Electron.IpcMainInvokeEvent,
        id: string,
        host: string,
        port: number,
        topicName: string,
        topicType: string
      ) => {
        return this.start(id, host, port, topicName, topicType);
      }
    );
  }

  public has: (id: string) => Promise<boolean> = async (id) => {
    if (this.instances[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public close: (id: string) => Promise<boolean> = async (id) => {
    if (this.instances[id]) {
      this.instances[id].window.destroy();
      delete this.instances[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public start: (
    id: string,
    host: string,
    port: number,
    topicName: string,
    topicType: string
  ) => Promise<string | null> = async (id, host, port, topicName, topicType) => {
    if (this.instances[id]) {
      this.instances[id].window.restore();
      this.instances[id].window.focus();
      return Promise.resolve(null);
    }

    const pubWindowStateKeeper = await windowStateKeeper("publisher");

    const window = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: pubWindowStateKeeper.x,
      y: pubWindowStateKeeper.y,
      width: pubWindowStateKeeper.width,
      height: pubWindowStateKeeper.height,
      icon: pubIcon,
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[id] = { window: window };
    // Track window state
    pubWindowStateKeeper.track(window);

    window.on("ready-to-show", () => {
      if (!window) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        window.minimize();
      } else {
        if (pubWindowStateKeeper.isMaximized) window.maximize();
        window.show();
      }
    });

    window.on("close", async (e) => {
      // send close request to the renderer
      if (this.instances[id]) {
        e.preventDefault();
        this.instances[id].window.webContents.send(PublishManagerEvents.onClose, id);
      }
    });

    window.on("closed", () => {
      delete this.instances[id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    if (is.dev && process.env.ELECTRON_RENDERER_URL) {
      window.loadURL(
        `${process.env.ELECTRON_RENDERER_URL}/publisher.html?id=${id}&host=${host}&port=${port}&topicName=${topicName}&topicType=${topicType}`
      );
    } else {
      window.loadFile(join(__dirname, "../renderer/publisher.html"), {
        query: {
          id: id,
          host: host,
          port: `${port}`,
          topicName: topicName,
          topicType: `${topicType}`,
        },
      });
    }
    return Promise.resolve(null);
  };
}
