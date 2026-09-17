import { join } from "node:path";
import pubIcon from "@public/google_play_circle.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import { PublishCloseCallback, PublishManagerEvents, TPublisherConfig, TPublishManager } from "@/types";
import windowStateKeeper from "../windowStateKeeper";
import { openUrl } from ".";

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
    ipcMain.handle(PublishManagerEvents.start, (_event: Electron.IpcMainInvokeEvent, props: TPublisherConfig) => {
      return this.start(props);
    });
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

  public start: (props: TPublisherConfig) => Promise<string | null> = async (props) => {
    if (this.instances[props.id]) {
      this.instances[props.id].window.restore();
      this.instances[props.id].window.focus();
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
    this.instances[props.id] = { window: window };
    // Track window state
    pubWindowStateKeeper.track(window);

    window.on("ready-to-show", () => {
      if (!window) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        window.minimize();
      } else {
        window.show();
      }
    });

    window.on("close", async (e) => {
      // send close request to the renderer
      if (this.instances[props.id]) {
        e.preventDefault();
        this.instances[props.id].window.webContents.send(PublishManagerEvents.onClose, props.id);
      }
    });

    window.on("closed", () => {
      delete this.instances[props.id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    openUrl(window, "publisher", props);
    return Promise.resolve(null);
  };
}
