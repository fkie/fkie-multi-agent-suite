import { join } from "node:path";
import subIcon from "@public/google_chat_bubble.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import { SubscriberCloseCallback, SubscriberManagerEvents, TSubscriberConfig, TSubscriberManager } from "@/types";
import windowStateKeeper from "../windowStateKeeper";
import { openUrl } from ".";

type TSubscriber = {
  window: BrowserWindow;
};

/**
 * Class SubscriberManager: handle communication with external echo window
 */
export default class SubscriberManager implements TSubscriberManager {
  instances: { [id: string]: TSubscriber } = {};

  onClose: (callback: SubscriberCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers(): void {
    ipcMain.handle(SubscriberManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(SubscriberManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(SubscriberManagerEvents.open, (_event: Electron.IpcMainInvokeEvent, props: TSubscriberConfig) => {
      return this.open(props);
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

  public open: (props: TSubscriberConfig) => Promise<string | null> = async (props) => {
    if (this.instances[props.id]) {
      this.instances[props.id].window.restore();
      this.instances[props.id].window.focus();
      return Promise.resolve(null);
    }

    const editorWindowStateKeeper = await windowStateKeeper("subscriber");

    const window = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: editorWindowStateKeeper.x,
      y: editorWindowStateKeeper.y,
      width: editorWindowStateKeeper.width,
      height: editorWindowStateKeeper.height,
      icon: subIcon,
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[props.id] = { window: window };
    // Track window state
    editorWindowStateKeeper.track(window);

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
        this.instances[props.id].window.webContents.send(SubscriberManagerEvents.onClose, props.id);
      }
    });

    window.on("closed", () => {
      delete this.instances[props.id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    openUrl(window, "subscriber", props);
    return Promise.resolve(null);
  };
}
