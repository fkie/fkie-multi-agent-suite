import { is } from "@electron-toolkit/utils";
import { BrowserWindow, ipcMain } from "electron";
import { join } from "path";
import { SubscriberManagerEvents, ISubscriber, ISubscriberManager } from "@/types";
import windowStateKeeper from "../windowStateKeeper";

/**
 * Class SubscriberManager: handle communication with external echo window
 */
class SubscriberManager implements ISubscriberManager {
  instances: { [id: string]: ISubscriber } = {};

  constructor() {}

  public registerHandlers: () => void = () => {
    ipcMain.handle(SubscriberManagerEvents.has, this.handleHasSubscriber);
    ipcMain.handle(SubscriberManagerEvents.open, this.handleSubscriberOpen);
    ipcMain.handle(SubscriberManagerEvents.close, this.handleSubscriberClose);
  };

  public handleHasSubscriber: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.instances[id]) {
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleSubscriberClose: (_event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean> = async (
    _event,
    id
  ) => {
    if (this.instances[id]) {
      this.instances[id].window.destroy();
      delete this.instances[id];
      return Promise.resolve(true);
    }
    return Promise.resolve(false);
  };

  public handleSubscriberOpen: (
    _event: Electron.IpcMainInvokeEvent,
    id: string,
    host: string,
    port: number,
    topic: string,
    showOptions: boolean,
    noData: boolean
  ) => Promise<string | null> = async (_event, id, host, port, topic, showOptions, noData) => {
    if (this.instances[id]) {
      this.instances[id].window.focus();
      return Promise.resolve(null);
    }

    const editorWindowStateKeeper = await windowStateKeeper("editor");

    const window = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: editorWindowStateKeeper.x,
      y: editorWindowStateKeeper.y,
      width: editorWindowStateKeeper.width,
      height: editorWindowStateKeeper.height,
      icon: join(__dirname, "../../icon/sekkyumu_topic_echo.png"),
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[id] = { window: window };
    // Track window state
    editorWindowStateKeeper.track(window);

    window.on("ready-to-show", () => {
      if (!window) {
        throw new Error('"mainWindow" is not defined');
      }
      if (process.env.START_MINIMIZED) {
        window.minimize();
      } else {
        if (editorWindowStateKeeper.isMaximized) window.maximize();
        window.show();
      }
    });

    window.on("close", async (e) => {
      // send close request to the renderer
      if (this.instances[id]) {
        e.preventDefault();
        this.instances[id].window.webContents.send(SubscriberManagerEvents.onClose, id);
      }
    });

    window.on("closed", () => {
      delete this.instances[id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    if (is.dev && process.env.ELECTRON_RENDERER_URL) {
      window.loadURL(
        `${process.env.ELECTRON_RENDERER_URL}/subscriber.html?id=${id}&host=${host}&port=${port}&topic=${topic}&showOptions=${showOptions}&noData=${noData}`
      );
    } else {
      window.loadFile(join(__dirname, `../renderer/subscriber.html`), {
        query: {
          id: id,
          host: host,
          port: `${port}`,
          topic: topic,
          showOptions: `${showOptions}`,
          noData: `${noData}`,
        },
      });
    }
    return Promise.resolve(null);
  };
}

export default SubscriberManager;
