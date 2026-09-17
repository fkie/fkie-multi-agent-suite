import { join } from "node:path";
import actionIcon from "@public/google_start.png?asset";
import serviceCallIcon from "@public/google_sync_alt.png?asset";
import introspectionIcon from "@public/google_troubleshoot.png?asset";
import { BrowserWindow, ipcMain } from "electron";
import { ServiceCloseCallback, ServiceManagerEvents, TServiceManager } from "@/types";
import { TServiceConfig } from "@/types/ServiceManager";
import windowStateKeeper from "../windowStateKeeper";
import { openUrl } from ".";

type TPublisher = {
  window: BrowserWindow;
};

/**
 * Class ServiceManager: handle communication with external window
 */
export default class ServiceManager implements TServiceManager {
  instances: { [id: string]: TPublisher } = {};

  onClose: (callback: ServiceCloseCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers(): void {
    ipcMain.handle(ServiceManagerEvents.has, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.has(id);
    });
    ipcMain.handle(ServiceManagerEvents.close, (_event: Electron.IpcMainInvokeEvent, id: string) => {
      return this.close(id);
    });
    ipcMain.handle(ServiceManagerEvents.start, (_event: Electron.IpcMainInvokeEvent, props: TServiceConfig) => {
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

  public start: (props: TServiceConfig) => Promise<string | null> = async (props) => {
    if (this.instances[props.id]) {
      this.instances[props.id].window.restore();
      this.instances[props.id].window.focus();
      return Promise.resolve(null);
    }
    let serviceIcon = "";
    if (props.htmlName === "serviceCaller") {
      serviceIcon = serviceCallIcon;
    } else if (props.htmlName === "serviceIntrospection" || props.htmlName === "actionIntrospection") {
      serviceIcon = introspectionIcon;
    } else {
      serviceIcon = actionIcon;
    }

    const serviceWindowStateKeeper = await windowStateKeeper(props.htmlName);

    const window = new BrowserWindow({
      autoHideMenuBar: true,
      show: false,
      frame: true,
      x: serviceWindowStateKeeper.x,
      y: serviceWindowStateKeeper.y,
      width: serviceWindowStateKeeper.width,
      height: serviceWindowStateKeeper.height,
      icon: serviceIcon,
      webPreferences: {
        sandbox: false,
        nodeIntegration: true,
        preload: join(__dirname, "../preload/index.js"),
      },
    });
    this.instances[props.id] = { window: window };
    // Track window state
    serviceWindowStateKeeper.track(window);

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
        this.instances[props.id].window.webContents.send(ServiceManagerEvents.onClose, props.id);
      }
    });

    window.on("closed", () => {
      delete this.instances[props.id];
    });

    // HMR for renderer base on electron-vite cli.
    // Load the remote URL for development or the local html file for production.
    openUrl(window, props.htmlName, props);
    return Promise.resolve(null);
  };
}
