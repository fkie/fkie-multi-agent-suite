import { ShutdownManagerEvents, TerminateCallback, TShutdownManager } from "@/types";
import { app, BrowserWindow, ipcMain } from "electron";
import log from "electron-log";
import { ARGUMENTS, hasArgument } from "../CommandLineInterface";

/**
 * Class ShutdownManager: Handles termination of the app
 */
export default class ShutdownManager implements TShutdownManager {
  mainWindow: BrowserWindow | null = null;
  closeTimeout: ReturnType<typeof setTimeout> | null = null;
  headless = hasArgument(ARGUMENTS.HEADLESS);

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    this.registerHandlers();
  }

  onCloseAppRequest: (callback: TerminateCallback) => void = () => {
    // implemented in preload script
    console.log("onCloseAppRequest");
  };

  public registerHandlers(): void {
    ipcMain.handle(ShutdownManagerEvents.cancelCloseTimeout, this.cancelCloseTimeout);
    ipcMain.handle(ShutdownManagerEvents.quitGui, this.quitGui);
  }

  public emitCloseAppRequest = (): void => {
    if (this.headless) this.quitGui();
    else {
      this.closeTimeout = setTimeout(() => {
        this.quitGui();
      }, 6000);
      this.mainWindow?.webContents.send(ShutdownManagerEvents.onCloseAppRequest);
    }
  };

  public cancelCloseTimeout = (): void => {
    if (this.closeTimeout) {
      clearTimeout(this.closeTimeout);
      this.closeTimeout = null;
    }
  };

  /**
   * Destroy main window and quit the electron app
   */
  public quitGui = (): void => {
    if (!this.headless) log.info("Quitting GUI...");
    this.cancelCloseTimeout();
    this.mainWindow?.destroy();
    app.quit();
    log.info("bye!");
  };
}
