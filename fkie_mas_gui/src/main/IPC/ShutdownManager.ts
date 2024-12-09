import { ShutdownManagerEvents, TerminateCallback, TShutdownManager } from "@/types";
import { app, BrowserWindow, ipcMain } from "electron";
import log from "electron-log";

/**
 * Class ShutdownManager: Handles termination of the app
 */
class ShutdownManager implements TShutdownManager {
  mainWindow: BrowserWindow | null = null;
  closeTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    this.registerHandlers();
  }

  onCloseAppRequest: (callback: TerminateCallback) => void = () => {
    // implemented in preload script
  };

  public registerHandlers: () => void = () => {
    ipcMain.handle(ShutdownManagerEvents.cancelCloseTimeout, this.cancelCloseTimeout);
    ipcMain.handle(ShutdownManagerEvents.quitGui, this.quitGui);
  };

  public emitCloseAppRequest: () => void = () => {
    this.closeTimeout = setTimeout(() => { log.info("Timeout reply quit dialog"); this.quitGui() }, 6000);
    this.mainWindow?.webContents.send(ShutdownManagerEvents.onCloseAppRequest);
  };

  public cancelCloseTimeout = (): void => {
    log.info("close timer canceled");
    if (this.closeTimeout) {
      clearTimeout(this.closeTimeout);
      this.closeTimeout = null;
    }
  };

  /**
   * Destroy main window and quit the electron app
   */
  public quitGui = (): void => {
    log.info("Quitting GUI...");
    this.mainWindow?.destroy();
    app.quit();
  };
}

export default ShutdownManager;
