import { ShutdownManagerEvents, TerminateCallback, TShutdownManager } from "@/types";
import { app, BrowserWindow, ipcMain } from "electron";
import log from "electron-log";
import CommandLine from "./CommandLine";

/**
 * Class ShutdownManager: Handles termination of the app
 */
export default class ShutdownManager implements TShutdownManager {
  mainWindow: BrowserWindow | null = null;
  commandLine: CommandLine | null = null;
  closeTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(mainWindow: BrowserWindow, commandLine: CommandLine) {
    this.mainWindow = mainWindow;
    this.commandLine = commandLine;
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
    if (this.commandLine?.getArg("headless")) this.quitGui();
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
    if (!this.commandLine?.getArg("headless")) log.info("Quitting GUI...");
    this.cancelCloseTimeout();
    this.mainWindow?.destroy();
    app.quit();
    log.info("bye!");
  };
}
