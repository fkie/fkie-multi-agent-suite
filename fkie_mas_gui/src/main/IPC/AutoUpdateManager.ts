import {
  AuCheckingForUpdateCallback,
  AuDownloadProgressCallback,
  AutoUpdateManagerEvents,
  AuUpdateAvailableCallback,
  AuUpdateDownloadedCallback,
  AuUpdateErrorCallback,
  TAutoUpdateManager,
} from "@/types";
import { BrowserWindow, ipcMain } from "electron";
import log from "electron-log";
import { autoUpdater } from "electron-updater";

/**
 * Class AutoUpdateManager: Handles autoUpdate of the app
 */
export default class AutoUpdateManager implements TAutoUpdateManager {
  isChecking: boolean = false;
  mainWindow: BrowserWindow | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    log.transports.file.level = "info";
    autoUpdater.logger = log;
    autoUpdater.setFeedURL({
      provider: "github",
      repo: "fkie-multi-agent-suite",
      owner: "fkie",
      private: true,
      token: process.env.GH_TOKEN,
    });

    this.registerHandlers();
  }

  onCheckingForUpdate: (callback: AuCheckingForUpdateCallback) => void = () => {
    // implemented in preload script
  };
  onUpdateAvailable: (callback: AuUpdateAvailableCallback) => void = () => {
    // implemented in preload script
  };
  onUpdateNotAvailable: (callback: AuUpdateAvailableCallback) => void = () => {
    // implemented in preload script
  };
  onDownloadProgress: (callback: AuDownloadProgressCallback) => void = () => {
    // implemented in preload script
  };
  onUpdateDownloaded: (callback: AuUpdateDownloadedCallback) => void = () => {
    // implemented in preload script
  };
  onUpdateError: (callback: AuUpdateErrorCallback) => void = () => {
    // implemented in preload script
  };

  checkForUpdate = (): void => {
    if (this.isChecking) return;
    if (process.env.APPIMAGE === undefined) {
      this.mainWindow?.webContents.send(
        AutoUpdateManagerEvents.onUpdateError,
        "APPIMAGE env is not defined, current application is not an AppImage"
      );
      return;
    }
    autoUpdater.checkForUpdates().catch(() => {
      this.isChecking = false;
    });
  };
  quitAndInstall = (): void => {
    autoUpdater.quitAndInstall();
  };

  public registerHandlers(): void {
    ipcMain.handle(AutoUpdateManagerEvents.checkForUpdate, () => {
      this.checkForUpdate();
    });
    ipcMain.handle(AutoUpdateManagerEvents.quitAndInstall, () => {
      this.quitAndInstall();
    });
    autoUpdater.on("checking-for-update", () => {
      this.isChecking = true;
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onCheckingForUpdate, this.isChecking);
    });
    autoUpdater.on("update-available", (info) => {
      this.isChecking = false;
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onCheckingForUpdate, this.isChecking);
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onUpdateAvailable, info);
    });
    autoUpdater.on("update-not-available", (info) => {
      this.isChecking = false;
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onCheckingForUpdate, this.isChecking);
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onUpdateNotAvailable, info);
    });
    autoUpdater.on("error", (err) => {
      this.isChecking = false;
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onCheckingForUpdate, this.isChecking);
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onUpdateError, err.message);
    });
    autoUpdater.on("download-progress", (progressObj) => {
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onDownloadProgress, progressObj);
    });
    autoUpdater.on("update-downloaded", (path) => {
      this.mainWindow?.webContents.send(AutoUpdateManagerEvents.onUpdateDownloaded, path);
    });
  }

  quit = (): void => {
    // TODO
  };
}
