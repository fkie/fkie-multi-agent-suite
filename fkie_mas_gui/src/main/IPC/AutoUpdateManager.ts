import { BrowserWindow, ipcMain } from 'electron';
import log from 'electron-log';
import { autoUpdater } from 'electron-updater';

/**
 * Class AutoUpdateManager: Handles autoUpdate of the app
 */
class AutoUpdateManager {
  isChecking: boolean = false;
  mainWindow: BrowserWindow | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    log.transports.file.level = 'info';
    autoUpdater.logger = log;
    autoUpdater.setFeedURL({
      provider: 'github',
      repo: 'fkie-multi-agent-suite',
      owner: 'fkie',
      private: true,
      token: process.env.GH_TOKEN,
    });

    autoUpdater.on('checking-for-update', () => {
      this.isChecking = true;
      if (this.mainWindow)
        this.mainWindow.webContents.send(
          'checking-for-update',
          this.isChecking,
        );
    });
    autoUpdater.on('update-available', (info) => {
      this.isChecking = false;
      if (this.mainWindow)
        this.mainWindow.webContents.send('update-available', info);
    });
    autoUpdater.on('update-not-available', (info) => {
      this.isChecking = false;
      if (this.mainWindow)
        this.mainWindow.webContents.send('update-not-available', info);
    });
    autoUpdater.on('error', (err) => {
      if (this.mainWindow)
        this.mainWindow.webContents.send('update-error', err.message);
    });
    autoUpdater.on('download-progress', (progressObj) => {
      if (this.mainWindow)
        this.mainWindow.webContents.send('download-progress', progressObj);
    });
    autoUpdater.on('update-downloaded', () => {
      if (this.mainWindow)
        this.mainWindow.webContents.send('update-downloaded', true);
    });

    ipcMain.on('check-for-updates', () => {
      if (this.isChecking) return;
      if (process.env.APPIMAGE === undefined) {
        if (this.mainWindow)
          this.mainWindow.webContents.send(
            'update-error',
            'APPIMAGE env is not defined, current application is not an AppImage',
          );
        return;
      }
      autoUpdater.checkForUpdates().catch(() => {
        this.isChecking = false;
      });
    });
    ipcMain.on('quit-and-install', () => {
      autoUpdater.quitAndInstall();
    });
  }
}

export default AutoUpdateManager;
