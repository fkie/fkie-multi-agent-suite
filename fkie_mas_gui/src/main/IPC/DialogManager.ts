import { DialogManagerEvents, TDialogManager } from "@/types";
import { BrowserWindow, dialog, ipcMain } from "electron";
import log from "electron-log";

/**
 * Class DialogManager: Open files requests
 */
export default class DialogManager implements TDialogManager {
  mainWindow: BrowserWindow | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    log.transports.file.level = "info";
    this.registerHandlers();
  }

  public registerHandlers(): void {
    ipcMain.handle(DialogManagerEvents.openFile, (_event: Electron.IpcMainInvokeEvent, path: string) => {
      return this.openFile(path);
    });
    ipcMain.handle(DialogManagerEvents.openDirectory, (_event: Electron.IpcMainInvokeEvent, path: string) => {
      return this.openDirectory(path);
    });
  }

  public openFile: (path: string) => Promise<string | null> = async (path) => {
    const { canceled, filePaths } = await dialog.showOpenDialog({
      defaultPath: path,
      properties: ["openFile"],
    });
    if (!canceled) {
      return filePaths[0];
    }
    return null;
  };

  public openDirectory: (path: string) => Promise<string | null> = async (path) => {
    const { canceled, filePaths } = await dialog.showOpenDialog({
      defaultPath: path,
      properties: ["openDirectory"],
    });
    if (!canceled) {
      return filePaths[0];
    }
    return null;
  };

  quit(): void {
    // TODO
  }
}
