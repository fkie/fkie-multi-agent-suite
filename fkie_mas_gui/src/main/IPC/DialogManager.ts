import { DialogManagerEvents, TDialogManager } from "@/types";
import { BrowserWindow, dialog, ipcMain } from "electron";
import log from "electron-log";

/**
 * Class DialogManager: Open files requests
 */
class DialogManager implements TDialogManager {
  mainWindow: BrowserWindow | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    log.transports.file.level = "info";
    this.registerHandlers();
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(DialogManagerEvents.openFile, (_event: Electron.IpcMainInvokeEvent, path: string) => {
      return this.openFile(path);
    });
  };

  public openFile: (path: string) => Promise<string | null> = async (path) => {
    const { canceled, filePaths } = await dialog.showOpenDialog({
      defaultPath: path,
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

export default DialogManager;
