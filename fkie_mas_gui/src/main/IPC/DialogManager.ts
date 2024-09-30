import { BrowserWindow, dialog, ipcMain } from "electron";
import log from "electron-log";
import { IDialogManager, DialogManagerEvents } from "@/types";

/**
 * Class DialogManager: Open files requests
 */
class DialogManager implements IDialogManager {
  mainWindow: BrowserWindow | null = null;

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow;
    log.transports.file.level = "info";
    this.registerHandlers();
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(DialogManagerEvents.openFile, this.handleFileOpen);
  };

  public handleFileOpen: (event: Electron.IpcMainInvokeEvent, path: string) => Promise<string | null> = async (
    _event,
    path
  ) => {
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
