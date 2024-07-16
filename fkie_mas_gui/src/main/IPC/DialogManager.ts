import { BrowserWindow, dialog, ipcMain } from 'electron'
import log from 'electron-log'

/**
 * Class DialogManager: Open files requests
 */
class DialogManager {
  mainWindow: BrowserWindow | null = null

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow
    log.transports.file.level = 'info'

    ipcMain.on('select-file', () => {
      if (this.mainWindow)
        this.mainWindow.webContents.send(
          'file-selected',
          dialog.showOpenDialog({ properties: ['openFile'] })
        )
    })
  }

  quit(): void {
    // TODO
  }
}

export default DialogManager
