import { app, BrowserWindow } from 'electron'
import log from 'electron-log'

/**
 * Class ShutdownInterface: Handles termination of the app
 */
class ShutdownInterface {
  mainWindow: BrowserWindow | null = null

  constructor(mainWindow: BrowserWindow) {
    this.mainWindow = mainWindow
  }

  public onTerminateSubprocesses: ((callback: Function) => void) | undefined

  /**
   * Destroy main window and quit the electron app
   */
  public quitGui = (): void => {
    log.info('Quitting GUI...')
    this.mainWindow?.destroy()
    app.quit()
  }
}

export default ShutdownInterface
