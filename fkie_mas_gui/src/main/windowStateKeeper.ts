import { BrowserWindow, screen } from 'electron'
import settings from 'electron-settings'

const windowStateKeeper = async (windowName: string) => {
  let window: BrowserWindow | null = null
  let windowState: any

  const setBounds = async (): Promise<void> => {
    // Restore from appConfig
    if (await settings.has(`windowState.${windowName}`)) {
      windowState = await settings.get(`windowState.${windowName}`)
      return
    }

    const size = screen.getPrimaryDisplay().workAreaSize

    // Default
    windowState = {
      x: undefined,
      y: undefined,
      width: size.width / 2,
      height: size.height / 2
    }
  }

  const saveState = async (): Promise<void> => {
    if (window === null) return
    // TODO lots of save state events are called. they should be debounced
    const bounds = window.getBounds()
    if (bounds.x !== 0) {
      windowState = bounds
    }
    windowState.isMaximized = window.isMaximized()
    await settings.set(`windowState.${windowName}`, windowState)
  }

  const track = async (win: BrowserWindow): Promise<void> => {
    window = win
    ;['resize', 'move', 'close', 'maximize', 'unmaximize'].forEach((eventName) => {
      const eventType: any = eventName
      win.on(eventType, saveState)
    })
  }

  await setBounds()

  return {
    x: windowState.x,
    y: windowState.y,
    width: windowState.width,
    height: windowState.height,
    isMaximized: windowState.isMaximized,
    track
  }
}

export default windowStateKeeper
