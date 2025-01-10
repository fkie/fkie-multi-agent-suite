import { JSONValue } from "@/types";
import { BrowserWindow, screen } from "electron";
import settings from "electron-settings";

export type TWindowState = {
  x: number | undefined;
  y: number | undefined;
  width: number;
  height: number;
  isMaximized?: boolean;
};

export interface TWindowStateRun extends TWindowState {
  track: (win: BrowserWindow) => Promise<void>;
}

export default async function windowStateKeeper(windowName: string): Promise<TWindowStateRun> {
  let window: BrowserWindow | null = null;
  const size = screen.getPrimaryDisplay().workAreaSize;
  let windowState: TWindowState = {
    x: undefined,
    y: undefined,
    width: size.width / 2,
    height: size.height / 2,
  };

  async function setBounds(): Promise<void> {
    // Restore from appConfig
    if (await settings.has(`windowState.${windowName}`)) {
      windowState = (await settings.get(`windowState.${windowName}`)) as TWindowState;
      return;
    }
  }

  async function saveState(): Promise<void> {
    if (window === null) return;
    // TODO lots of save state events are called. they should be debounced
    const bounds = window.getBounds() as TWindowState;
    if (bounds.x !== 0) {
      windowState = bounds;
    }
    windowState.isMaximized = window.isMaximized();
    await settings.set(`windowState.${windowName}`, windowState as JSONValue);
  }

  async function track(win: BrowserWindow): Promise<void> {
    window = win;
    win.on("resize", saveState);
    // win.on("move", saveState);
    win.on("close", saveState);
    win.on("maximize", saveState);
    win.on("unmaximize", saveState);
  }

  await setBounds();

  return {
    x: windowState.x,
    y: windowState.y,
    width: windowState.width,
    height: windowState.height,
    isMaximized: windowState.isMaximized,
    track,
  };
}
