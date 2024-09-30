import { BrowserWindow } from "electron";

export const TerminalManagerEvents = {
  has: "terminal:has",
  open: "terminal:open",
  close: "terminal:close",
  onClose: "terminal:onClose",
};

export interface ITerminal {
  window: BrowserWindow;
}

export interface ITerminalManager {
  instances: { [id: string]: ITerminal };

  registerHandlers: () => void;
  handleHasTerminal: (event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean>;
  handleCloseTerminal: (event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean>;
  handleOpenTerminal: (
    event: Electron.IpcMainInvokeEvent,
    id: string,
    host: string,
    port: number,
    info: string,
    node: string,
    screen: string,
    cmd: string
  ) => Promise<string | null>;
}

export type TerminalCloseCallback = (tabId: string) => void;

export type TTerminalManager = {
  open: (
    id: string,
    host: string,
    port: number,
    info: string,
    node: string,
    screen: string,
    cmd: string
  ) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: TerminalCloseCallback) => void;
};
