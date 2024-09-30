import { BrowserWindow } from "electron";

export const SubscriberManagerEvents = {
  has: "subscriber:has",
  open: "subscriber:open",
  close: "subscriber:close",
  onClose: "subscriber:onClose",
};

export interface ISubscriber {
  window: BrowserWindow;
}

export interface ISubscriberManager {
  instances: { [id: string]: ISubscriber };

  registerHandlers: () => void;
  handleHasSubscriber: (event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean>;
  handleSubscriberClose: (event: Electron.IpcMainInvokeEvent, id: string) => Promise<boolean>;
  handleSubscriberOpen: (
    event: Electron.IpcMainInvokeEvent,
    id: string,
    host: string,
    port: number,
    topic: string,
    showOptions: boolean,
    noData: boolean
  ) => Promise<string | null>;
}

export type SubscriberCloseCallback = (tabId: string) => void;

export type TSubscriberManager = {
  open: (
    id: string,
    host: string,
    port: number,
    topic: string,
    showOptions: boolean,
    noData: boolean
  ) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: SubscriberCloseCallback) => void;
};
