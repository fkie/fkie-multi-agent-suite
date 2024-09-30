// import { ElectronAPI } from "@electron-toolkit/preload";
import { TEditorManager, TShutdownManager, TSubscriberManager, TTerminalManager } from "./types";

declare global {
  interface Window {
    editorManager: TEditorManager;
    shutdownManager: TShutdownManager;
    subscriberManager: TSubscriberManager;
    terminalManager: TTerminalManager;
    autoUpdate: unknown;
    electronAPI: unknown;
  }
}
