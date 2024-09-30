// import { ElectronAPI } from "@electron-toolkit/preload";
import { TEditorManager, TShutdownManager, TSubscriberManager } from "./types";

declare global {
  interface Window {
    editorManager: TEditorManager;
    shutdownManager: TShutdownManager;
    subscriberManager: TSubscriberManager;
    autoUpdate: unknown;
    electronAPI: unknown;
  }
}
