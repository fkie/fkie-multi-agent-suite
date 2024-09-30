// import { ElectronAPI } from "@electron-toolkit/preload";
import { TEditorManager, TShutdownManager } from "./types";

declare global {
  interface Window {
    editorManager: TEditorManager;
    shutdownManager: TShutdownManager;
    autoUpdate: unknown;
    electronAPI: unknown;
  }
}
