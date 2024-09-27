// import { ElectronAPI } from "@electron-toolkit/preload";
import { TShutdownManager } from "./types";

declare global {
  interface Window {
    shutdownManager: TShutdownManager;
    autoUpdate: unknown;
    electronAPI: unknown;
  }
}
