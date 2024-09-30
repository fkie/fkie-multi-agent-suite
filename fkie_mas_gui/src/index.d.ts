import { TDialogManager, TEditorManager, TShutdownManager, TSubscriberManager, TTerminalManager } from "./types";

declare global {
  interface Window {
    dialogManager: TDialogManager;
    editorManager: TEditorManager;
    shutdownManager: TShutdownManager;
    subscriberManager: TSubscriberManager;
    terminalManager: TTerminalManager;
    autoUpdate: unknown;
  }
}
