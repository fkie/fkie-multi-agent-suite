import {
  TAutoUpdateManager,
  TCommandExecutor,
  TDialogManager,
  TEditorManager,
  TPublishManager,
  TRosInfo,
  TShutdownManager,
  TSubscriberManager,
  TSystemInfo,
  TTerminalManager,
} from "@/types";

declare global {
  interface Window {
    dialogManager?: TDialogManager;
    editorManager?: TEditorManager;
    publishManager?: TPublishManager;
    rosInfo?: TRosInfo;
    systemInfo?: TSystemInfo;
    shutdownManager?: TShutdownManager;
    subscriberManager?: TSubscriberManager;
    terminalManager?: TTerminalManager;
    autoUpdate?: TAutoUpdateManager;
    commandExecutor?: TCommandExecutor;
  }
}
