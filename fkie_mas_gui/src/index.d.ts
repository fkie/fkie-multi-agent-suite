import {
  TAutoUpdateManager,
  TCommandExecutor,
  TCommandLine,
  TDialogManager,
  TEditorManager,
  TPublishManager,
  TRosInfo,
  TServiceManager,
  TShutdownManager,
  TSubscriberManager,
  TSystemInfo,
  TTerminalManager,
} from "@/types";

declare global {
  interface Window {
    APP_VERSION?: string;
    dialogManager?: TDialogManager;
    editorManager?: TEditorManager;
    publishManager?: TPublishManager;
    serviceManager?: TServiceManager;
    rosInfo?: TRosInfo;
    systemInfo?: TSystemInfo;
    shutdownManager?: TShutdownManager;
    subscriberManager?: TSubscriberManager;
    terminalManager?: TTerminalManager;
    autoUpdate?: TAutoUpdateManager;
    commandExecutor?: TCommandExecutor;
    commandLine?: TCommandLine;
    ttydApi?: { fetchToken: (url: string) => string };
  }
}
