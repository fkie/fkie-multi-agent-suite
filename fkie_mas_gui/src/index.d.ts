import { TRosInfo, TSystemInfo } from "@/types";
import {
  TAutoUpdateManager,
  TCommandExecutor,
  TDialogManager,
  TEditorManager,
  TShutdownManager,
  TSubscriberManager,
  TTerminalManager,
} from "./types";

declare global {
  interface Window {
    dialogManager?: TDialogManager;
    editorManager?: TEditorManager;
    rosInfo?: TRosInfo;
    systemInfo?: TSystemInfo;
    shutdownManager?: TShutdownManager;
    subscriberManager?: TSubscriberManager;
    terminalManager?: TTerminalManager;
    autoUpdate?: TAutoUpdateManager;
    commandExecutor?: TCommandExecutor;
  }
}
