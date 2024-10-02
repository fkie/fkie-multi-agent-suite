import { TRosInfo, TSystemInfo } from "@/types";
import {
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
    autoUpdate?: unknown;
    commandExecutor?: TCommandExecutor;
  }
}
