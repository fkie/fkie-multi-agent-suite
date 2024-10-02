import { TRosInfo, TSystemInfo } from "@/types";
import {
  TDialogManager,
  TEditorManager,
  TShutdownManager,
  TSubscriberManager,
  TTerminalManager,
  TCommandExecutor,
  TPasswordManager,
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
    passwordManager?: TPasswordManager;
    commandExecutor?: TCommandExecutor;
  }
}
