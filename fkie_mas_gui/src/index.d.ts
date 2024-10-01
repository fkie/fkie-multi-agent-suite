import { TRosInfo, TSystemInfo } from "@/types";
import {
  TDialogManager,
  TEditorManager,
  TShutdownManager,
  TSubscriberManager,
  TTerminalManager,
  TLaunchManager,
  TCommandExecutor,
  TPasswordManager,
} from "./types";

declare global {
  interface Window {
    dialogManager?: TDialogManager;
    editorManager?: TEditorManager;
    launchManager?: TLaunchManager;
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
