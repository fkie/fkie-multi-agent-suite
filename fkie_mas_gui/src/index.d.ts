import { TRosInfo, TSystemInfo } from "@/types";
import { TDialogManager, TEditorManager, TShutdownManager, TSubscriberManager, TTerminalManager, TLaunchManager } from "./types";
import CommandExecutor from "./main/IPC/CommandExecutor";
import PasswordManager from "./main/IPC/PasswordManager";


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
    PasswordManager?: PasswordManager;
    CommandExecutor?: CommandExecutor;
  }
}
