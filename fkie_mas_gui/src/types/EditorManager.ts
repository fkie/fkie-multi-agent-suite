import { TFileRange } from "./FileRange";
import { TLaunchArg } from "./LaunchArg";

export const EditorManagerEvents = {
  has: "editor:has",
  open: "editor:open",
  close: "editor:close",
  changed: "editor:changed",
  emitFileRange: "editor:emitFileRange",
  onFileRange: "editor:onFileRange",
  onClose: "editor:onClose",
};

export type FileRangeCallback = (
  tabId: string,
  filePath: string,
  fileRange: TFileRange | null,
  launchArgs: TLaunchArg[]
) => void;

export type EditorCloseCallback = (tabId: string) => void;

export type TEditorManager = {
  open: (
    id: string,
    host: string,
    port: number,
    path: string,
    rootLaunch: string,
    fileRange: TFileRange | null,
    launchArgs: TLaunchArg[]
  ) => Promise<string | null>;

  close: (id: string) => Promise<boolean>;

  changed: (id: string, path: string, changed: boolean) => Promise<boolean>;

  has: (id: string) => Promise<boolean>;

  emitFileRange: (id: string, path: string, fileRange: TFileRange | null, launchArgs: TLaunchArg[]) => Promise<boolean>;

  onFileRange: (callback: FileRangeCallback) => void;

  onClose: (callback: EditorCloseCallback) => void;
};
