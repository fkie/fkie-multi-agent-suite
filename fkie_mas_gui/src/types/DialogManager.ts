export const DialogManagerEvents = {
  openFile: "dialog:openFile",
};

export interface IDialogManager {
  registerHandlers: () => void;
  handleFileOpen: (event: Electron.IpcMainInvokeEvent, path: string) => Promise<string | null>;
  quit: () => void;
}

export type TDialogManager = {
  openFile: (path: string) => Promise<string | null>;
};
