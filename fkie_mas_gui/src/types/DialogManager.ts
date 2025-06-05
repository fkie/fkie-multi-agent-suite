export const DialogManagerEvents = {
  openFile: "dialog:openFile",
  openDirectory: "dialog:openDirectory",
};

export type TDialogManager = {
  openFile: (path: string) => Promise<string | null>;
  openDirectory: (path: string) => Promise<string | null>;
  registerHandlers: () => void;
  quit: () => void;
};
