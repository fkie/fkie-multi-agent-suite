export const DialogManagerEvents = {
  openFile: "dialog:openFile",
};

export type TDialogManager = {
  openFile: (path: string) => Promise<string | null>;
  registerHandlers: () => void;
  quit: () => void;
};
