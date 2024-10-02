export const TerminalManagerEvents = {
  has: "terminal:has",
  open: "terminal:open",
  close: "terminal:close",
  onClose: "terminal:onClose",
};

export type TerminalCloseCallback = (tabId: string) => void;

export type TTerminalManager = {
  open: (
    id: string,
    host: string,
    port: number,
    info: string,
    node: string,
    screen: string,
    cmd: string
  ) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: TerminalCloseCallback) => void;
};
