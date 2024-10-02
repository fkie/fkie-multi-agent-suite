export const SubscriberManagerEvents = {
  has: "subscriber:has",
  open: "subscriber:open",
  close: "subscriber:close",
  onClose: "subscriber:onClose",
};

export type SubscriberCloseCallback = (tabId: string) => void;

export type TSubscriberManager = {
  open: (
    id: string,
    host: string,
    port: number,
    topic: string,
    showOptions: boolean,
    noData: boolean
  ) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: SubscriberCloseCallback) => void;
};
