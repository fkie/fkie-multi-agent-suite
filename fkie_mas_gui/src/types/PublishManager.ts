export const PublishManagerEvents = {
  has: "publisher:has",
  start: "publisher:start",
  close: "publisher:close",
  onClose: "publisher:onClose",
};

export type PublishCloseCallback = (tabId: string) => void;

export type TPublishManager = {
  start: (id: string, host: string, port: number, topicName: string, topicType: string) => Promise<string | null>;
  close: (id: string) => Promise<boolean>;
  has: (id: string) => Promise<boolean>;
  onClose: (callback: PublishCloseCallback) => void;
};
