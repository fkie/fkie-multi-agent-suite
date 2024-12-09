export const ShutdownManagerEvents = {
  emitCloseAppRequest: "ShutdownManager:emitCloseAppRequest",
  onCloseAppRequest: "ShutdownManager:onCloseAppRequest",
  cancelCloseTimeout: "ShutdownManager:cancelCloseTimeout",
  quitGui: "ShutdownManager:quitGui",
};

export type TerminateCallback = () => void;

export type TShutdownManager = {
  onCloseAppRequest: (callback: TerminateCallback) => void;
  emitCloseAppRequest: () => void;
  cancelCloseTimeout: () => void;
  quitGui: () => void;
};
