export const ShutdownManagerEvents = {
  terminateSubprocesses: "ShutdownManager:terminateSubprocesses",
  quitGui: "ShutdownManager:quitGui",
};

export type TerminateCallback = () => void;

export type TShutdownManager = {
  onTerminateSubprocesses: (callback: TerminateCallback) => void;
  sendTerminateSubprocesses: () => void;
  registerHandlers: () => void;
  quitGui: () => void;
};
