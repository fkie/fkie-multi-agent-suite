export const ShutdownManagerEvents = {
  emitTerminateSubprocesses: "ShutdownManager:emitTerminateSubprocesses",
  onTerminateSubprocesses: "ShutdownManager:onTerminateSubprocesses",
  quitGui: "ShutdownManager:quitGui",
};

export type TerminateCallback = () => void;

export type TShutdownManager = {
  onTerminateSubprocesses: (callback: TerminateCallback) => void;
  emitTerminateSubprocesses: () => void;
  quitGui: () => void;
};
