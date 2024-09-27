const ShutdownManagerEvents = {
  terminateSubprocesses: "ShutdownManager:terminateSubprocesses",
  quitGui: "ShutdownManager:quitGui",
};

type TerminateCallback = () => void;

interface IShutdownManager {
  registerHandlers: () => void;
  sendTerminateSubprocesses: () => void;
  quitGui: () => void;
}

type TShutdownManager = {
  onTerminateSubprocesses: (callback: TerminateCallback) => void;
  quitGui: () => void;
};

export { ShutdownManagerEvents };
export type { IShutdownManager, TShutdownManager, TerminateCallback };

