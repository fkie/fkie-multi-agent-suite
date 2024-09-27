const ShutdownManagerEvents = {
  terminateSubprocesses: "ShutdownManager:terminateSubprocesses",
  quitGui: "ShutdownManager:quitGui",
};

interface IShutdownManager {
  registerHandlers: () => void;
  sendTerminateSubprocesses: () => void;
  quitGui: () => void;
}

type TShutdownManager = {
  onTerminateSubprocesses: (callback: Function) => void;
  quitGui: () => void;
};

export { ShutdownManagerEvents };
export type { IShutdownManager, TShutdownManager };

