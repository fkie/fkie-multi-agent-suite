import { TShutdownManager } from "@/types";
import { createContext, useEffect, useMemo, useState } from "react";

export interface IElectronContext {
  shutdownManager: TShutdownManager | null;
  terminateSubprocesses: boolean;
  cancelCloseApp: () => void;
  cancelCloseTimer: () => void;
}

export const DEFAULT = {
  shutdownManager: null,
  terminateSubprocesses: false,
  cancelCloseApp: (): void => {},
  cancelCloseTimer: (): void => {},
};

interface IElectronProviderComponent {
  children: React.ReactNode;
}

export const ElectronContext = createContext<IElectronContext>(DEFAULT);

export function ElectronProvider({
  children,
}: IElectronProviderComponent): ReturnType<React.FC<IElectronProviderComponent>> {
  const [shutdownManager, setShutdownManager] = useState<TShutdownManager | null>(null);
  const [terminateSubprocesses, setTerminateSubprocesses] = useState<boolean>(false);

  function cancelCloseApp(): void {
    cancelCloseTimer();
    setTerminateSubprocesses(false);
  }

  function cancelCloseTimer(): void {
    shutdownManager?.cancelCloseTimeout();
  }

  // Effect to initialize the shutdownManager
  useEffect(() => {
    if (window.shutdownManager) {
      setShutdownManager(window.shutdownManager);
    }
  }, []);

  // Effect to initialize the onCloseAppRequest callback
  useEffect(() => {
    if (shutdownManager?.onCloseAppRequest) {
      shutdownManager?.onCloseAppRequest(() => {
        setTerminateSubprocesses(true);
      });
    }
  }, [shutdownManager]);

  const attributesMemo = useMemo(
    () => ({
      shutdownManager,
      terminateSubprocesses,
      cancelCloseApp,
      cancelCloseTimer,
    }),
    [shutdownManager, terminateSubprocesses]
  );

  return <ElectronContext.Provider value={attributesMemo}>{children}</ElectronContext.Provider>;
}
