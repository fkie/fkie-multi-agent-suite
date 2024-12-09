import { TShutdownManager } from "@/types";
import { createContext, useEffect, useMemo, useState } from "react";

export interface IElectronContext {
  shutdownManager: TShutdownManager | null;
  terminateSubprocesses: boolean;
  cancelCloseApp: () => void;
}

export const DEFAULT = {
  shutdownManager: null,
  terminateSubprocesses: false,
  cancelCloseApp: () => {},
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

  function cancelCloseApp() {
    shutdownManager?.cancelCloseTimeout();
    setTerminateSubprocesses(false);
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
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [shutdownManager, terminateSubprocesses]
  );

  return <ElectronContext.Provider value={attributesMemo}>{children}</ElectronContext.Provider>;
}
