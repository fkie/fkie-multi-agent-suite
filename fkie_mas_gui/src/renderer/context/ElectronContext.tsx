import { TShutdownManager } from "@/types";
import { createContext, useEffect, useMemo, useState } from "react";

export interface IElectronContext {
  shutdownManager: TShutdownManager | null;
  terminateSubprocesses: boolean;
  setTerminateSubprocesses: (terminate: boolean) => void;
}

export const DEFAULT = {
  shutdownManager: null,
  terminateSubprocesses: false,
  setTerminateSubprocesses: () => {},
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
  // Effect to initialize the shutdownManager
  useEffect(() => {
    if (window.shutdownManager) {
      setShutdownManager(window.shutdownManager);
    }
  }, []);

  // Effect to initialize the onTerminateSubprocesses callback
  useEffect(() => {
    if (shutdownManager?.onTerminateSubprocesses) {
      shutdownManager?.onTerminateSubprocesses(() => {
        setTerminateSubprocesses(true);
      });
    }
  }, [shutdownManager]);

  const attributesMemo = useMemo(
    () => ({
      shutdownManager,
      terminateSubprocesses,
      setTerminateSubprocesses,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [shutdownManager, terminateSubprocesses]
  );

  return <ElectronContext.Provider value={attributesMemo}>{children}</ElectronContext.Provider>;
}
