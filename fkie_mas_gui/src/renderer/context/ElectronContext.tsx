import React, { createContext, useEffect, useMemo, useState } from 'react';
import ShutdownInterface from '../../main/IPC/ShutdownInterface';

declare global {
  interface Window {
    ShutdownInterface?: ShutdownInterface;
  }
}

export interface IElectronContext {
  shutdownInterface: ShutdownInterface | null;
  terminateSubprocesses: boolean;
  setTerminateSubprocesses: (terminate: boolean) => void;
  requestedInstallUpdate: boolean;
  setRequestedInstallUpdate?: (state: boolean) => void;
  updateAvailable: string;
  setUpdateAvailable?: (version: string) => void;
}

export const DEFAULT = {
  shutdownInterface: null,
  terminateSubprocesses: false,
  setTerminateSubprocesses: () => {},
  requestedInstallUpdate: false,
  updateAvailable: '',
};

interface IElectronProviderComponent {
  children: React.ReactNode;
}

export const ElectronContext = createContext<IElectronContext>(DEFAULT);

export function ElectronProvider({
  children,
}: IElectronProviderComponent): ReturnType<
  React.FC<IElectronProviderComponent>
> {
  const [shutdownInterface, setShutdownInterface] =
    useState<ShutdownInterface | null>(null);
  const [terminateSubprocesses, setTerminateSubprocesses] =
    useState<boolean>(false);
  const [requestedInstallUpdate, setRequestedInstallUpdate] =
    useState<boolean>(false);
  const [updateAvailable, setUpdateAvailable] = useState<string>('');

  // Effect to initialize the shutdownInterface
  useEffect(() => {
    if (window.ShutdownInterface) {
      setShutdownInterface(window.ShutdownInterface);
    }
  }, []);

  // Effect to initialize the onTerminateSubprocesses callback
  useEffect(() => {
    if (shutdownInterface?.onTerminateSubprocesses) {
      shutdownInterface.onTerminateSubprocesses(() => {
        setTerminateSubprocesses(true);
      });
    }
  }, [shutdownInterface]);

  const attributesMemo = useMemo(
    () => ({
      shutdownInterface,
      terminateSubprocesses,
      setTerminateSubprocesses,
      requestedInstallUpdate,
      setRequestedInstallUpdate,
      updateAvailable,
      setUpdateAvailable,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      requestedInstallUpdate,
      shutdownInterface,
      terminateSubprocesses,
      updateAvailable,
    ],
  );

  return (
    <ElectronContext.Provider value={attributesMemo}>
      {children}
    </ElectronContext.Provider>
  );
}
