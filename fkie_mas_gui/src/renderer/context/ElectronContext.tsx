import { createContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import ShutdownInterface from "../../main/IPC/ShutdownInterface";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "../utils/events";

declare global {
  interface Window {
    ShutdownInterface?: ShutdownInterface;
    electronAPI: any;
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
  checkedForUpdates: boolean;
  setCheckedForUpdates?: (state: boolean) => void;
}

export const DEFAULT = {
  shutdownInterface: null,
  terminateSubprocesses: false,
  setTerminateSubprocesses: () => {},
  requestedInstallUpdate: false,
  updateAvailable: "",
  checkedForUpdates: false,
};

interface IElectronProviderComponent {
  children: React.ReactNode;
}

export const ElectronContext = createContext<IElectronContext>(DEFAULT);

export function ElectronProvider({
  children,
}: IElectronProviderComponent): ReturnType<React.FC<IElectronProviderComponent>> {
  const [shutdownInterface, setShutdownInterface] = useState<ShutdownInterface | null>(null);
  const [terminateSubprocesses, setTerminateSubprocesses] = useState<boolean>(false);
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState<boolean>(false);
  const [updateAvailable, setUpdateAvailable] = useState<string>("");
  const [checkedForUpdates, setCheckedForUpdates] = useState<boolean>(false);
  // Effect to initialize the shutdownInterface
  useEffect(() => {
    if (window.ShutdownInterface) {
      setShutdownInterface(window.ShutdownInterface);
    }
    window.electronAPI?.onEditorFileRange((tabId: string, filePath: string, fileRange) => {
      emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, filePath, fileRange));
    });
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
      checkedForUpdates,
      setCheckedForUpdates,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [requestedInstallUpdate, shutdownInterface, terminateSubprocesses, updateAvailable, checkedForUpdates]
  );

  return <ElectronContext.Provider value={attributesMemo}>{children}</ElectronContext.Provider>;
}
