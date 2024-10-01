import { FileRange, TShutdownManager } from "@/types";
import { createContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  eventCloseComponent,
  eventEditorSelectRange,
} from "../utils/events";

export interface IElectronContext {
  shutdownManager: TShutdownManager | null;
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
  shutdownManager: null,
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
  const [shutdownManager, setShutdownManager] = useState<TShutdownManager | null>(null);
  const [terminateSubprocesses, setTerminateSubprocesses] = useState<boolean>(false);
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState<boolean>(false);
  const [updateAvailable, setUpdateAvailable] = useState<string>("");
  const [checkedForUpdates, setCheckedForUpdates] = useState<boolean>(false);
  // Effect to initialize the shutdownManager
  useEffect(() => {
    if (window.shutdownManager) {
      setShutdownManager(window.shutdownManager);
    }
    window.editorManager?.onFileRange((tabId: string, filePath: string, fileRange: FileRange | null) => {
      if (fileRange) {
        emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, filePath, fileRange));
      }
    });
    window.editorManager?.onClose((tabId: string) => {
      emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(tabId));
    });
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
      requestedInstallUpdate,
      setRequestedInstallUpdate,
      updateAvailable,
      setUpdateAvailable,
      checkedForUpdates,
      setCheckedForUpdates,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [requestedInstallUpdate, shutdownManager, terminateSubprocesses, updateAvailable, checkedForUpdates]
  );

  return <ElectronContext.Provider value={attributesMemo}>{children}</ElectronContext.Provider>;
}
