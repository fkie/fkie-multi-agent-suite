import { ProgressInfo, UpdateInfo } from "electron-updater";
import { TAutoUpdateManager } from "@/types";
import { createContext, useContext, useEffect, useMemo, useState } from "react";
import { LoggingContext } from "./LoggingContext";
import { SettingsContext } from "./SettingsContext";

export interface IAutoUpdateContext {
  autoUpdateManager: TAutoUpdateManager | null;
  checkForUpdate: () => void;
  checkingForUpdate: boolean;
  updateAvailable: UpdateInfo | null;
  downloadProgress: ProgressInfo | null;
  updateError: string;
  requestInstallUpdate: () => void;
  requestedInstallUpdate: boolean;
}

export const DEFAULT = {
  autoUpdateManager: null,
  checkForUpdate: () => {},
  checkingForUpdate: false,
  updateAvailable: null,
  downloadProgress: null,
  updateError: "",
  requestInstallUpdate: () => {},
  requestedInstallUpdate: false,
};

interface IAutoUpdateProviderComponent {
  children: React.ReactNode;
}

export const AutoUpdateContext = createContext<IAutoUpdateContext>(DEFAULT);

export function AutoUpdateProvider({
  children,
}: IAutoUpdateProviderComponent): ReturnType<React.FC<IAutoUpdateProviderComponent>> {
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [autoUpdateManager, setAutoUpdateManager] = useState<TAutoUpdateManager | null>(null);
  const [checkingForUpdate, setCheckingForUpdate] = useState(false);
  const [updateAvailable, setUpdateAvailable] = useState<UpdateInfo | null>(null);
  const [downloadProgress, setDownloadProgress] = useState<ProgressInfo | null>(null);
  const [updateError, setUpdateError] = useState<string>("");
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState<boolean>(false);

  function checkForUpdate() {
    logCtx.debug(`Check for new release on https://github.com/fkie/fkie-multi-agent-suite`, "");
    setUpdateAvailable(null);
    setUpdateError("");
    setCheckingForUpdate(false);
    setDownloadProgress(null);
    autoUpdateManager?.checkForUpdate();
  }

  function requestInstallUpdate() {
    setUpdateError("");
    setRequestedInstallUpdate(true);
    autoUpdateManager?.quitAndInstall();
  }

  useEffect(() => {
    if (window.autoUpdate) {
      setAutoUpdateManager(window.autoUpdate);
    }
  }, []);

  // Effect to initialize the auto update callbacks
  useEffect(() => {
    autoUpdateManager?.onCheckingForUpdate((state) => {
      setCheckingForUpdate(state);
    });
    autoUpdateManager?.onUpdateAvailable((info) => {
      setUpdateAvailable(info);
      logCtx.info(`New version ${info.version} available! Please update in 'About'-tab in settings dialog.`, "");
    });
    autoUpdateManager?.onUpdateNotAvailable(() => {
      setUpdateAvailable(null);
    });
    autoUpdateManager?.onDownloadProgress((info) => {
      setDownloadProgress(info);
    });
    autoUpdateManager?.onUpdateDownloaded((info) => {
      setDownloadProgress(null);
      setUpdateAvailable(info);
      // extended info with "downloadedFile" parameter
    });
    autoUpdateManager?.onUpdateError((message) => {
      setUpdateError(message);
      logCtx.debug("update failed", message);
    });

    if (settingsCtx.get("checkForUpdates")) {
      checkForUpdate();
    }
  }, [autoUpdateManager]);

  const attributesMemo = useMemo(
    () => ({
      autoUpdateManager,
      checkForUpdate,
      checkingForUpdate,
      updateAvailable,
      downloadProgress,
      updateError,
      requestedInstallUpdate,
      requestInstallUpdate,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [autoUpdateManager, checkingForUpdate, updateAvailable, downloadProgress, updateError, requestedInstallUpdate]
  );

  return <AutoUpdateContext.Provider value={attributesMemo}>{children}</AutoUpdateContext.Provider>;
}
