import { TAutoUpdateManager } from "@/types";
import { ProgressInfo, UpdateInfo } from "electron-updater";
import { createContext, useContext, useEffect, useMemo, useState } from "react";
import useLocalStorage from "../hooks/useLocalStorage";
import { LoggingContext } from "./LoggingContext";
import { SettingsContext } from "./SettingsContext";

export interface IAutoUpdateContext {
  autoUpdateManager: TAutoUpdateManager | null;
  checkForUpdate: () => void;
  updateChannel: string;
  setUpdateChannel: (channelType: "prerelease" | "release") => void;
  checkingForUpdate: boolean;
  updateAvailable: UpdateInfo | null;
  downloadProgress: ProgressInfo | null;
  updateError: string;
  requestInstallUpdate: () => void;
  requestedInstallUpdate: boolean;
}

export const DEFAULT = {
  autoUpdateManager: null,
  updateChannel: "release",
  checkForUpdate: (): void => {},
  setUpdateChannel: (): void => {},
  checkingForUpdate: false,
  updateAvailable: null,
  downloadProgress: null,
  updateError: "",
  requestInstallUpdate: (): void => {},
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
  const [updateChannel, setChannel] = useLocalStorage<"prerelease" | "release">("AutoUpdate:updateChannel", "release");

  function checkForUpdate(channel?: "prerelease" | "release"): void {
    logCtx.info(`Check for new ${updateChannel} on https://github.com/fkie/fkie-multi-agent-suite`, "", false);
    setUpdateAvailable(null);
    setUpdateError("");
    setCheckingForUpdate(false);
    setDownloadProgress(null);
    if (channel) {
      autoUpdateManager?.setChannel(channel);
    }
    autoUpdateManager?.checkForUpdate();
  }

  function requestInstallUpdate(): void {
    setUpdateError("");
    setRequestedInstallUpdate(true);
    autoUpdateManager?.quitAndInstall();
  }

  function setUpdateChannel(channelType: "prerelease" | "release"): void {
    if (updateChannel != channelType) {
      setChannel(channelType);
      autoUpdateManager?.setChannel(channelType);
      checkForUpdate(channelType);
    }
  }

  useEffect(() => {
    if (window.autoUpdate) {
      setAutoUpdateManager(window.autoUpdate);
    }
  }, []);

  // Effect to initialize the auto update callbacks
  useEffect(() => {
    if (!autoUpdateManager) return;
    autoUpdateManager?.onCheckingForUpdate((state) => {
      setCheckingForUpdate(state);
    });
    autoUpdateManager?.onUpdateAvailable((info) => {
      setUpdateAvailable(info);
      logCtx.info(`New version ${info.version} available!`, "");
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
      checkForUpdate(updateChannel);
    }
  }, [autoUpdateManager]);

  const attributesMemo = useMemo(
    () => ({
      autoUpdateManager,
      checkForUpdate,
      updateChannel,
      setUpdateChannel,
      checkingForUpdate,
      updateAvailable,
      downloadProgress,
      updateError,
      requestedInstallUpdate,
      requestInstallUpdate,
    }),
    [autoUpdateManager, checkingForUpdate, updateAvailable, downloadProgress, updateError, requestedInstallUpdate]
  );

  return <AutoUpdateContext.Provider value={attributesMemo}>{children}</AutoUpdateContext.Provider>;
}
