import { ProgressInfo, UpdateInfo } from "electron-updater";
import { createContext, useContext, useEffect, useMemo, useState } from "react";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { TAutoUpdateManager } from "@/types";
import packageJson from "../../../package.json";
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
  isAppImage: boolean;
  installDebian: (gui: boolean, ros: boolean) => void;
  setLocalProviderId: (providerId: string) => void;
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
  isAppImage: true,
  installDebian: (): void => {},
  setLocalProviderId: (): void => {},
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
  const [localProviderId, setLocalProviderId] = useState<string>("");
  const [isAppImage, setIsAppImage] = useState<boolean>(true);
  const [updateChannel, setChannel] = useLocalStorage<"prerelease" | "release">("AutoUpdate:updateChannel", "release");

  function checkForUpdate(channel?: "prerelease" | "release"): void {
    logCtx.info(`Check for new ${updateChannel} on https://github.com/fkie/fkie-multi-agent-suite`, "", false);
    setUpdateAvailable(null);
    setUpdateError("");
    setCheckingForUpdate(false);
    setDownloadProgress(null);
    if (isAppImage) {
      if (channel) {
        autoUpdateManager?.setChannel(channel);
      }
      autoUpdateManager?.checkForUpdate();
    } else {
      fetchRelease(channel);
    }
  }

  function installDebian(gui: boolean, ros: boolean): void {
    logCtx.info(
      `start update for gui(${gui}) on ros(${ros}) on channel (${updateChannel}) to version (${updateAvailable?.version})`,
      "",
      true
    );
    //TODO
    //navCtx.openTerminal(type, providerId, nodeName, screen, "", externalKeyModifier, openInTerminal);
  }

  const fetchRelease = async (channel?: "prerelease" | "release"): Promise<void> => {
    try {
      setUpdateError("");
      setCheckingForUpdate(true);
      console.log(`${channel}`);
      if (channel === "release") {
        const response = await fetch("https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases/latest");
        if (!response.ok) {
          setUpdateError("Network error");
        }
        const data = await response.json();
        if (data.name !== packageJson.version) {
          setUpdateAvailable({ version: data.name, releaseDate: data.published_at } as UpdateInfo);
        }
      } else {
        const response = await fetch("https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases");
        if (!response.ok) {
          setUpdateError("Network error");
        }
        const data = await response.json();
        const prereleases = data.filter((release) => release.prerelease);
        if (prereleases.length > 0) {
          if (prereleases[0].name !== packageJson.version) {
            setUpdateAvailable({ version: prereleases[0].name, releaseDate: data.published_at } as UpdateInfo);
          }
        } else {
          setUpdateError("No prereleases found");
        }
      }
    } catch (error) {
      setUpdateError(error.message);
    } finally {
      setCheckingForUpdate(false);
    }
  };

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
      if (message.includes("APPIMAGE")) {
        setIsAppImage(false);
      }
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
      isAppImage,
      installDebian,
      setLocalProviderId,
    }),
    [autoUpdateManager, checkingForUpdate, updateAvailable, downloadProgress, updateError, requestedInstallUpdate]
  );

  return <AutoUpdateContext.Provider value={attributesMemo}>{children}</AutoUpdateContext.Provider>;
}
