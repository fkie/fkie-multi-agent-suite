import { ProgressInfo, UpdateInfo } from "electron-updater";
import { createContext, useCallback, useContext, useEffect, useMemo, useState } from "react";
import semver from "semver";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { JSONObject, TAutoUpdateManager } from "@/types";
import packageJson from "../../../package.json";
import { CmdType } from "../providers";
import { LoggingContext } from "./LoggingContext";
import NavigationContext from "./NavigationContext";
import RosContext from "./RosContext";
import { SettingsContext } from "./SettingsContext";

export interface IAutoUpdateContext {
  autoUpdateManager: TAutoUpdateManager | null;
  checkForUpdate: () => void;
  updateChannel: string;
  checkTimestamp: number;
  setUpdateChannel: (channelType: "prerelease" | "release" | string) => void;
  checkingForUpdate: boolean;
  updateAvailable: UpdateInfo | null;
  downloadProgress: ProgressInfo | null;
  updateError: string;
  requestInstallUpdate: () => void;
  requestedInstallUpdate: boolean;
  isAppImage: boolean;
  getUpdateCli: (gui: boolean, ros: boolean) => string;
  installDebian: (gui: boolean, ros: boolean) => void;
  setLocalProviderId: (providerId: string) => void;
}

export const DEFAULT = {
  autoUpdateManager: null,
  updateChannel: "release",
  checkTimestamp: 0,
  checkForUpdate: (): void => {},
  setUpdateChannel: (): void => {},
  checkingForUpdate: false,
  updateAvailable: null,
  downloadProgress: null,
  updateError: "",
  requestInstallUpdate: (): void => {},
  requestedInstallUpdate: false,
  isAppImage: true,
  getUpdateCli: (): string => {
    return "";
  },
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
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const minDelayBetweenAutoChecks = 86400; // one day in seconds
  const [autoUpdateManager, setAutoUpdateManager] = useState<TAutoUpdateManager | null>(null);
  const [checkingForUpdate, setCheckingForUpdate] = useState(false);
  const [updateAvailable, setUpdateAvailable] = useLocalStorage<UpdateInfo | null>("AutoUpdate:updateAvailable", null);
  const [downloadProgress, setDownloadProgress] = useState<ProgressInfo | null>(null);
  const [updateError, setUpdateError] = useState<string>("");
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState<boolean>(false);
  const [localProviderId, setLocalProviderId] = useState<string>("");
  const [isAppImage, setIsAppImage] = useState<boolean>(true);
  const [updateChannel, setChannel] = useLocalStorage<"prerelease" | "release" | string>(
    "AutoUpdate:updateChannel",
    "release"
  );
  const [checkTimestamp, setCheckTimestamp] = useLocalStorage<number>("AutoUpdate:checkTimestamp", 0);

  function getUpdateCli(gui: boolean, ros: boolean): string {
    const prereleaseOpt =
      updateChannel === "prerelease"
        ? " -p" // prerelease channel
        : updateChannel !== "release" && updateChannel.length > 0
          ? ` -s ${updateChannel}` // specific release
          : ""; // latest release
    const noGuiOpt = !gui ? " -r" : "";
    const noRosOpt = !ros ? " -g" : "";
    const args = prereleaseOpt || noGuiOpt || noRosOpt ? ` -s -- ${prereleaseOpt}${noGuiOpt}${noRosOpt}` : "";
    return `wget -qO - https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/${prereleaseOpt ? "devel" : "master"}/install_mas_debs.sh | bash${args}`;
  }

  function checkForUpdate(channel?: "prerelease" | "release" | string): void {
    logCtx.info(
      `Check for new ${updateChannel}${isAppImage ? " for AppImage" : " for debian"} on https://github.com/fkie/fkie-multi-agent-suite`,
      "",
      false
    );
    setUpdateAvailable(null);
    setUpdateError("");
    setCheckingForUpdate(false);
    setDownloadProgress(null);
    setCheckTimestamp(Math.floor(Date.now() / 1000));
    if (isAppImage && autoUpdateManager) {
      if (channel && ["prerelease", "release"].includes(channel)) {
        autoUpdateManager.setChannel(channel as "prerelease" | "release");
      }
      autoUpdateManager.checkForUpdate();
    } else {
      fetchRelease(channel ? channel : updateChannel);
    }
  }

  function installDebian(gui: boolean, ros: boolean): void {
    logCtx.info(
      `start update for gui(${gui}) on ros(${ros}) on channel (${updateChannel}) to version (${updateAvailable?.version})`,
      "",
      false
    );
    setUpdateError("");
    const providerId = getLocalProviderId();
    if (providerId) {
      navCtx.openTerminal(CmdType.TERMINAL, providerId, "", "", getUpdateCli(gui, ros), false, false);
    } else {
      setUpdateError("Could not connect to the local TTYD. Please start the local TTYD.");
    }
  }

  function getTitle(release: JSONObject): string {
    const date = (release.published_at as string).split("T")[0];
    return `Changes in version ${release.name} (${date})${release.prerelease ? " prerelease" : ""}:`;
  }

  const fetchRelease = async (channel: "prerelease" | "release" | string): Promise<void> => {
    try {
      setUpdateError("");
      setCheckingForUpdate(true);
      const response = await fetch("https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases");
      if (!response.ok) {
        setUpdateError(`HTTP Response Status Code: ${response.status}`);
        return;
      }
      const data = await response.json();
      if (data.message) {
        setUpdateError(`${JSON.stringify(data)}`);
        return;
      }
      let release: JSONObject | undefined = undefined;
      if (data.length > 0) {
        if (channel === "prerelease") {
          // take the first version, regardless of whether it is labeled as a pre-release version
          release = data[0];
          if (semver.gt(data[0].name, packageJson.version)) {
            setUpdateAvailable({
              version: data[0].name,
              releaseDate: data[0].published_at,
              releaseNotes: data[0].body,
            } as UpdateInfo);
          }
        } else if (channel === "release") {
          // take first not prerelease version
          for (const r of data) {
            if (!r.prerelease) {
              release = r;
              break;
            }
          }
        } else {
          // try to find specified release
          for (const r of data) {
            if (r.name === channel) {
              release = r;
              break;
            }
          }
        }
      } else {
        setUpdateError("No releases found on github.com");
        return;
      }
      if (release) {
        if (channel === release.name && packageJson.version !== release.name) {
          // specified release
          setUpdateAvailable({
            version: release.name,
            releaseDate: release.published_at,
            releaseNotes: (release.body as string)
              ?.replace("Changes", getTitle(release))
              .replace("\r\n\r\n", "<br/>")
              .replaceAll("\r\n", "<br/>"),
          } as UpdateInfo);
        } else if (["prerelease", "release"].includes(channel) && semver.gt(release.name, packageJson.version)) {
          // new release
          // create history
          let changes: string = "";
          for (const r of data) {
            if (semver.gt(r.name, packageJson.version)) {
              if (changes.length > 0) {
                changes += "<br/><br/>";
              }
              changes += r.body
                ?.replace("Changes", getTitle(r))
                .replace("\r\n\r\n", "<br/>")
                .replaceAll("\r\n", "<br/>");
            } else {
              break;
            }
          }
          setUpdateAvailable({
            version: release.name,
            releaseDate: release.published_at,
            releaseNotes: changes,
          } as UpdateInfo);
        }
      } else {
        setUpdateError(`No ${channel} found`);
      }
    } catch (error) {
      let errorMessage = "Failed to fetch release";
      if (error instanceof Error) {
        errorMessage = error.message;
      }
      setUpdateError(errorMessage);
    } finally {
      setCheckingForUpdate(false);
    }
  };

  function requestInstallUpdate(): void {
    setUpdateError("");
    setRequestedInstallUpdate(true);
    autoUpdateManager?.quitAndInstall();
  }

  function setUpdateChannel(channelType: "prerelease" | "release" | string): void {
    if (updateChannel !== channelType) {
      setChannel(channelType);
      if (["prerelease", "release"].includes(channelType))
        autoUpdateManager?.setChannel(channelType as "prerelease" | "release");
      // checkForUpdate(channelType);
    }
  }

  function getLocalProviderId(): string {
    const localProvider = rosCtx.getLocalProvider();
    if (localProvider.length > 0) {
      return localProvider[0].id;
    }
    return "";
  }

  function autoCheckAllowed(timestamp: number): boolean {
    return Math.floor(Date.now() / 1000) - timestamp > minDelayBetweenAutoChecks;
  }

  const updateIsAppImage = useCallback(
    async (autoUpdateManager: TAutoUpdateManager): Promise<void> => {
      const isAppImageLocal = await autoUpdateManager.isAppImage();
      setIsAppImage(isAppImageLocal);
      if (settingsCtx.get("checkForUpdates") && isAppImageLocal) {
        if (autoCheckAllowed(checkTimestamp)) {
          checkForUpdate(updateChannel);
        }
      }
    },
    [checkTimestamp, updateChannel]
  );

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
      logCtx.info(`New version ${info.version} of AppImage is available!`, "");
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
    updateIsAppImage(autoUpdateManager);
  }, [autoUpdateManager]);

  useEffect(() => {
    if (!localProviderId) {
      const localProvider = rosCtx.getLocalProvider();
      if (localProvider.length > 0) {
        setLocalProviderId(localProvider[0].id);
      }
    }
  }, [rosCtx.providersConnected]);

  useEffect(() => {
    if (localProviderId) {
      if (autoCheckAllowed(checkTimestamp)) {
        checkForUpdate(updateChannel);
      }
    }
  }, [localProviderId, updateChannel, checkTimestamp]);

  useEffect(() => {
    // on start after update, we have to check if current version is the last available update
    if (updateAvailable?.version === packageJson.version) {
      setUpdateAvailable(null);
    }
  }, [updateAvailable])

  const attributesMemo = useMemo(
    () => ({
      autoUpdateManager,
      checkForUpdate,
      updateChannel,
      checkTimestamp,
      setUpdateChannel,
      checkingForUpdate,
      updateAvailable,
      downloadProgress,
      updateError,
      requestedInstallUpdate,
      requestInstallUpdate,
      isAppImage,
      getUpdateCli,
      installDebian,
      setLocalProviderId,
    }),
    [
      autoUpdateManager,
      updateChannel,
      checkTimestamp,
      checkingForUpdate,
      updateAvailable,
      downloadProgress,
      updateError,
      requestedInstallUpdate,
      isAppImage,
    ]
  );

  return <AutoUpdateContext.Provider value={attributesMemo}>{children}</AutoUpdateContext.Provider>;
}
