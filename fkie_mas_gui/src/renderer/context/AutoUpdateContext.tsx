import { ProgressInfo, UpdateInfo } from "electron-updater";
import { createContext, useContext, useEffect, useMemo, useState } from "react";
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
  const [checkedChannel, setCheckedChannel] = useState<string>("");
  const [autoUpdateManager, setAutoUpdateManager] = useState<TAutoUpdateManager | null>(null);
  const [checkingForUpdate, setCheckingForUpdate] = useState(false);
  const [updateAvailable, setUpdateAvailable] = useState<UpdateInfo | null>(null);
  const [downloadProgress, setDownloadProgress] = useState<ProgressInfo | null>(null);
  const [updateError, setUpdateError] = useState<string>("");
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState<boolean>(false);
  const [localProviderId, setLocalProviderId] = useState<string>("");
  const [isAppImage, setIsAppImage] = useState<boolean>(true);
  const [updateChannel, setChannel] = useLocalStorage<"prerelease" | "release" | string>(
    "AutoUpdate:updateChannel",
    "release"
  );

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
    logCtx.info(`Check for new ${updateChannel} on https://github.com/fkie/fkie-multi-agent-suite`, "", false);
    setUpdateAvailable(null);
    setUpdateError("");
    setCheckingForUpdate(false);
    setDownloadProgress(null);
    if (isAppImage) {
      if (channel && ["prerelease", "release"].includes(channel)) {
        autoUpdateManager?.setChannel(channel as "prerelease" | "release");
      }
      autoUpdateManager?.checkForUpdate();
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
        setUpdateError("Network error");
      }
      const data = await response.json();
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
      setCheckedChannel(channel);
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
      setCheckedChannel(updateChannel);
      logCtx.info(`New version ${info.version} available!`, "");
    });
    autoUpdateManager?.onUpdateNotAvailable(() => {
      setUpdateAvailable(null);
      setCheckedChannel(updateChannel);
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

    if (settingsCtx.get("checkForUpdates") && !checkedChannel) {
      checkForUpdate(updateChannel);
    }
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
    if (localProviderId && updateChannel !== checkedChannel) {
      checkForUpdate(updateChannel);
    }
  }, [localProviderId, updateChannel, checkedChannel]);

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
      getUpdateCli,
      installDebian,
      setLocalProviderId,
    }),
    [autoUpdateManager, checkingForUpdate, updateAvailable, downloadProgress, updateError, requestedInstallUpdate]
  );

  return <AutoUpdateContext.Provider value={attributesMemo}>{children}</AutoUpdateContext.Provider>;
}
