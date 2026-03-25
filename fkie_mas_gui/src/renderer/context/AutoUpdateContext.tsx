import { ProgressInfo, UpdateInfo } from "electron-updater";
import { createContext, useCallback, useContext, useEffect, useMemo, useState } from "react";
import semver from "semver";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { JSONObject, TAutoUpdateManager } from "@/types";
import packageJson from "../../../package.json";
import { useLoggingContext } from "../hooks/useLoggingContext";
import { useNavigationContext } from "../hooks/useNavigationContext";
import { useRosContext } from "../hooks/useRosContext";
import { useSettingsContext } from "../hooks/useSettingsContext";
import { CmdType } from "../providers";

/**
 * Context providing automatic update management for both AppImage and Debian builds.
 */
export interface IAutoUpdateContext {
  autoUpdateManager: TAutoUpdateManager | null;
  checkedThisRun: boolean;
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
  installing: boolean;
  setLocalProviderId: (providerId: string) => void;
}

export const AutoUpdateContext = createContext<IAutoUpdateContext | null>(null);

interface IAutoUpdateProviderProps {
  children: React.ReactNode;
}

export const AutoUpdateProvider = ({
  children,
}: IAutoUpdateProviderProps): ReturnType<React.FC<IAutoUpdateProviderProps>> => {
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  const MIN_DELAY_AUTO_CHECK = 86400; // 1 day (in seconds)

  // ==== State ====
  const [autoUpdateManager, setAutoUpdateManager] = useState<TAutoUpdateManager | null>(null);
  const [checkingForUpdate, setCheckingForUpdate] = useState(false);
  const [checkedThisRun, setCheckedThisRun] = useState(false);
  const [updateAvailable, setUpdateAvailable] = useLocalStorage<UpdateInfo | null>("AutoUpdate:updateAvailable", null);
  const [downloadProgress, setDownloadProgress] = useState<ProgressInfo | null>(null);
  const [updateError, setUpdateError] = useState("");
  const [requestedInstallUpdate, setRequestedInstallUpdate] = useState(false);
  const [installing, setInstalling] = useState(false);
  const [localProviderId, setLocalProviderId] = useState("");
  const [isAppImage, setIsAppImage] = useState(true);
  const [updateChannel, setStoredChannel] = useLocalStorage<"prerelease" | "release" | string>(
    "AutoUpdate:updateChannel",
    "release"
  );
  const [checkTimestamp, setCheckTimestamp] = useLocalStorage<number>("AutoUpdate:checkTimestamp", 0);

  /** Builds CLI command for updating MAS */
  const getUpdateCli = useCallback(
    (gui: boolean, ros: boolean): string => {
      const channelOpt =
        updateChannel === "prerelease" ? " -p" : updateChannel !== "release" ? ` -s ${updateChannel}` : "";
      const args = `${!gui ? " -r" : ""}${!ros ? " -g" : ""}`;
      const branch = updateChannel === "prerelease" ? "devel" : "master";
      return `wget -O /tmp/install_mas_debs.sh https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/${branch}/install_mas_debs.sh && bash /tmp/install_mas_debs.sh${channelOpt}${args}`;
    },
    [updateChannel]
  );

  /** Checks for new release versions on GitHub or via AppImage updater */
  const checkForUpdate = useCallback(
    (channelParam?: "prerelease" | "release" | string): void => {
      const channel = channelParam || updateChannel;
      logCtx.info(`Checking for new ${channel} ${isAppImage ? "AppImage" : "Debian"} update`, "", "check update");

      setUpdateAvailable(null);
      setUpdateError("");
      setDownloadProgress(null);
      setCheckingForUpdate(true);
      setCheckTimestamp(Math.floor(Date.now() / 1000));

      if (isAppImage && autoUpdateManager) {
        if (["prerelease", "release"].includes(channel)) {
          autoUpdateManager.setChannel(channel as "prerelease" | "release");
        }
        autoUpdateManager.checkForUpdate();
        setCheckedThisRun(true);
      } else {
        fetchRelease(channel);
      }
    },
    [autoUpdateManager, isAppImage, updateChannel]
  );

  /** Fetches release info directly from GitHub */
  const fetchRelease = useCallback(async (channel: "prerelease" | "release" | string): Promise<void> => {
    try {
      setUpdateError("");
      const response = await fetch("https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases");
      if (!response.ok) throw new Error(`HTTP ${response.status}`);

      const data = await response.json();
      if (!Array.isArray(data)) throw new Error(`Unexpected GitHub API response: ${JSON.stringify(data)}`);

      let release: JSONObject | undefined;
      if (channel === "release") release = data.find((r) => !r.prerelease);
      else if (channel === "prerelease") release = data[0];
      else release = data.find((r) => r.name === channel);

      if (!release) throw new Error(`No ${channel} release found`);

      if (packageJson.version !== release.name) {
        // build changelog for all newer versions
        const changes = data
          .filter((r) => semver.gt(r.name, packageJson.version))
          .map((r) => r.body?.replace("Changes", getTitle(r)).replace("\r\n\r\n", "<br/>").replaceAll("\r\n", "<br/>"))
          .join("<br/><br/>");

        setUpdateAvailable({
          version: release.name,
          releaseDate: release.published_at,
          releaseNotes: changes || release.body,
        } as UpdateInfo);
      }
    } catch (e) {
      setUpdateError(e instanceof Error ? e.message : "Unknown error during fetchRelease()");
    } finally {
      setCheckingForUpdate(false);
      setCheckedThisRun(true);
    }
  }, []);

  const getTitle = (release: JSONObject) =>
    `Changes in version ${release.name} (${(release.published_at as string).split("T")[0]})${release.prerelease ? " prerelease" : ""}:`;

  const installDebian = useCallback(
    async (gui: boolean, ros: boolean): Promise<void> => {
      logCtx.info(`Starting Debian update to ${updateAvailable?.version}`, "", "install update");
      setUpdateError("");
      const providerId = getLocalProviderId();

      if (!providerId && !window.commandExecutor) {
        return setUpdateError("Could not connect to the local TTYD. Please start it first.");
      }

      setInstalling(true);
      if (providerId) {
        await navCtx.openTerminal(CmdType.CMD, providerId, "", "", getUpdateCli(gui, ros), false, false);
      } else {
        const result = await window.commandExecutor?.execTerminal(
          null,
          "'update mas'",
          `'${getUpdateCli(gui, ros)} -w'`
        );
        if (result) {
          if (!result.result) setUpdateError(result.message);
        } else {
          setUpdateError("commandExecutor not available");
        }
      }
      setInstalling(false);
    },
    [getUpdateCli, navCtx, logCtx, updateAvailable]
  );

  const requestInstallUpdate = useCallback(() => {
    setUpdateError("");
    setRequestedInstallUpdate(true);
    autoUpdateManager?.quitAndInstall();
  }, [autoUpdateManager]);

  const setUpdateChannel = useCallback(
    (channel: "prerelease" | "release" | string): void => {
      if (updateChannel === channel) return;
      setStoredChannel(channel);
      if (["prerelease", "release"].includes(channel))
        autoUpdateManager?.setChannel(channel as "prerelease" | "release");
    },
    [updateChannel, autoUpdateManager]
  );

  const getLocalProviderId = () => rosCtx.getLocalProvider()[0]?.id || "";

  const autoCheckAllowed = (timestamp: number) => Math.floor(Date.now() / 1000) - timestamp > MIN_DELAY_AUTO_CHECK;

  /** Determine if we are using AppImage and maybe trigger background check */
  const updateIsAppImage = useCallback(
    async (manager: TAutoUpdateManager): Promise<void> => {
      const result = await manager.isAppImage();
      setIsAppImage(result);
      if (settingsCtx.get("checkForUpdates") && result && autoCheckAllowed(checkTimestamp)) {
        checkForUpdate(updateChannel);
      }
    },
    [settingsCtx, checkTimestamp, updateChannel]
  );

  // ==== Effects ====
  useEffect(() => {
    if (window.autoUpdate) setAutoUpdateManager(window.autoUpdate);
  }, []);

  useEffect(() => {
    if (!autoUpdateManager) return;

    // Register electron-updater event hooks
    autoUpdateManager.onCheckingForUpdate(setCheckingForUpdate);
    autoUpdateManager.onUpdateAvailable((info) => {
      setUpdateAvailable(info);
      logCtx.info(`New version ${info.version} available`, "", "update available");
    });
    autoUpdateManager.onUpdateNotAvailable(() => setUpdateAvailable(null));
    autoUpdateManager.onDownloadProgress(setDownloadProgress);
    autoUpdateManager.onUpdateDownloaded(setUpdateAvailable);
    autoUpdateManager.onUpdateError((msg) => {
      setUpdateError(msg);
      if (msg.includes("APPIMAGE")) setIsAppImage(false);
    });

    updateIsAppImage(autoUpdateManager);
  }, [autoUpdateManager]);

  // Initialize the local provider id if not set
  useEffect(() => {
    if (!localProviderId) {
      const local = rosCtx.getLocalProvider();
      if (local.length > 0) setLocalProviderId(local[0].id);
    }
  }, [rosCtx.providers]);

  // Auto-check for updates if enough time elapsed
  useEffect(() => {
    if (localProviderId && autoCheckAllowed(checkTimestamp)) checkForUpdate(updateChannel);
  }, [localProviderId, updateChannel, checkTimestamp]);

  // Clear stored update if same version as current
  useEffect(() => {
    if (updateAvailable?.version === packageJson.version) setUpdateAvailable(null);
  }, [updateAvailable]);

  const contextValue = useMemo(
    () => ({
      autoUpdateManager,
      checkForUpdate,
      checkedThisRun,
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
      installing,
      setLocalProviderId,
    }),
    [
      autoUpdateManager,
      checkedThisRun,
      updateChannel,
      checkTimestamp,
      checkingForUpdate,
      updateAvailable,
      downloadProgress,
      updateError,
      requestedInstallUpdate,
      isAppImage,
      installing,
    ]
  );

  return <AutoUpdateContext.Provider value={contextValue}>{children}</AutoUpdateContext.Provider>;
};

/** Hook to access the auto-update context. Throws if used outside provider. */
export const useAutoUpdateContext = (): IAutoUpdateContext => {
  const ctx = useContext(AutoUpdateContext);
  if (!ctx) throw new Error("useAutoUpdateContext must be used inside AutoUpdateProvider");
  return ctx;
};
