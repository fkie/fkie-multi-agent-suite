import { ProgressInfo, UpdateDownloadedEvent, UpdateInfo } from "electron-updater";

export const AutoUpdateManagerEvents = {
  checkForUpdate: "au:check-for-updates",
  quitAndInstall: "au:quit-and-install",
  setChannel: "au:set-channel",
  isAppImage: "au:is-app-image",
  onCheckingForUpdate: "au:checking-for-update",
  onUpdateAvailable: "au:update-available",
  onUpdateNotAvailable: "au:update-not-available",
  onDownloadProgress: "au:download-progress",
  onUpdateDownloaded: "au:update-downloaded",
  onUpdateError: "au:update-error",
};

export type AuCheckingForUpdateCallback = (state: boolean) => void;
export type AuUpdateAvailableCallback = (info: UpdateInfo) => void;
export type AuDownloadProgressCallback = (info: ProgressInfo) => void;
export type AuUpdateDownloadedCallback = (info: UpdateDownloadedEvent) => void;
export type AuUpdateErrorCallback = (message: string) => void;

export type TAutoUpdateManager = {
  checkForUpdate: () => void;
  quitAndInstall: () => void;
  setChannel: (channelType: "prerelease" | "release") => void;
  isAppImage: () => Promise<boolean>;

  onCheckingForUpdate: (callback: AuCheckingForUpdateCallback) => void;
  onUpdateAvailable: (callback: AuUpdateAvailableCallback) => void;
  onUpdateNotAvailable: (callback: AuUpdateAvailableCallback) => void;
  onDownloadProgress: (callback: AuDownloadProgressCallback) => void;
  onUpdateDownloaded: (callback: AuUpdateDownloadedCallback) => void;
  onUpdateError: (callback: AuUpdateErrorCallback) => void;
};
