import licenses from "@/renderer/deps-licenses.json";
import ErrorOutlineIcon from "@mui/icons-material/ErrorOutline";
import { Box, Button, IconButton, Link, Stack, Typography } from "@mui/material";
import CircularProgress from "@mui/material/CircularProgress";
import LinearProgress from "@mui/material/LinearProgress";
import PropTypes from "prop-types";
import { useContext, useEffect, useState } from "react";
import packageJson from "../../../../package.json";
import { ElectronContext } from "../../context/ElectronContext";
import { LoggingContext } from "../../context/LoggingContext";
import { SettingsContext } from "../../context/SettingsContext";
import CopyButton from "../UI/CopyButton";

function LinearProgressWithLabel({ value, ...props }) {
  return (
    <Box sx={{ display: "flex", alignItems: "center" }}>
      <Box sx={{ width: "100%", mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box sx={{ minWidth: 35 }}>
        <Typography variant="body2" color="text.secondary">{`${Math.round(value)}%`}</Typography>
      </Box>
    </Box>
  );
}
LinearProgressWithLabel.propTypes = {
  value: PropTypes.number.isRequired,
};

function About() {
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const electronCtx = useContext(ElectronContext);
  const [updateError, setUpdateError] = useState("");
  const [checkingForUpdate, setCheckingForUpdate] = useState(false);
  const [updateAvailable, setUpdateAvailable] = useState(null);
  const [updateNotAvailable, setUpdateNotAvailable] = useState(null);
  const [updateDownloaded, setUpdateDownloaded] = useState(false);
  const [downloadedProgress, setDownloadedProgress] = useState(null);
  const [openErrorTooltip, setOpenErrorTooltip] = useState(false);

  function checkForUpdate(force) {
    if (electronCtx.checkedForUpdates && !force) return;
    logCtx.debug(`Check for new release on https://github.com/fkie/fkie-multi-agent-suite`);
    electronCtx.setUpdateAvailable("");
    electronCtx.setCheckedForUpdates(true);
    setUpdateError("");
    setCheckingForUpdate(false);
    setUpdateAvailable(null);
    setUpdateNotAvailable(null);
    setUpdateDownloaded(null);
    setDownloadedProgress(null);
    window.autoUpdate.send("check-for-updates");
  }

  function installUpdate() {
    setUpdateError("");
    electronCtx.setRequestedInstallUpdate(true);
    window.autoUpdate.send("quit-and-install");
  }

  // register icp events published by ICP/AutoUpdateManager
  useEffect(() => {
    if (!window.autoUpdate) return;

    window.autoUpdate.receive("checking-for-update", () => {
      setCheckingForUpdate(true);
    });

    window.autoUpdate.receive("update-available", (data) => {
      setCheckingForUpdate(false);
      setUpdateAvailable(data);
      electronCtx.setUpdateAvailable(data.version);
      logCtx.info(`New version ${data.version} available! Please update in 'About'-tab in settings dialog.`);
    });

    window.autoUpdate.receive("update-not-available", (data) => {
      setCheckingForUpdate(false);
      setUpdateNotAvailable(data);
    });

    window.autoUpdate.receive("download-progress", (data) => {
      setCheckingForUpdate(false);
      setDownloadedProgress(data);
    });
    window.autoUpdate.receive("update-downloaded", (data) => {
      setUpdateDownloaded(data);
    });

    window.autoUpdate.receive("update-error", (data) => {
      setCheckingForUpdate(false);
      setUpdateError(data);
      logCtx.debug("update failed", data);
    });
    if (settingsCtx.get("checkForUpdates")) {
      checkForUpdate();
    }
  }, []);

  return (
    <Stack paddingTop={2} spacing={0.2} overflow="hidden">
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          Version:
        </Typography>
        <Typography variant="body">{packageJson.version}</Typography>

        {window.autoUpdate && (
          <Stack spacing={0.2} direction="column">
            <Stack spacing={0.2} direction="row">
              {checkingForUpdate && (
                <Box sx={{ display: "flex" }}>
                  <CircularProgress size="1em" />
                </Box>
              )}
              {updateAvailable && !updateDownloaded && (
                <Stack spacing={0.2} direction="row">
                  <Typography variant="body">downloading {updateAvailable.version}</Typography>
                  {downloadedProgress && downloadedProgress.percent < 100 && (
                    <Box sx={{ width: "100%" }}>
                      <LinearProgressWithLabel value={downloadedProgress?.percent} />
                    </Box>
                  )}
                </Stack>
              )}
              {updateNotAvailable && <Typography variant="body">Your version is up to date!</Typography>}
              {updateDownloaded && (
                <Stack spacing={0.2} direction="row">
                  <Typography variant="body">Version {updateAvailable?.version} downloaded</Typography>
                  <Button color="primary" onClick={() => installUpdate()} variant="text">
                    Restart and Install
                  </Button>
                </Stack>
              )}
              {!checkingForUpdate && (
                <Button color="primary" onClick={() => checkForUpdate(true)} variant="text">
                  check for updates
                </Button>
              )}
              {updateError?.length > 0 && (
                <IconButton
                  edge="start"
                  aria-label="error message"
                  onClick={() => {
                    setOpenErrorTooltip(!openErrorTooltip);
                  }}
                >
                  <ErrorOutlineIcon
                    sx={{
                      fontSize: "inherit",
                      color: "red",
                    }}
                  />
                </IconButton>
              )}
            </Stack>
            {openErrorTooltip && (
              <Stack mt={1} direction="row" justifyContent="center">
                <CopyButton value={updateError} />
                <Typography variant="body" color="red">
                  {updateError}
                </Typography>
              </Stack>
            )}
          </Stack>
        )}
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          License:
        </Typography>
        <Typography variant="body">{packageJson.license}</Typography>
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          Contributors:
        </Typography>
        <Typography variant="body">
          <Stack>
            {packageJson.contributors.map((item) => (
              <Typography key={`contributor-${item}`} variant="body">
                {item}
              </Typography>
            ))}
          </Stack>
        </Typography>
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          Required additional software:
        </Typography>
        <Typography variant="body">
          <Stack>
            <Link href="https://github.com/tsl0922/ttyd" target="_blank" rel="noopener">
              https://github.com/tsl0922/ttyd
            </Link>
            <Link href="https://github.com/fkie/fkie-multi-agent-suite" target="_blank" rel="noopener">
              https://github.com/fkie/fkie-multi-agent-suite
            </Link>
          </Stack>
        </Typography>
      </Stack>
      <h3>Licenses of {Object.entries(licenses).length} dependencies</h3>
      <Stack>
        {licenses && (
          <ul>
            {Object.entries(licenses).map(([pkg, info]) => (
              <li key={pkg}>
                <strong>{pkg}</strong>: {info.licenses}
              </li>
            ))}
          </ul>
        )}
      </Stack>
    </Stack>
  );
}

export default About;
