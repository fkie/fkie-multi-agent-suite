import ErrorOutlineIcon from "@mui/icons-material/ErrorOutline";
import { Autocomplete, Box, Button, IconButton, Link, Stack, TextField, Typography } from "@mui/material";
import CircularProgress from "@mui/material/CircularProgress";
import LinearProgress from "@mui/material/LinearProgress";
import { MuiMarkdown } from "mui-markdown";
import PropTypes from "prop-types";
import { useContext, useState } from "react";

import { AutoUpdateContext } from "@/renderer//context/AutoUpdateContext";
import licenses from "@/renderer/deps-licenses.json";
import packageJson from "../../../../package.json";
import CopyButton from "../UI/CopyButton";

function LinearProgressWithLabel({ value, ...props }): JSX.Element {
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

export default function About(): JSX.Element {
  const auCtx = useContext(AutoUpdateContext);
  const [openErrorTooltip, setOpenErrorTooltip] = useState(!!auCtx.updateError);

  const updateCli = auCtx.getUpdateCli(true, true);
  const updateCliRobot = auCtx.getUpdateCli(false, true);

  return (
    <Stack height="100%" padding="0.3em" overflow="auto">
      {/** Version */}
      <Stack direction="column" justifyItems="center">
        <Stack spacing={1} direction="row" justifyItems="center" alignItems="center">
          <Typography variant="body1" sx={{ fontWeight: "bold" }}>
            Version:
          </Typography>
          <Typography variant="body1">{packageJson.version}</Typography>
          {auCtx.autoUpdateManager && !auCtx.checkingForUpdate && (
            <Button color="primary" onClick={() => auCtx.checkForUpdate()} variant="text">
              check for updates
            </Button>
          )}
          {auCtx.updateError.length > 0 && (
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
          {auCtx.checkTimestamp > 0 && (
            <Typography variant="body1" color="grey" sx={{ fontSize: "0.8em" }}>
              last check: {new Date(auCtx.checkTimestamp * 1000).toLocaleString()}
            </Typography>
          )}
        </Stack>
        {auCtx.autoUpdateManager && (
          <Stack ml="1em" direction="column">
            {auCtx.checkedThisRun && !auCtx.checkingForUpdate && !auCtx.updateAvailable && !auCtx.updateError && (
              <Typography variant="body1" color="green">
                Your version is up to date!
              </Typography>
            )}
          </Stack>
        )}
        {auCtx.autoUpdateManager && (
          <Stack ml="1em" spacing={0.2} direction="row" alignItems="center">
            <Autocomplete
              handleHomeEndKeys={false}
              disablePortal
              disableClearable
              freeSolo
              // disableCloseOnSelect
              // multiple
              id="auto-complete-au-channel"
              size="small"
              options={["prerelease", "release"]}
              sx={{ margin: 0, width: "9em" }}
              renderInput={(params) => <TextField {...params} label="Update channel" />}
              value={auCtx.updateChannel}
              onChange={(_event: unknown, newValue: string | null) => {
                if (newValue && (["prerelease", "release"].includes(newValue) || newValue.split(".").length === 3)) {
                  auCtx.setUpdateChannel(newValue);
                }
              }}
            />
            {auCtx.checkingForUpdate && (
              <Box sx={{ display: "flex" }}>
                <CircularProgress size="1em" />
              </Box>
            )}
            {auCtx.downloadProgress && (
              <Stack spacing={0.2} direction="row">
                <Typography variant="body1">downloading {auCtx?.updateAvailable?.version}</Typography>
                {auCtx.downloadProgress.percent < 100 && (
                  <Box sx={{ width: "100%" }}>
                    <LinearProgressWithLabel value={auCtx.downloadProgress.percent} />
                  </Box>
                )}
              </Stack>
            )}

            {auCtx?.updateAvailable?.version && (
              <Stack spacing={0.2} direction="row" alignItems="center">
                <Typography variant="body1" color="green">
                  Version {auCtx.updateAvailable?.version} available
                </Typography>
                {auCtx.isAppImage ? (
                  <Button color="primary" onClick={() => auCtx.requestInstallUpdate()} variant="text">
                    Restart required
                  </Button>
                ) : auCtx?.installing ? (
                  <CircularProgress style={{ marginLeft: "0.5em" }} size="1em" />
                ) : (
                  <Button
                    color="primary"
                    onClick={async () => {
                      auCtx.installDebian(true, true);
                    }}
                    variant="text"
                  >
                    Update
                  </Button>
                )}
              </Stack>
            )}
          </Stack>
        )}
        {auCtx.autoUpdateManager && auCtx.isAppImage && auCtx.updateChannel === "prerelease" && (
          <Typography ml="1em" variant="body1" color="orange">
            You must switch to the 'prerelease' branch for Daemon and Discovery
          </Typography>
        )}
        {openErrorTooltip && auCtx.isAppImage && (
          <Stack ml="1em" direction="row" alignItems="center">
            <CopyButton value={auCtx.updateError} />
            <Typography variant="body1" color="red">
              {auCtx.updateError}
            </Typography>
          </Stack>
        )}
        {openErrorTooltip && !auCtx.isAppImage && (
          <Stack ml="1em" direction="column" justifyItems="left">
            <Typography variant="body1" color="red">
              {auCtx.updateError}
            </Typography>
            <Stack mb="0.5em" mt="0.5em" direction="column">
              <Typography variant="body1" color="green">
                You can try to change the default terminal using the following command and run the update again:
              </Typography>
              <Stack ml="1em" direction="row" alignItems="center">
                <Typography variant="body1" color="grey">
                  sudo update-alternatives --config x-terminal-emulator
                </Typography>
                <CopyButton value={"sudo update-alternatives --config x-terminal-emulator"} />
              </Stack>
              <Typography variant="body1" color="green">
                Or try to start the local TTYD.
              </Typography>{" "}
            </Stack>
          </Stack>
        )}
        {!auCtx.isAppImage && (
          <Stack ml="1em" direction="column" justifyItems="left">
            <Typography variant="body1">Manual update in the terminal of your choice:</Typography>
            <Stack ml="1em" direction="row" alignItems="center">
              <CopyButton value={updateCli} />
              <Typography variant="body1" color="grey">
                {updateCli}
              </Typography>
            </Stack>
            <Typography ml="0.5em" variant="body2">
              without mas gui:
            </Typography>
            <Stack ml="1em" direction="row" alignItems="center">
              <CopyButton value={updateCli} />
              <Typography variant="body1" color="grey">
                {updateCliRobot}
              </Typography>
            </Stack>
          </Stack>
        )}
        {auCtx?.updateAvailable?.releaseNotes && (
          <Stack ml="1em" mt="0.6em" spacing={0.2} color="grey" direction="column">
            <MuiMarkdown>{auCtx?.updateAvailable?.releaseNotes as string}</MuiMarkdown>
          </Stack>
        )}
      </Stack>
      {/** License */}
      <Stack mt="0.6em" spacing={1} direction="row">
        <Typography variant="body1" sx={{ fontWeight: "bold" }}>
          License:
        </Typography>
        <Typography variant="body1">{packageJson.license}</Typography>
      </Stack>
      {/** Contributors */}
      <Stack mt="0.6em" spacing="0.2em" direction="column">
        <Typography variant="body1" sx={{ fontWeight: "bold" }}>
          Contributors:
        </Typography>
        <Stack paddingLeft="1em">
          {packageJson.contributors.map((item) => (
            <Typography key={`contributor-${item}`} variant="body1">
              {item}
            </Typography>
          ))}
        </Stack>
      </Stack>
      {/** additional software */}
      <Stack mt="0.6em" spacing="0.2em" direction="column">
        <Typography variant="body1" sx={{ fontWeight: "bold" }}>
          Required additional software:
        </Typography>
        <Stack paddingLeft="1em">
          <Link href="https://github.com/tsl0922/ttyd" target="_blank" rel="noopener">
            https://github.com/tsl0922/ttyd
          </Link>
          <Link href="https://github.com/fkie/fkie-multi-agent-suite" target="_blank" rel="noopener">
            https://github.com/fkie/fkie-multi-agent-suite
          </Link>
        </Stack>
      </Stack>
      {/** dependencies */}
      <Typography variant="body1" mt="0.6em" sx={{ fontWeight: "bold" }}>
        List of {Object.entries(licenses).length} dependencies:
      </Typography>
      <Stack>
        {licenses && (
          <ul>
            {licenses.map((item) => (
              <li key={item.name}>
                {item.name}@{item.installedVersion}: {item.licenseType}
              </li>
            ))}
          </ul>
        )}
      </Stack>
    </Stack>
  );
}
