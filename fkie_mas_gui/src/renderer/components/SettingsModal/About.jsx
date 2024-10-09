import licenses from "@/renderer/deps-licenses.json";
import ErrorOutlineIcon from "@mui/icons-material/ErrorOutline";
import { Box, Button, IconButton, Link, Stack, Typography } from "@mui/material";
import CircularProgress from "@mui/material/CircularProgress";
import LinearProgress from "@mui/material/LinearProgress";
import PropTypes from "prop-types";
import { useContext, useState } from "react";
import packageJson from "../../../../package.json";
import { AutoUpdateContext } from "../../context/AutoUpdateContext";
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
  const auCtx = useContext(AutoUpdateContext);
  const [openErrorTooltip, setOpenErrorTooltip] = useState(false);

  return (
    <Stack height="100%" padding="0.3em">
      {/** Version */}
      <Stack direction="column" justifyItems="center">
        <Stack spacing={1} direction="row" justifyItems="center" alignItems="center">
          <Typography variant="body" sx={{ fontWeight: "bold" }}>
            Version:
          </Typography>
          <Typography variant="body">{packageJson.version}</Typography>
          {!auCtx.checkingForUpdate && (
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
        </Stack>
        {auCtx.autoUpdateManager && (
          <Stack ml="1em" direction="column">
            {!auCtx.checkingForUpdate && !auCtx.updateAvailable && !auCtx.updateError && (
              <Typography variant="body" color="green">
                Your version is up to date!
              </Typography>
            )}
          </Stack>
        )}
        {auCtx.autoUpdateManager && (
          <Stack ml="1em" spacing={0.2} direction="row" alignItems="center">
            {auCtx.checkingForUpdate && (
              <Box sx={{ display: "flex" }}>
                <CircularProgress size="1em" />
              </Box>
            )}
            {auCtx.downloadProgress && (
              <Stack spacing={0.2} direction="row">
                <Typography variant="body">downloading {auCtx.updateAvailable.version}</Typography>
                {auCtx.downloadProgress.percent < 100 && (
                  <Box sx={{ width: "100%" }}>
                    <LinearProgressWithLabel value={auCtx.downloadProgress.percent} />
                  </Box>
                )}
              </Stack>
            )}

            {auCtx.updateAvailable?.downloadedFile && (
              <Stack spacing={0.2} direction="row" alignItems="center">
                <Typography variant="body" color="green">
                  Version {auCtx.updateAvailable?.version} downloaded
                </Typography>
                <Button color="primary" onClick={() => auCtx.requestInstallUpdate()} variant="text">
                  Restart and Install
                </Button>
              </Stack>
            )}
          </Stack>
        )}
        {openErrorTooltip && (
          <Stack ml="1em" direction="row" alignItems="center">
            <CopyButton value={auCtx.updateError} />
            <Typography variant="body" color="red">
              {auCtx.updateError}
            </Typography>
          </Stack>
        )}
      </Stack>
      {/** License */}
      <Stack mt="0.6em" spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          License:
        </Typography>
        <Typography variant="body">{packageJson.license}</Typography>
      </Stack>
      {/** Contributors */}
      <Stack mt="0.6em" spacing="0.2em" direction="column">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          Contributors:
        </Typography>
        <Typography variant="body" paddingLeft="1em">
          <Stack>
            {packageJson.contributors.map((item) => (
              <Typography key={`contributor-${item}`} variant="body">
                {item}
              </Typography>
            ))}
          </Stack>
        </Typography>
      </Stack>
      {/** additional software */}
      <Stack mt="0.6em" spacing="0.2em" direction="column">
        <Typography variant="body" sx={{ fontWeight: "bold" }}>
          Required additional software:
        </Typography>
        <Typography variant="body" paddingLeft="1em">
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
      {/** dependencies */}
      <Typography variant="body" mt="0.6em" sx={{ fontWeight: "bold" }}>
        List of {Object.entries(licenses).length} dependencies:
      </Typography>
      <Stack overflow="auto">
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

export default About;
