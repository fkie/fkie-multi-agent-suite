import CloseOutlinedIcon from "@mui/icons-material/CloseOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import {
  Box,
  Button,
  Card,
  CardActions,
  Checkbox,
  Collapse,
  FormControlLabel,
  IconButton,
  Paper,
  Stack,
  Typography,
} from "@mui/material";
import { SnackbarContent, SnackbarKey, SnackbarMessage, useSnackbar } from "notistack";
import { forwardRef, useCallback, useContext, useEffect, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { SettingsContext } from "../../context/SettingsContext";
import { PATH_EVENT_TYPE } from "../../models";
import Provider from "../../providers/Provider";
import { EVENT_PROVIDER_LAUNCH_LOADED } from "../../providers/eventTypes";
import { EventProviderLaunchLoaded } from "../../providers/events";

interface ReloadFileComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  provider: Provider;
  modifiedFile: string;
  modification: PATH_EVENT_TYPE;
  launchFile: string;
  onReload: (providerId: string, launchFile: string) => void;
}

const ReloadFileAlertComponent = forwardRef<HTMLDivElement, ReloadFileComponentProps>((props, ref) => {
  const { id, message, provider, modifiedFile, modification, launchFile, onReload } = props;

  const settingsCtx = useContext(SettingsContext);
  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);
  const [rememberChange, setRememberChange] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleReload = useCallback(() => {
    if (rememberChange) {
      settingsCtx.set("actionOnChangeLaunch", "RELOAD");
    }
    if (onReload) onReload(provider.id, launchFile);
    closeSnackbar(id);
  }, [rememberChange, onReload, provider.id, launchFile, closeSnackbar, id, settingsCtx]);

  const handleDismiss = useCallback(() => {
    if (rememberChange) {
      settingsCtx.set("actionOnChangeLaunch", "DISMISS");
    }
    closeSnackbar(id);
  }, [rememberChange, closeSnackbar, id, settingsCtx]);

  useEffect(() => {
    switch (settingsCtx.get("actionOnChangeLaunch")) {
      case "RELOAD":
        if (provider.className === "Provider") {
          handleReload();
        } else {
          handleDismiss();
        }
        break;
      case "DISMISS":
        handleDismiss();
        break;
      default:
        break;
    }
  }, [handleDismiss, handleReload, settingsCtx]);

  // close this alert if launch file was loaded
  useCustomEventListener(
    EVENT_PROVIDER_LAUNCH_LOADED,
    (data: EventProviderLaunchLoaded) => {
      if (data.provider.id === provider.id) {
        if (data.launchFile === launchFile) {
          handleDismiss();
        }
      }
    },
    [launchFile, provider]
  );

  return (
    <SnackbarContent ref={ref}>
      <Card
        sx={{
          // marginTop: 7,
          color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
          backgroundColor: (theme) => theme.palette.warning.main,
        }}
      >
        <CardActions>
          <Stack
            sx={{ width: "100%" }}
            direction="row"
            spacing="0.5em"
            justifyContent="space-around"
            alignItems="center"
          >
            <Box sx={{ flexGrow: 1 }} />
            <IconButton
              aria-label="Show more"
              sx={{
                color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
                transform: "rotate(0deg)",
                transition: "all .2s",
              }}
              style={expanded ? { transform: "rotate(180deg)" } : undefined}
              onClick={handleExpandClick}
            >
              <ExpandMoreIcon />
            </IconButton>
            <Stack direction="column">
              <Stack direction="row" spacing="1em">
                <Typography variant="subtitle1">{message}</Typography>
              </Stack>
              <FormControlLabel
                control={
                  <Checkbox
                    onChange={(event) => {
                      setRememberChange(event.target.checked);
                    }}
                    sx={{
                      "& .MuiSvgIcon-root": {
                        fontSize: "inherit",
                        color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
                      },
                    }}
                  />
                }
                label="remember the decision"
              />
            </Stack>
            <Button
              size="small"
              color="success"
              variant="contained"
              onClick={() => {
                handleReload();
              }}
            >
              Reload
            </Button>
            <IconButton
              onClick={handleDismiss}
              size="small"
              sx={{ color: (theme) => theme.palette.getContrastText(theme.palette.warning.main) }}
            >
              <CloseOutlinedIcon fontSize="inherit" />
            </IconButton>
          </Stack>
        </CardActions>
        <Collapse in={expanded} timeout="auto" unmountOnExit>
          <Paper sx={{ padding: 2 }}>
            <Typography variant="body1">
              {modification}: {modifiedFile}
            </Typography>
            <Typography variant="body1">provider: {provider.name()}</Typography>
          </Paper>
        </Collapse>
      </Card>
    </SnackbarContent>
  );
});

ReloadFileAlertComponent.displayName = "ReloadFileAlertComponent";

export default ReloadFileAlertComponent;
