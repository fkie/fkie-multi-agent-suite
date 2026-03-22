import useInterval from "@/renderer/hooks/useInterval";
import CloseOutlinedIcon from "@mui/icons-material/CloseOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import {
  Box,
  Card,
  CardActions,
  CircularProgress,
  Collapse,
  IconButton,
  Paper,
  Stack,
  Typography,
} from "@mui/material";
import { SnackbarContent, SnackbarKey, SnackbarMessage, useSnackbar } from "notistack";
import { forwardRef, useCallback, useEffect, useState } from "react";

interface ErrorAlertComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  details: string;
}

const ErrorAlertComponent = forwardRef<HTMLDivElement, ErrorAlertComponentProps>(
  function ErrorAlertComponent(props, ref) {
    const { id, message, details } = props;

    const { closeSnackbar } = useSnackbar();
    const [expanded, setExpanded] = useState(false);
    const [progress, setProgress] = useState(100);
    const [autoDismissDisabled, setAutoDismissDisabled] = useState(false);

    const handleDismiss = useCallback((): void => {
      closeSnackbar(id);
    }, [closeSnackbar, id]);

    function handleExpandClick(): void {
      setExpanded((oldExpanded) => {
        const next = !oldExpanded;
        if (next) {
          // on expand disable auto dismiss
          setAutoDismissDisabled(true);
          setProgress(0);
        }
        return next;
      });
    }

    const updateProgress = useCallback(() => {
      if (autoDismissDisabled) {
        return;
      }
      setProgress((prevProgress) => (prevProgress < 0 ? 0 : prevProgress - 10));
    }, [autoDismissDisabled]);

    useInterval(updateProgress, 1000);

    useEffect(() => {
      if (!autoDismissDisabled && progress <= 0) handleDismiss();
    }, [progress, autoDismissDisabled]);

    return (
      <SnackbarContent ref={ref}>
        <Card
          sx={{
            color: (theme) => theme.palette.getContrastText(theme.palette.error.main),
            backgroundColor: (theme) => theme.palette.error.main,
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
              </Stack>

              {!autoDismissDisabled ? (
                <Box sx={{ position: "relative", display: "inline-flex" }}>
                  <CircularProgress
                    variant="determinate"
                    value={progress}
                    size={32}
                    thickness={4}
                    sx={{
                      color: (theme) => theme.palette.getContrastText(theme.palette.error.main),
                    }}
                  />
                  <Box
                    sx={{
                      top: 0,
                      left: 0,
                      bottom: 0,
                      right: 0,
                      position: "absolute",
                      display: "flex",
                      alignItems: "center",
                      justifyContent: "center",
                    }}
                  >
                    <IconButton
                      onClick={handleDismiss}
                      size="small"
                      sx={{
                        color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
                      }}
                    >
                      <CloseOutlinedIcon fontSize="inherit" />
                    </IconButton>
                  </Box>
                </Box>
              ) : (
                <IconButton
                  onClick={handleDismiss}
                  size="small"
                  sx={{
                    color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
                  }}
                >
                  <CloseOutlinedIcon fontSize="inherit" />
                </IconButton>
              )}
            </Stack>
          </CardActions>
          <Collapse in={expanded} timeout="auto" unmountOnExit>
            <Paper sx={{ padding: 2 }}>
              <Typography variant="body1">{details}</Typography>
            </Paper>
          </Collapse>
        </Card>
      </SnackbarContent>
    );
  }
);

export default ErrorAlertComponent;
