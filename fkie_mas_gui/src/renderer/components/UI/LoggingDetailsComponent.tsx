import CloseOutlinedIcon from "@mui/icons-material/CloseOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { Box, Card, CardActions, Collapse, IconButton, Paper, Stack, Typography } from "@mui/material";
import { SnackbarContent, SnackbarKey, SnackbarMessage, VariantType, useSnackbar } from "notistack";
import React, { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { JSONTree } from "react-json-tree";
import { SettingsContext } from "../../context/SettingsContext";
import { darkThemeJson } from "../../themes/darkTheme";
import { lightThemeJson } from "../../themes/lightTheme";
import { levelColorsWbg } from "./Colors";

interface LoggingDetailsComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  details: string | object | undefined;
  variant: VariantType;
  onDetailsClick: (id: SnackbarKey | undefined, event: React.MouseEvent) => void;
}

const LoggingDetailsComponent = forwardRef<HTMLDivElement, LoggingDetailsComponentProps>((props, ref) => {
  const settingsCtx = useContext(SettingsContext);
  const { id, message, details, variant, onDetailsClick } = props;

  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);
  const [detailsObject, setDetailsObject] = useState(details);

  useEffect(() => {
    try {
      if (typeof details === "string") {
        const obj = JSON.parse(details);
        setDetailsObject(obj);
      }
    } catch {
      //
    }
  }, [details]);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleDismiss = useCallback(() => {
    closeSnackbar(id);
  }, [id, closeSnackbar]);

  const generateInfo = useMemo(() => {
    return (
      <Stack direction="row" spacing="0.5em" alignItems="center">
        <Box sx={{ flexGrow: 1 }} />
        {details && (
          <IconButton
            aria-label="Show more"
            sx={{
              color: (theme) => theme.palette.getContrastText(levelColorsWbg[variant].backgroundColor),
              transform: "rotate(0deg)",
              transition: "all .2s",
            }}
            style={expanded ? { transform: "rotate(180deg)" } : undefined}
            onClick={handleExpandClick}
          >
            <ExpandMoreIcon />
          </IconButton>
        )}
        <Box
          sx={{
            "&:hover": {
              cursor: "pointer",
            },
          }}
        >
          <Typography
            onClick={(event) => {
              if (onDetailsClick) onDetailsClick(id, event);
            }}
            variant="subtitle1"
          >
            {message}
          </Typography>
        </Box>
        <IconButton
          onClick={handleDismiss}
          size="small"
          sx={{ color: (theme) => theme.palette.getContrastText(levelColorsWbg[variant].backgroundColor) }}
        >
          <CloseOutlinedIcon fontSize="inherit" />
        </IconButton>
      </Stack>
    );
  }, [expanded, variant, message, id, details]);

  return (
    <SnackbarContent ref={ref}>
      <Card
        sx={{
          marginBottom: 3,
          flexGrow: 1,
          color: (theme) => theme.palette.getContrastText(levelColorsWbg[variant].backgroundColor),
          backgroundColor: levelColorsWbg[variant].backgroundColor,
        }}
      >
        <CardActions
          sx={{
            justifyContent: "right",
          }}
        >
          {generateInfo}
        </CardActions>
        {details && (
          <Collapse in={expanded} timeout={expanded ? undefined : "auto"} unmountOnExit>
            <Paper sx={{ padding: 2 }}>
              {!(typeof detailsObject === "string" || detailsObject instanceof String) && (
                <JSONTree
                  data={detailsObject}
                  sortObjectKeys={true}
                  theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
                  invertTheme={false}
                  hideRoot={true}
                  shouldExpandNodeInitially={() => {
                    return false;
                  }}
                />
              )}
              {(typeof detailsObject === "string" || detailsObject instanceof String) && (
                <Typography overflow="auto" noWrap={false} maxHeight="10em" fontSize="0.9em">
                  {JSON.stringify(details)}
                </Typography>
              )}
            </Paper>
          </Collapse>
        )}
      </Card>
    </SnackbarContent>
  );
});

LoggingDetailsComponent.displayName = "LoggingDetailsComponent";

export default LoggingDetailsComponent;
