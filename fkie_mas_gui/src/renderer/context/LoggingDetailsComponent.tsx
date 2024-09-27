import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { Box, Button, Card, CardActions, Collapse, IconButton, Stack, Typography } from "@mui/material";
import { SnackbarContent, SnackbarKey, SnackbarMessage, VariantType, useSnackbar } from "notistack";
import { forwardRef, useCallback, useContext, useState } from "react";
import { JSONTree } from "react-json-tree";
import { levelColorsWbg } from "../components/UI/Colors";
import { darkThemeJson } from "../themes/darkTheme";
import { lightThemeJson } from "../themes/lightTheme";
import { SettingsContext } from "./SettingsContext";

interface LoggingDetailsComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  details: string | object;
  variant: VariantType;
  onDetailsClick: Function;
}

const LoggingDetailsComponent = forwardRef<HTMLDivElement, LoggingDetailsComponentProps>((props, ref) => {
  const settingsCtx = useContext(SettingsContext);
  const { id, message, details, variant, onDetailsClick } = props;

  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleDismiss = useCallback(() => {
    closeSnackbar(id);
  }, [id, closeSnackbar]);

  return (
    <SnackbarContent ref={ref} style={{ maxHeight: "50%" }}>
      <Card>
        <CardActions sx={levelColorsWbg[variant]}>
          <Stack
            direction="row"
            spacing={0.5}
            // justifyContent="space-around"
            alignItems="center"
          >
            <IconButton
              aria-label="Show more"
              style={expanded ? { transform: "rotate(180deg)" } : undefined}
              onClick={handleExpandClick}
            >
              <ExpandMoreIcon />
            </IconButton>
            <Box
              sx={{
                "&:hover": {
                  cursor: "zoom-in",
                },
              }}
            >
              <Typography
                onClick={(event) => {
                  if (onDetailsClick) onDetailsClick(id, event);
                }}
                variant="body2"
              >
                {message}
              </Typography>
            </Box>
            <Button size="small" onClick={handleDismiss}>
              Close
            </Button>
          </Stack>
        </CardActions>
        <Collapse in={expanded} unmountOnExit>
          {!(typeof details === "string" || details instanceof String) && (
            <JSONTree
              data={details}
              sortObjectKeys={true}
              theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
              invertTheme={false}
              hideRoot={true}
              shouldExpandNodeInitially={() => {
                return false;
              }}
            />
          )}
          {(typeof details === "string" || details instanceof String) && (
            <Typography overflow="auto" noWrap={false} maxHeight="10em" fontSize="0.9em">
              {JSON.stringify(details)}
            </Typography>
          )}
        </Collapse>
      </Card>
    </SnackbarContent>
  );
});

export default LoggingDetailsComponent;
