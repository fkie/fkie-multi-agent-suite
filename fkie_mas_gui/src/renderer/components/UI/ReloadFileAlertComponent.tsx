import {
  forwardRef,
  useCallback,
  useContext,
  useEffect,
  useState,
} from 'react';

import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
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
} from '@mui/material';
import {
  SnackbarContent,
  SnackbarKey,
  SnackbarMessage,
  useSnackbar,
} from 'notistack';
import { SettingsContext } from '../../context/SettingsContext';
import { PATH_EVENT_TYPE } from '../../models';
import CrossbarIOProvider from '../../providers/crossbar_io/CrossbarIOProvider';

interface ReloadFileComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  provider: CrossbarIOProvider;
  modifiedFile: string;
  modification: PATH_EVENT_TYPE;
  launchFile: string;
  onReload: (providerId: string, launchFile: string) => void;
}

const ReloadFileAlertComponent = forwardRef<
  HTMLDivElement,
  ReloadFileComponentProps
>((props, ref) => {
  const {
    id,
    message,
    provider,
    modifiedFile,
    modification,
    launchFile,
    onReload,
  } = props;

  const settingsCtx = useContext(SettingsContext);
  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);
  const [rememberChange, setRememberChange] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleReload = useCallback(() => {
    if (rememberChange) {
      settingsCtx.set('actionOnChangeLaunch', 'RELOAD');
    }
    if (onReload) onReload(provider.id, launchFile);
    closeSnackbar(id);
  }, [
    rememberChange,
    onReload,
    provider.id,
    launchFile,
    closeSnackbar,
    id,
    settingsCtx,
  ]);

  const handleDismiss = useCallback(() => {
    if (rememberChange) {
      settingsCtx.set('actionOnChangeLaunch', 'DISMISS');
    }
    closeSnackbar(id);
  }, [rememberChange, closeSnackbar, id, settingsCtx]);

  useEffect(() => {
    switch (settingsCtx.get('actionOnChangeLaunch')) {
      case 'RELOAD':
        handleReload();
        break;
      case 'DISMISS':
        handleDismiss();
        break;
      default:
        break;
    }
  }, [handleDismiss, handleReload, settingsCtx]);

  return (
    <SnackbarContent ref={ref}>
      <Card sx={{ marginTop: 4, backgroundColor: '#fddc6c' }}>
        <CardActions>
          <Stack
            sx={{ width: '100%' }}
            direction="row"
            spacing={0.5}
            justifyContent="space-around"
            alignItems="center"
          >
            <IconButton
              aria-label="Show more"
              style={expanded ? { transform: 'rotate(180deg)' } : undefined}
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
                    sx={{ '& .MuiSvgIcon-root': { fontSize: 'inherit' } }}
                  />
                }
                label="remember the decision"
              />
            </Stack>

            <Button size="small" onClick={handleDismiss}>
              Dismiss
            </Button>

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

export default ReloadFileAlertComponent;
