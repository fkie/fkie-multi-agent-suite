import { forwardRef, useCallback, useState } from 'react';

import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Button,
  Card,
  CardActions,
  Collapse,
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
import { PATH_EVENT_TYPE } from '../../models';
import { CrossbarIOProvider } from '../../providers';

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

  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleDismiss = useCallback(() => {
    closeSnackbar(id);
  }, [id, closeSnackbar]);

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

            <Typography variant="body2">{message}</Typography>
            <Typography variant="subtitle2">{provider.name()}</Typography>

            <Button size="small" onClick={handleDismiss}>
              Dismiss
            </Button>

            <Button
              size="small"
              color="success"
              variant="contained"
              onClick={() => {
                if (onReload) onReload(provider.id, launchFile);

                handleDismiss();
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
          </Paper>
        </Collapse>
      </Card>
    </SnackbarContent>
  );
});

export default ReloadFileAlertComponent;
