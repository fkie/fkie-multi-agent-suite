import { forwardRef, useCallback, useContext, useState } from 'react';

import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Button,
  Card,
  CardActions,
  Collapse,
  IconButton,
  Stack,
  Typography,
} from '@mui/material';
import {
  SnackbarContent,
  SnackbarKey,
  SnackbarMessage,
  VariantType,
  useSnackbar,
} from 'notistack';
import ReactJson from 'react-json-view';
import { levelColorsWbg } from '../components/UI/Colors';
import { SettingsContext } from './SettingsContext';

interface LoggingDetailsComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  details: string | Object;
  variant: VariantType;
}

const LoggingDetailsComponent = forwardRef<
  HTMLDivElement,
  LoggingDetailsComponentProps
>((props, ref) => {
  const settingsCtx = useContext(SettingsContext);
  const { id, message, details, variant } = props;

  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleDismiss = useCallback(() => {
    closeSnackbar(id);
  }, [id, closeSnackbar]);

  return (
    <SnackbarContent ref={ref} style={{ maxHeight: '50%' }}>
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
              style={expanded ? { transform: 'rotate(180deg)' } : undefined}
              onClick={handleExpandClick}
            >
              <ExpandMoreIcon />
            </IconButton>
            <Typography variant="body2">{message}</Typography>
            <Button size="small" onClick={handleDismiss}>
              Close
            </Button>
          </Stack>
        </CardActions>
        <Collapse in={expanded} unmountOnExit>
          {!(typeof details === 'string' || details instanceof String) && (
            <ReactJson
              name={false}
              collapsed={1}
              theme={
                settingsCtx.get('useDarkMode') ? 'grayscale' : 'rjv-default'
              }
              src={details}
              collapseStringsAfterLength={200}
              displayObjectSize={false}
              enableClipboard
              indentWidth={2}
              displayDataTypes={false}
              iconStyle="triangle"
              quotesOnKeys={false}
              sortKeys
            />
          )}
          {(typeof details === 'string' || details instanceof String) && (
            <Typography
              overflow="auto"
              noWrap={false}
              maxHeight="10em"
              fontSize="0.9em"
            >
              {JSON.stringify(details)}
            </Typography>
          )}
        </Collapse>
      </Card>
    </SnackbarContent>
  );
});

export default LoggingDetailsComponent;
