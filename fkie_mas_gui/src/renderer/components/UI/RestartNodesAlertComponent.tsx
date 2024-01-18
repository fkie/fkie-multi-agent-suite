import { forwardRef, useCallback, useEffect, useState } from 'react';

import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Button,
  Card,
  CardActions,
  Checkbox,
  Collapse,
  IconButton,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
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
import { CrossbarIOProvider } from '../../providers';

interface RestartNodesComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  provider: CrossbarIOProvider;
  nodeList: string[];
  onReload: (providerId: string, nodeList: string[]) => void;
}

const RestartNodesAlertComponent = forwardRef<
  HTMLDivElement,
  RestartNodesComponentProps
>((props, ref) => {
  const { id, message, provider, nodeList, onReload } = props;

  const [checked, setChecked] = useState<string[]>([]);

  // set all items checked as default
  useEffect(() => {
    setChecked(nodeList);
  }, [nodeList]);

  const { closeSnackbar } = useSnackbar();
  const [expanded, setExpanded] = useState(false);

  const handleExpandClick = useCallback(() => {
    setExpanded((oldExpanded) => !oldExpanded);
  }, []);

  const handleDismiss = useCallback(() => {
    closeSnackbar(id);
  }, [id, closeSnackbar]);

  const handleToggle = (value: string) => () => {
    const currentIndex = checked.indexOf(value);
    const newChecked = [...checked];

    if (currentIndex === -1) {
      newChecked.push(value);
    } else {
      newChecked.splice(currentIndex, 1);
    }

    setChecked(newChecked);
  };

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
              sx={{
                padding: '8px 8px',
                transform: 'rotate(0deg)',
                transition: 'all .2s',
              }}
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
                handleDismiss();
                if (onReload) onReload(provider.id, checked);
              }}
            >
              Restart
            </Button>
          </Stack>
        </CardActions>
        <Collapse in={expanded} timeout="auto" unmountOnExit>
          <Paper>
            <List
              sx={{
                width: '100%',
                maxHeight: 400,
                overflow: 'auto',
                bgcolor: 'background.paper',
              }}
            >
              {nodeList &&
                nodeList.map((node) => {
                  const labelId = `checkbox-list-label-${node}`;

                  return (
                    <ListItem key={node} disablePadding>
                      <ListItemButton
                        role={undefined}
                        onClick={handleToggle(node)}
                        dense
                      >
                        <ListItemIcon>
                          <Checkbox
                            edge="start"
                            checked={checked.indexOf(node) !== -1}
                            tabIndex={-1}
                            disableRipple
                            inputProps={{ 'aria-labelledby': labelId }}
                          />
                        </ListItemIcon>
                        <ListItemText id={labelId} primary={node} />
                      </ListItemButton>
                    </ListItem>
                  );
                })}
            </List>
          </Paper>
        </Collapse>
      </Card>
    </SnackbarContent>
  );
});

export default RestartNodesAlertComponent;
