import CloseOutlinedIcon from "@mui/icons-material/CloseOutlined";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import {
  Box,
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
} from "@mui/material";
import { SnackbarContent, SnackbarKey, SnackbarMessage, useSnackbar } from "notistack";
import { forwardRef, useEffect, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import Provider from "@/renderer/providers/Provider";
import { EVENT_PROVIDER_NODE_BINARY_MODIFIED, EVENT_PROVIDER_NODE_STARTED } from "@/renderer/providers/eventTypes";
import { EventProviderNodeBinaryModified, EventProviderNodeStarted } from "@/renderer/providers/events";

interface RestartNodesComponentProps {
  id: SnackbarKey | undefined;
  message: SnackbarMessage;
  provider: Provider;
  nodeList: string[];
  onReload: (providerId: string, nodeList: string[]) => void;
}

const RestartNodesAlertComponent = forwardRef<HTMLDivElement, RestartNodesComponentProps>(
  function RestartNodesAlertComponent(props, ref) {
    const { id, message, provider, nodeList, onReload } = props;

    const [checked, setChecked] = useState<string[]>([]);
    const [currentNodeList, setCurrentNodeList] = useState<string[]>(nodeList);

    // set all items checked as default
    useEffect(() => {
      setChecked(nodeList);
    }, [nodeList]);

    const { closeSnackbar } = useSnackbar();
    const [expanded, setExpanded] = useState(false);

    function handleExpandClick(): void {
      setExpanded((oldExpanded) => !oldExpanded);
    }

    function handleDismiss(): void {
      closeSnackbar(id);
    }

    function handleToggle(value: string): void {
      const currentIndex = checked.indexOf(value);
      const newChecked = [...checked];

      if (currentIndex === -1) {
        newChecked.push(value);
      } else {
        newChecked.splice(currentIndex, 1);
      }

      setChecked(newChecked);
    }

    // remove nodes started from this list to close this alert if the list is empty
    useCustomEventListener(
      EVENT_PROVIDER_NODE_STARTED,
      (data: EventProviderNodeStarted) => {
        if (data.provider.id === provider.id) {
          // remote node from checked and currentNodeList
          setChecked((prev) => prev.filter((value) => value !== data.node.name));
          const newNodeList = currentNodeList.filter((value) => value !== data.node.name);
          if (newNodeList.length > 0) {
            setCurrentNodeList(currentNodeList.filter((value) => value !== data.node.name));
          } else {
            // close this alert if all nodes are started on another way, e.g. restart button.
            handleDismiss();
          }
        }
      },
      [checked, currentNodeList]
    );

    // remove nodes started from this list to close this alert if the list is empty
    useCustomEventListener(
      EVENT_PROVIDER_NODE_BINARY_MODIFIED,
      (data: EventProviderNodeBinaryModified) => {
        if (data.provider.id === provider.id) {
          setChecked((prev) => [...prev, ...data.nodeNames.filter((name) => !prev.includes(name))]);
          setCurrentNodeList((prev) => [...prev, ...data.nodeNames.filter((name) => !prev.includes(name))]);
        }
      },
      [checked, currentNodeList]
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
            <Stack sx={{ width: "100%" }} direction="row" spacing="0.5em" alignItems="center">
              <Box sx={{ flexGrow: 1 }} />
              <IconButton
                aria-label="Show more"
                sx={{
                  color: (theme) => theme.palette.getContrastText(theme.palette.warning.main),
                  transform: "rotate(0deg)",
                  transition: "all .2s",
                }}
                style={expanded ? { transform: "rotate(180deg)" } : undefined}
                onClick={() => handleExpandClick()}
              >
                <ExpandMoreIcon fontSize="inherit" />
              </IconButton>

              <Typography variant="subtitle1">{`${message} Do you want to restart ${currentNodeList.length} nodes on: `}</Typography>
              <Typography variant="subtitle1" fontWeight="bold">
                {provider.name()}
              </Typography>

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
              <IconButton
                onClick={() => handleDismiss()}
                size="small"
                sx={{ color: (theme) => theme.palette.getContrastText(theme.palette.warning.main) }}
              >
                <CloseOutlinedIcon fontSize="inherit" />
              </IconButton>
            </Stack>
          </CardActions>
          <Collapse in={expanded} timeout="auto" unmountOnExit>
            <Paper>
              <List
                sx={{
                  width: "100%",
                  maxHeight: 400,
                  overflow: "auto",
                  padding: 0,
                  margin: 0,
                }}
              >
                {currentNodeList?.map((node) => {
                  const labelId = `checkbox-list-label-${node}`;
                  return (
                    <ListItem key={node} disablePadding>
                      <ListItemButton onClick={() => handleToggle(node)} dense>
                        <ListItemIcon>
                          <Checkbox
                            edge="start"
                            checked={checked.indexOf(node) !== -1}
                            tabIndex={-1}
                            disableRipple
                            inputProps={{ "aria-labelledby": labelId }}
                            sx={{ padding: 0 }}
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
  }
);

export default RestartNodesAlertComponent;
