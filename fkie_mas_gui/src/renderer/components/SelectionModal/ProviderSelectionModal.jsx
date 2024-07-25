import {
  Button,
  Checkbox,
  CircularProgress,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Typography,
} from "@mui/material";
import LinearProgress from "@mui/material/LinearProgress";
import PropTypes from "prop-types";
import { useCallback, useEffect, useRef, useState } from "react";
import { delay } from "../../utils";
import DraggablePaper from "../UI/DraggablePaper";

function ListSelectionModal({
  title = "Select providers",
  providers,
  onCloseCallback,
  onConfirmCallback,
  onForceCloseCallback,
}) {
  const [selectedProviders, setSelectedProviders] = useState([]);
  const [progress, setProgress] = useState(100);
  const [showProgress, setShowProgress] = useState(true);
  const closeInterval = useRef(null);
  const [onShutdown, setOnShutdown] = useState(false);

  const handleToggle = (value) => () => {
    const currentIndex = selectedProviders.findIndex((prov) => prov.id === value);
    const newSelectedProviders = [...selectedProviders];

    if (currentIndex === -1) {
      newSelectedProviders.push(providers.find((provider) => provider.id === value));
    } else {
      newSelectedProviders.splice(currentIndex, 1);
    }

    setSelectedProviders(newSelectedProviders);
    setShowProgress(false);
    if (closeInterval.current) {
      clearInterval(closeInterval.current);
      closeInterval.current = null;
    }
  };

  const handleClose = (event, reason) => {
    if (reason && reason === "backdropClick") return;
    if (closeInterval.current) {
      clearInterval(closeInterval.current);
      closeInterval.current = null;
    }
    onCloseCallback();
  };

  const onConfirm = () => {
    setOnShutdown(true);
    onConfirmCallback(selectedProviders);
    // handleClose();
  };

  const performCloseProgress = useCallback(
    async (oldProgress) => {
      if (!showProgress) return;
      if (oldProgress < 0) {
        onConfirmCallback([]);
      } else {
        await delay(1000);
        setProgress(oldProgress - 20);
      }
    },
    [onConfirmCallback, showProgress]
  );

  useEffect(() => {
    if (progress === 100) {
      setProgress(progress - 20);
    } else {
      performCloseProgress(progress);
    }
  }, [performCloseProgress, progress]);

  const dialogRef = useRef(null);

  return (
    <Dialog
      open
      onClose={handleClose}
      fullWidth
      maxWidth="md"
      ref={dialogRef}
      PaperProps={{
        component: DraggablePaper,
        dialogRef: dialogRef,
      }}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="handle" style={{ cursor: "move" }} id="draggable-dialog-title">
        {title}
      </DialogTitle>

      <DialogContent scroll="paper" aria-label="list">
        <List
          sx={{
            width: "100%",
            maxHeight: 400,
            overflow: "auto",
            bgcolor: "background.paper",
          }}
        >
          {providers &&
            providers.map((prov) => {
              const labelId = `checkbox-list-label-${prov.id}`;

              return (
                <ListItem key={prov.id} disablePadding>
                  <ListItemButton role={undefined} onClick={handleToggle(prov.id)} dense>
                    <ListItemIcon>
                      <Checkbox
                        edge="start"
                        checked={selectedProviders.findIndex((selProv) => selProv.id === prov.id) !== -1}
                        tabIndex={-1}
                        disableRipple
                        inputProps={{ "aria-labelledby": labelId }}
                      />
                    </ListItemIcon>
                    <ListItemText id={labelId} primary={prov.name()} />
                  </ListItemButton>
                </ListItem>
              );
            })}
        </List>
        {showProgress && <LinearProgress variant="determinate" value={progress} />}
      </DialogContent>
      {!onShutdown && (
        <DialogActions>
          <Button color="primary" onClick={handleClose}>
            Cancel
          </Button>

          <Button autoFocus color="success" variant="contained" onClick={onConfirm}>
            Confirm
          </Button>
        </DialogActions>
      )}
      {onShutdown && (
        <DialogActions>
          <Typography>Shutting down</Typography>
          <CircularProgress size="1em" />

          <Button autoFocus color="warning" variant="contained" onClick={onForceCloseCallback}>
            close app
          </Button>
        </DialogActions>
      )}
    </Dialog>
  );
}

ListSelectionModal.propTypes = {
  title: PropTypes.string,
  providers: PropTypes.array.isRequired,
  onCloseCallback: PropTypes.func.isRequired,
  onConfirmCallback: PropTypes.func.isRequired,
  onForceCloseCallback: PropTypes.func.isRequired,
};

export default ListSelectionModal;
