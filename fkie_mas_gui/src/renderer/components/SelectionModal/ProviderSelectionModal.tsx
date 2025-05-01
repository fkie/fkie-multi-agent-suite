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
import { ForwardedRef, forwardRef, useCallback, useEffect, useRef, useState } from "react";

import { Provider } from "@/renderer/providers";
import { delay } from "@/renderer/utils";
import DraggablePaper from "../UI/DraggablePaper";

interface ProviderSelectionModalProps {
  providers: Provider[];
  title?: string;
  onConfirmCallback: (items: Provider[]) => void;
  onCloseCallback: () => void;
  onForceCloseCallback: () => void;
  onToggle: (item: string, value: boolean) => void;
}

const ProviderSelectionModal = forwardRef<HTMLDivElement, ProviderSelectionModalProps>(
  function ProviderSelectionModal(props, ref) {
    const {
      title = "Select providers",
      providers,
      onConfirmCallback = (): void => {},
      onCloseCallback = (): void => {},
      onForceCloseCallback = (): void => {},
      onToggle = (): void => {},
    } = props;
    const [selectedProviders, setSelectedProviders] = useState<Provider[]>([]);
    const [progress, setProgress] = useState<number>(100);
    const [showProgress, setShowProgress] = useState<boolean>(true);
    const [onShutdown, setOnShutdown] = useState<boolean>(false);

    function handleToggle(value: string): void {
      const currentIndex = selectedProviders.findIndex((prov) => prov.id === value);
      const newSelectedProviders: Provider[] = [...selectedProviders];

      if (currentIndex === -1) {
        const selected = providers.find((provider: Provider) => provider.id === value);
        if (selected) {
          newSelectedProviders.push(selected);
        }
        onToggle(value, true);
      } else {
        newSelectedProviders.splice(currentIndex, 1);
        onToggle(value, false);
      }

      setSelectedProviders(newSelectedProviders);
      setShowProgress(false);
    }

    function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
      if (reason && reason === "backdropClick") return;
      onCloseCallback();
    }

    function onConfirm(): void {
      setOnShutdown(true);
      onConfirmCallback(selectedProviders);
      // handleClose();
    }

    const performCloseProgress = useCallback(
      async (oldProgress: number) => {
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

    const dialogRef = useRef(ref);

    return (
      <Dialog
        open
        onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
        fullWidth
        scroll="paper"
        maxWidth="md"
        ref={dialogRef as ForwardedRef<HTMLDivElement>}
        PaperProps={{
          component: DraggablePaper,
          dialogRef: dialogRef,
        }}
        aria-labelledby="draggable-dialog-title"
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
          {title}
        </DialogTitle>

        <DialogContent dividers={false} aria-label="list">
          <List
            sx={{
              width: "100%",
              maxHeight: 400,
              overflow: "auto",
              bgcolor: "background.paper",
            }}
          >
            {providers?.map((prov: Provider) => {
              const labelId = `checkbox-list-label-${prov.id}`;

              return (
                <ListItem key={prov.id} disablePadding>
                  <ListItemButton onClick={() => handleToggle(prov.id)} dense>
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
            <Button color="primary" onClick={() => handleClose("cancel")}>
              Cancel
            </Button>

            <Button autoFocus color="success" variant="contained" onClick={() => onConfirm()}>
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
);

export default ProviderSelectionModal;
