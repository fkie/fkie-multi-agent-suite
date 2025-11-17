import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from "@mui/material";
import { useState } from "react";

import DraggablePaper from "../UI/DraggablePaper";

interface ConfirmModalProps {
  title: string;
  message: string;
  onConfirmCallback: () => void;
  onCancelCallback?: () => void;
  showCancelButton?: boolean;
}

export default function ConfirmModal(props: ConfirmModalProps): JSX.Element {
  const {
    title,
    message,
    onConfirmCallback = (): void => {},
    onCancelCallback = (): void => {},
    showCancelButton = true,
  } = props;

  const [open, setOpen] = useState(true);

  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    // if (reason && reason === 'backdropClick') return;
    setOpen(false);
    if (reason !== "confirmed" && onCancelCallback) {
      onCancelCallback();
    }
  }

  function onConfirm(): void {
    onConfirmCallback();
    handleClose("confirmed");
  }

  return (
    <Dialog
      open={open}
      onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
      fullWidth
      scroll="paper"
      maxWidth="sm"
      PaperComponent={DraggablePaper}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
        {title}
      </DialogTitle>

      <DialogContent dividers={false} aria-label="list">
        <DialogContentText id="alert-dialog-description">{message}</DialogContentText>
      </DialogContent>

      <DialogActions>
        {showCancelButton && (
          <Button autoFocus color="primary" onClick={() => handleClose("cancel")}>
            Cancel
          </Button>
        )}

        <Button autoFocus={!showCancelButton} color="warning" onClick={() => onConfirm()}>
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  );
}
