import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from "@mui/material";
import { ForwardedRef, forwardRef, useRef, useState } from "react";
import DraggablePaper from "../UI/DraggablePaper";

interface ConfirmModalProps {
  title: string;
  message: string;
  onConfirmCallback: () => void;
  onCancelCallback: () => void;
}

const ConfirmModal = forwardRef<HTMLDivElement, ConfirmModalProps>(function ConfirmModal(props, ref) {
  const { title, message, onConfirmCallback = () => {}, onCancelCallback = () => {} } = props;

  const [open, setOpen] = useState(true);

  const handleClose = (reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel") => {
    // if (reason && reason === 'backdropClick') return;
    setOpen(false);
    if (reason !== "confirmed" && onCancelCallback) {
      onCancelCallback();
    }
  };

  const onConfirm = () => {
    onConfirmCallback();
    handleClose("confirmed");
  };

  const dialogRef = useRef(ref);

  return (
    <Dialog
      ref={dialogRef as ForwardedRef<HTMLDivElement>}
      open={open}
      onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
      fullWidth
      scroll="paper"
      maxWidth="sm"
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
        <DialogContentText id="alert-dialog-description">{message}</DialogContentText>
      </DialogContent>

      <DialogActions>
        <Button autoFocus color="primary" onClick={() => handleClose("cancel")}>
          Cancel
        </Button>

        <Button color="warning" onClick={() => onConfirm()}>
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  );
});

export default ConfirmModal;
