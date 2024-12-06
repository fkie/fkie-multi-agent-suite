import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from "@mui/material";
import PropTypes from "prop-types";
import { useRef, useState } from "react";
import DraggablePaper from "../UI/DraggablePaper";

function ConfirmModal({ title, message, onConfirmCallback, onCancelCallback }) {
  const [open, setOpen] = useState(true);

  const handleClose = (event, reason) => {
    // if (reason && reason === 'backdropClick') return;
    setOpen(false);
    if (reason !== "confirmed" && onCancelCallback) {
      onCancelCallback();
    }
  };

  const onConfirm = () => {
    onConfirmCallback();
    handleClose(null, "confirmed");
  };

  const dialogRef = useRef(null);

  return (
    <Dialog
      open={open}
      onClose={handleClose}
      fullWidth
      maxWidth="sm"
      ref={dialogRef}
      PaperProps={{
        component: DraggablePaper,
        dialogRef: dialogRef,
      }}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
        {title}
      </DialogTitle>

      <DialogContent scroll="paper" aria-label="list">
        <DialogContentText id="alert-dialog-description">{message}</DialogContentText>
      </DialogContent>

      <DialogActions>
        <Button autoFocus color="primary" onClick={handleClose}>
          Cancel
        </Button>

        <Button color="warning" onClick={onConfirm}>
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  );
}

ConfirmModal.propTypes = {
  title: PropTypes.string.isRequired,
  message: PropTypes.string.isRequired,
  onConfirmCallback: PropTypes.func.isRequired,
  onCancelCallback: PropTypes.func.isRequired,
};

export default ConfirmModal;
