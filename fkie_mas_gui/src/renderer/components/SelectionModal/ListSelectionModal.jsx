import {
  Button,
  Checkbox,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  FormControlLabel,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Stack,
} from "@mui/material";
import PropTypes from "prop-types";
import { useRef, useState } from "react";
import DraggablePaper from "../UI/DraggablePaper";

function ListSelectionModal({ title = "Confirm Selection", list, onConfirmCallback, onCancelCallback = null }) {
  const [open, setOpen] = useState(true);
  const [selectedItems, setSelectedItems] = useState(list.length < 5 ? list : []);

  const handleToggle = (value) => () => {
    const currentIndex = selectedItems.indexOf(value);
    const newSelectedItems = [...selectedItems];

    if (currentIndex === -1) {
      newSelectedItems.push(value);
    } else {
      newSelectedItems.splice(currentIndex, 1);
    }

    setSelectedItems(newSelectedItems);
  };

  const handleClose = (event, reason) => {
    if (reason && reason === "backdropClick") return;
    setSelectedItems([]);
    setOpen(false);
    if (reason !== "confirmed" && onCancelCallback) {
      onCancelCallback();
    }
  };

  const onConfirm = () => {
    onConfirmCallback(selectedItems);
    handleClose(null, "confirmed");
  };

  const dialogRef = useRef(null);

  return (
    <Dialog
      open={open}
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

      {selectedItems && (
        <Stack spacing={0}>
          <DialogContent scroll="paper" aria-label="list">
            <List
              sx={{
                width: "100%",
                maxHeight: 400,
                overflow: "auto",
                bgcolor: "background.paper",
              }}
            >
              {list &&
                list.map((node) => {
                  const labelId = `checkbox-list-label-${node}`;

                  return (
                    <ListItem key={node} disablePadding>
                      <ListItemButton role={undefined} onClick={handleToggle(node)} dense>
                        <ListItemIcon>
                          <Checkbox
                            edge="start"
                            checked={selectedItems.indexOf(node) !== -1}
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
          </DialogContent>
          <FormControlLabel
            label="Select all"
            control={
              <Checkbox
                checked={selectedItems.length === list.length}
                indeterminate={selectedItems.length > 0 && selectedItems.length < list.length}
                sx={{ marginLeft: "0.8em", paddingBottom: 0, marginBottom: 0 }}
                onChange={(event) => {
                  if (event.target.checked) {
                    setSelectedItems(list);
                  } else {
                    setSelectedItems([]);
                  }
                }}
              />
            }
          />
        </Stack>
      )}

      <DialogActions>
        <Button color="primary" onClick={handleClose}>
          Cancel
        </Button>

        <Button autoFocus color="success" variant="contained" onClick={onConfirm}>
          Confirm
        </Button>
      </DialogActions>
    </Dialog>
  );
}

ListSelectionModal.propTypes = {
  title: PropTypes.string,
  list: PropTypes.array.isRequired,
  onConfirmCallback: PropTypes.func.isRequired,
  onCancelCallback: PropTypes.func,
};

export default ListSelectionModal;
