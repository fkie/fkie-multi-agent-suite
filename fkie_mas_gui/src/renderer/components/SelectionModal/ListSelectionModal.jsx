import {
  Button,
  Checkbox,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
} from "@mui/material";
import PropTypes from "prop-types";
import { useRef, useState } from "react";
import DraggablePaper from "../UI/DraggablePaper";

function ListSelectionModal({ list, setList, onConfirmCallback }) {
  const [open, setOpen] = useState(true);
  const [selectedItems, setSelectedItems] = useState(list);

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
    setList(null);
    setSelectedItems(null);
    setOpen(false);
  };

  const onConfirm = () => {
    onConfirmCallback(selectedItems);
    handleClose();
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
        Confirm Selection
      </DialogTitle>

      {selectedItems && (
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
                        />
                      </ListItemIcon>
                      <ListItemText id={labelId} primary={node} />
                    </ListItemButton>
                  </ListItem>
                );
              })}
          </List>
        </DialogContent>
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
  list: PropTypes.array.isRequired,
  setList: PropTypes.func.isRequired,
  onConfirmCallback: PropTypes.func.isRequired,
};

export default ListSelectionModal;
