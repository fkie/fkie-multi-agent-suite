import {
  Button,
  Checkbox,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  FormControlLabel,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Stack,
} from "@mui/material";
import { ForwardedRef, forwardRef, useRef, useState } from "react";

import DraggablePaper from "../UI/DraggablePaper";

interface ListSelectionModalProps {
  title: string;
  list: string[];
  onConfirmCallback: (items: string[]) => void;
  onCancelCallback?: () => void;
}

const ListSelectionModal = forwardRef<HTMLDivElement, ListSelectionModalProps>(function ListSelectionModal(props, ref) {
  const {
    title = "Confirm Selection",
    list,
    onConfirmCallback = (): void => {},
    onCancelCallback = (): void => {},
  } = props;

  const [open, setOpen] = useState<boolean>(true);
  const [selectedItems, setSelectedItems] = useState<string[]>(list.length < 5 ? list : []);

  function handleToggle(value: string): void {
    const currentIndex = selectedItems.indexOf(value);
    const newSelectedItems = [...selectedItems];

    if (currentIndex === -1) {
      newSelectedItems.push(value);
    } else {
      newSelectedItems.splice(currentIndex, 1);
    }

    setSelectedItems(newSelectedItems);
  }

  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    setSelectedItems([]);
    setOpen(false);
    if (reason !== "confirmed" && onCancelCallback) {
      onCancelCallback();
    }
  }

  function onConfirm(): void {
    onConfirmCallback(selectedItems);
    handleClose("confirmed");
  }

  const dialogRef = useRef(ref);

  return (
    <Dialog
      ref={dialogRef as ForwardedRef<HTMLDivElement>}
      open={open}
      onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
      fullWidth
      scroll="paper"
      maxWidth="md"
      PaperProps={{
        component: DraggablePaper,
        dialogRef: dialogRef,
      }}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
        {title}
      </DialogTitle>

      {selectedItems && (
        <Stack spacing={0}>
          <DialogContent dividers={true} aria-label="list">
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
                      <ListItemButton role={undefined} onClick={() => handleToggle(node)} dense>
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
        <Button color="primary" onClick={() => handleClose("cancel")}>
          Cancel
        </Button>

        <Button autoFocus color="success" variant="contained" onClick={() => onConfirm()}>
          Confirm
        </Button>
      </DialogActions>
    </Dialog>
  );
});

export default ListSelectionModal;
