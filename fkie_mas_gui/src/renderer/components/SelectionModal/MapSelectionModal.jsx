import PropTypes from 'prop-types';
import { useState } from 'react';

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
  ListSubheader,
  Radio,
  RadioGroup,
} from '@mui/material';
import DraggablePaper from '../UI/DraggablePaper';

function MapSelectionModal({
  list,
  onConfirmCallback,
  onCancelCallback = null,
  useRadioGroup = false,
}) {
  const [open, setOpen] = useState(true);
  const [selectedItems, setSelectedItems] = useState(
    list.map((o) => {
      return { title: o.title, list: o.list.length <= 3 ? o.list : [] };
    }),
  );

  const initRadioMap = (initList) => {
    // convert list to map the 'title' as key and first list element as item
    const result = {};
    initList.forEach((o) => {
      [result[o.title]] = o.list;
    });
    return result;
  };

  const [selectedRadioItems, setSelectedRadioItems] = useState(
    initRadioMap(list),
  );

  const handleToggle = (title, value) => {
    const newSelectedItems = structuredClone(selectedItems);
    const item = newSelectedItems.find(
      (o) => title.localeCompare(o.title) === 0,
    );
    if (item) {
      const currentIndex = item.list.indexOf(value);
      if (currentIndex === -1) {
        item.list.push(value);
      } else {
        item.list.splice(currentIndex, 1);
      }
    }
    setSelectedItems(newSelectedItems);
  };

  const handleRadio = (title, value) => {
    const newSelectedItems = { ...selectedRadioItems };
    newSelectedItems[title] = value;
    setSelectedRadioItems(newSelectedItems);
  };

  const handleClose = (event, reason) => {
    if (reason && reason === 'backdropClick') return;
    setSelectedItems([]);
    setOpen(false);
    if (reason !== 'confirmed' && onCancelCallback) {
      onCancelCallback();
    }
  };

  const onConfirm = () => {
    if (useRadioGroup) {
      // convert map state to list of maps with 'title' and 'list' keys
      const result = Object.keys(selectedRadioItems).map((key) => {
        return { title: key, list: [selectedRadioItems[key]] };
      });
      onConfirmCallback(result);
    } else {
      onConfirmCallback(selectedItems);
    }
    handleClose(null, 'confirmed');
  };

  return (
    <Dialog
      key="map-selection"
      open={open}
      onClose={handleClose}
      fullWidth
      maxWidth="md"
      PaperComponent={DraggablePaper}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle style={{ cursor: 'move' }} id="draggable-dialog-title">
        Confirm Selection
      </DialogTitle>

      {list && (
        <DialogContent scroll="paper" aria-label="list">
          <List
            sx={{
              width: '100%',
              overflow: 'auto',
              // bgcolor: 'background.paper',
              '& ul': { padding: 0, margin: 0 },
            }}
          >
            {list.map((node) => {
              const labelId = `list-${node.title}`;
              return (
                <li key={`section-${labelId}`}>
                  <ul style={{ padding: 0, marginBottom: 0 }}>
                    <ListSubheader sx={{ padding: 0 }}>
                      {node.title}
                    </ListSubheader>
                    {useRadioGroup && (
                      <RadioGroup
                        key={`radio-group-${node.title}`}
                        name={`radio-group-${node.title}`}
                        value={selectedRadioItems[node.title]}
                        onChange={(event) => {
                          handleRadio(node.title, event.target.value);
                        }}
                      >
                        {node.list &&
                          node.list.map((item) => (
                            <FormControlLabel
                              key={`label-${item}`}
                              value={item}
                              control={<Radio size="small" />}
                              label={item}
                            />
                          ))}
                      </RadioGroup>
                    )}
                    {!useRadioGroup &&
                      node.list &&
                      node.list.map((item) => {
                        return (
                          <ListItem key={item} disablePadding>
                            <ListItemButton
                              role={undefined}
                              onClick={() => handleToggle(node.title, item)}
                              dense
                            >
                              <ListItemIcon>
                                <Checkbox
                                  size="small"
                                  edge="start"
                                  checked={
                                    selectedItems.find(
                                      (o) =>
                                        node.title.localeCompare(o.title) ===
                                          0 && o.list.indexOf(item) !== -1,
                                    ) !== undefined
                                  }
                                  tabIndex={-1}
                                  disableRipple
                                  inputProps={{
                                    'aria-labelledby': labelId,
                                  }}
                                />
                              </ListItemIcon>
                              <ListItemText id={labelId} primary={item} />
                            </ListItemButton>
                          </ListItem>
                        );
                      })}
                  </ul>
                </li>
              );
            })}
          </List>
        </DialogContent>
      )}

      <DialogActions>
        <Button color="primary" onClick={handleClose}>
          Cancel
        </Button>

        <Button
          autoFocus
          color="success"
          variant="contained"
          onClick={onConfirm}
        >
          Confirm
        </Button>
      </DialogActions>
    </Dialog>
  );
}

MapSelectionModal.propTypes = {
  list: PropTypes.array.isRequired, // [{'title': 'ab', 'list': ['a', 'b']}]
  onConfirmCallback: PropTypes.func.isRequired,
  onCancelCallback: PropTypes.func,
  useRadioGroup: PropTypes.bool,
};

export default MapSelectionModal;
