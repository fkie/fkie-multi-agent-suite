import PropTypes from 'prop-types';
import { useState } from 'react';

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
} from '@mui/material';
import DraggablePaper from '../UI/DraggablePaper';

function ListSelectionModal({
  title,
  providers,
  onCloseCallback,
  onConfirmCallback,
}) {
  const [selectedProviders, setSelectedProviders] = useState([]);

  const handleToggle = (value) => () => {
    const currentIndex = selectedProviders.findIndex(
      (prov) => prov.id === value,
    );
    const newSelectedProviders = [...selectedProviders];

    if (currentIndex === -1) {
      newSelectedProviders.push(
        providers.find((provider) => provider.id === value),
      );
    } else {
      newSelectedProviders.splice(currentIndex, 1);
    }

    setSelectedProviders(newSelectedProviders);
  };

  const handleClose = (event, reason) => {
    if (reason && reason === 'backdropClick') return;
    onCloseCallback();
  };

  const onConfirm = () => {
    onConfirmCallback(selectedProviders);
    handleClose();
  };

  return (
    <Dialog
      open={true}
      onClose={handleClose}
      fullWidth
      maxWidth="md"
      PaperComponent={DraggablePaper}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle style={{ cursor: 'move' }} id="draggable-dialog-title">
        {title}
      </DialogTitle>

      {providers && (
        <DialogContent scroll="paper" aria-label="list">
          <List
            sx={{
              width: '100%',
              maxHeight: 400,
              overflow: 'auto',
              bgcolor: 'background.paper',
            }}
          >
            {providers &&
              providers.map((prov) => {
                const labelId = `checkbox-list-label-${prov.id}`;

                return (
                  <ListItem key={prov.id} disablePadding>
                    <ListItemButton
                      role={undefined}
                      onClick={handleToggle(prov.id)}
                      dense
                    >
                      <ListItemIcon>
                        <Checkbox
                          edge="start"
                          checked={
                            selectedProviders.findIndex(
                              (selProv) => selProv.id === prov.id,
                            ) !== -1
                          }
                          tabIndex={-1}
                          disableRipple
                          inputProps={{ 'aria-labelledby': labelId }}
                        />
                      </ListItemIcon>
                      <ListItemText id={labelId} primary={prov.name()} />
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

ListSelectionModal.defaultProps = {
  title: 'Select providers',
};

ListSelectionModal.propTypes = {
  title: PropTypes.string,
  providers: PropTypes.array.isRequired,
  onCloseCallback: PropTypes.func.isRequired,
  onConfirmCallback: PropTypes.func.isRequired,
};

export default ListSelectionModal;
