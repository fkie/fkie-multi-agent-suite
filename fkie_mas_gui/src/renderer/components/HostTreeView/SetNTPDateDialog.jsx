import { Autocomplete, Box, Button, Dialog, DialogActions, DialogContent, DialogTitle, TextField } from "@mui/material";
import PropTypes from "prop-types";
import { useContext, useEffect, useState } from "react";
import { SettingsContext } from "../../context/SettingsContext";

function SetNTPDateDialog({ onClose, value: valueProp, open, ...other }) {
  const settingsCtx = useContext(SettingsContext);
  const [value, setValue] = useState(valueProp);
  const [timerServer, setTimerServer] = useState(settingsCtx.get("ntpServer"));
  const [timerServerValue, setTimerServerValue] = useState(settingsCtx.get("ntpServer")[0]);

  useEffect(() => {
    if (!open) {
      setValue(valueProp);
    }
  }, [valueProp, open]);

  const handleCancel = () => {
    onClose();
  };

  const handleOk = () => {
    if (timerServerValue) {
      if (!settingsCtx.get("ntpServer").includes(timerServerValue)) {
        setTimerServer([timerServerValue, ...settingsCtx.get("ntpServer")]);
        settingsCtx.set("ntpServer", [timerServerValue, ...settingsCtx.get("ntpServer")]);
      }
      onClose(timerServerValue ? `${value} ${timerServerValue}` : "");
    }
  };

  return (
    <Dialog sx={{ "& .MuiDialog-paper": { width: "80%", maxHeight: 435 } }} maxWidth="xs" open={open} {...other}>
      <DialogTitle className="draggable-dialog-title" id="draggable-dialog-title">
        Update system time
      </DialogTitle>
      <DialogContent dividers>
        <Box>
          <div>{`${value}`}</div>
          <Autocomplete
            handleHomeEndKeys={false}
            disablePortal
            id="auto-complete-donotsync"
            size="small"
            options={timerServer}
            freeSolo
            sx={{ margin: 0 }}
            renderInput={(params) => <TextField {...params} variant="outlined" label="Time Server" />}
            value={timerServer[0]}
            // inputValue={timerServerValue}
            onChange={(event, newValue) => {
              setTimerServerValue(newValue);
            }}
            onInputChange={(event, newInputValue) => {
              setTimerServerValue(newInputValue);
            }}
          />
        </Box>
      </DialogContent>
      <DialogActions>
        <Button autoFocus onClick={handleCancel}>
          Cancel
        </Button>
        <Button onClick={handleOk}>Ok</Button>
      </DialogActions>
    </Dialog>
  );
}

SetNTPDateDialog.propTypes = {
  onClose: PropTypes.func.isRequired,
  open: PropTypes.bool.isRequired,
  value: PropTypes.string.isRequired,
};

export default SetNTPDateDialog;
