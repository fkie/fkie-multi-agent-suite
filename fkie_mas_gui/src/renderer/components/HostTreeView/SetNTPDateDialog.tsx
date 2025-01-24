import { Autocomplete, Box, Button, Dialog, DialogActions, DialogContent, DialogTitle, TextField } from "@mui/material";
import { forwardRef, useContext, useEffect, useState } from "react";

import { SettingsContext } from "@/renderer/context/SettingsContext";

interface SetNTPDateDialogProps {
  open: boolean;
  defaultCmd: string;
  onClose: (cmd: string) => void;
}

const SetNTPDateDialog = forwardRef<HTMLDivElement, SetNTPDateDialogProps>(function SetNTPDateDialog(props, ref) {
  const { open, defaultCmd, onClose = (): void => {} } = props;

  const settingsCtx = useContext(SettingsContext);
  const [value, setValue] = useState<string>(defaultCmd);
  const [timerServer, setTimerServer] = useState<string[]>(settingsCtx.get("ntpServer") as string[]);
  const [timerServerValue, setTimerServerValue] = useState(timerServer?.length > 0 ? timerServer[0] : "");

  useEffect(() => {
    if (!open) {
      setValue(defaultCmd);
    }
  }, [defaultCmd, open]);

  function handleCancel(): void {
    onClose("");
  }

  function handleOk(): void {
    if (timerServerValue) {
      if (!(settingsCtx.get("ntpServer") as string[])?.includes(timerServerValue)) {
        setTimerServer([timerServerValue, ...(settingsCtx.get("ntpServer") as string[])]);
        settingsCtx.set("ntpServer", [timerServerValue, ...(settingsCtx.get("ntpServer") as string[])]);
      }
      onClose(timerServerValue ? `${value} ${timerServerValue}` : "");
    }
  }

  return (
    <Dialog
      ref={ref}
      sx={{ "& .MuiDialog-paper": { width: "80%", maxHeight: 435 } }}
      maxWidth="xs"
      open={open}
      keepMounted
    >
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
            onChange={(_event, newValue) => {
              setTimerServerValue(newValue ? newValue : "");
            }}
            onInputChange={(_event, newInputValue) => {
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
});

export default SetNTPDateDialog;
