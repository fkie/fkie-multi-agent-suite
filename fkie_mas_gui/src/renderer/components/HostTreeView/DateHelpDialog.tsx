import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  Typography,
} from "@mui/material";
import { forwardRef } from "react";

interface DateHelpDialogProps {
  open: boolean;
  onClose: () => void;
}

const DateHelpDialog = forwardRef<HTMLDivElement, DateHelpDialogProps>(function DateHelpDialog(props, ref) {
  const { open, onClose = (): void => {}, ...other } = props;

  const codeSnippet = `
  USER  ALL=NOPASSWD: /bin/date
  `;

  return (
    <Dialog
      ref={ref}
      sx={{ "& .MuiDialog-paper": { width: "80%", maxHeight: 435 } }}
      maxWidth="md"
      open={open}
      {...other}
    >
      <DialogTitle className="draggable-dialog-title" id="draggable-dialog-title">
        Set time
      </DialogTitle>
      <DialogContent dividers>
        <DialogContentText>
          <Typography>
            You can sync time to an NTP service using <b>ntpdate</b> option.
          </Typography>
          <Typography>
            If no NTP service is available you can change your local time with <b>sync me to this date</b> or update the
            time of the selected host using <b>set date</b>.
          </Typography>
          <Typography>
            In both cases we use <b>/bin/date</b> to change the system time.
          </Typography>
          <Typography>
            To avoid big time differences after setting the date you need to modify /etc/sudoers with sudoedit and add
            user privilege (replace USER), e.g.:
          </Typography>
          <pre>
            <code>{codeSnippet}</code>
          </pre>
          <Typography>!!!needed to be at the very end of file, don&apos;t forget a new line at the end!!!</Typography>
          <Typography>
            Be aware, it does not replace the time synchronization! It sets approximate time without undue delays on
            communication layer.
          </Typography>
        </DialogContentText>
      </DialogContent>
      <DialogActions>
        <Button autoFocus onClick={onClose}>
          Close
        </Button>
      </DialogActions>
    </Dialog>
  );
});

export default DateHelpDialog;
