import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import { Button, Dialog, DialogActions, DialogContent, DialogTitle, IconButton, Tooltip } from "@mui/material";
import { useContext, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { SettingsContext } from "../../context/SettingsContext";
import { EVENT_OPEN_SETTINGS, SETTING } from "../../pages/NodeManager/layout/events";
import DraggablePaper from "../UI/DraggablePaper";
import About from "./About";

function AboutModal() {
  const settingsCtx = useContext(SettingsContext);
  const [open, setOpen] = useState(false);
  const handleOpen = () => setOpen(true);
  const handleClose = (event, reason) => {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
  };

  useCustomEventListener(EVENT_OPEN_SETTINGS, (data) => {
    if (data.id === SETTING.IDS.ABOUT) {
      setOpen(true);
    }
  });

  const dialogRef = useRef(null);

  return (
    <div>
      <Dialog
        keepMounted
        id="about-dialog"
        scroll="paper"
        ref={dialogRef}
        PaperProps={{
          component: DraggablePaper,
          dialogRef: dialogRef,
          sx: {
            minHeight: "80vh",
            maxHeight: "80vh",
          },
        }}
        aria-labelledby="draggable-dialog-title"
        fullWidth
        maxWidth="md"
        open={open}
        onClose={handleClose}
        // disableEscapeKeyDown
      >
        <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
          About
        </DialogTitle>
        <DialogContent sx={{ padding: "16px" }}>
          <About />
        </DialogContent>
        <DialogActions>
          <Button
            color="primary"
            onClick={handleClose}
            variant="text"
            // style={{ height: 40, textAlign: 'center' }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
      <Tooltip title="About" placement="right" enterDelay={settingsCtx.get("tooltipEnterDelay")}>
        <IconButton
          sx={{
            // padding: "0.8em",
            color: settingsCtx.get("useDarkMode") ? "#fff" : "rgba(0, 0, 0, 0.54)",
          }}
          onClick={handleOpen}
        >
          <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    </div>
  );
}

export default AboutModal;
