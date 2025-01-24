import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import { Button, Dialog, DialogActions, DialogContent, DialogTitle, IconButton, Tooltip } from "@mui/material";
import { useContext, useRef, useState } from "react";

import { SettingsContext } from "@/renderer/context/SettingsContext";
import DraggablePaper from "../UI/DraggablePaper";
import About from "./About";

export default function AboutModal(): JSX.Element {
  const settingsCtx = useContext(SettingsContext);
  const [open, setOpen] = useState(false);
  function handleOpen(): void {
    setOpen(true);
  }
  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
  }

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
        onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
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
            onClick={() => handleClose("cancel")}
            variant="text"
            // style={{ height: 40, textAlign: 'center' }}
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
      <Tooltip title="About" placement="right" enterDelay={settingsCtx.get("tooltipEnterDelay") as number}>
        <IconButton
          sx={{
            // padding: "0.8em",
            color: settingsCtx.get("useDarkMode") ? "#fff" : "rgba(0, 0, 0, 0.54)",
          }}
          onClick={() => handleOpen()}
        >
          <InfoOutlinedIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    </div>
  );
}
