import SettingsIcon from "@mui/icons-material/Settings";
import {
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  Tab,
  Tabs,
  Tooltip,
} from "@mui/material";
import { useContext, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { SettingsContext } from "../../context/SettingsContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import { EVENT_OPEN_SETTINGS, SETTING } from "../../utils/events";
import DraggablePaper from "../UI/DraggablePaper";
import TabPanel from "../UI/TabPanel";
import About from "./About";
import GuiPanel from "./GuiPanel";
import SSHCredentialsPanel from "./SSHCredentialsPanel";

function SettingsModal() {
  const settingsCtx = useContext(SettingsContext);
  const [selectedTabIndex, setSelectedTabIndex] = useLocalStorage("SettingsModal:selectedTabIndex", 1);
  const [open, setOpen] = useState(false);
  const handleOpen = () => setOpen(true);
  const handleClose = (event, reason) => {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
  };

  function a11yProps(index) {
    return {
      id: `simple-tab-${index}`,
      "aria-controls": `simple-tabpanel-${index}`,
    };
  }

  useCustomEventListener(EVENT_OPEN_SETTINGS, (data) => {
    if (data.id === SETTING.IDS.SSH) {
      setSelectedTabIndex(1);
      setOpen(true);
    }
    if (data.id === SETTING.IDS.INTERFACE) {
      setSelectedTabIndex(0);
      setOpen(true);
    }
    if (data.id === SETTING.IDS.ABOUT) {
      setSelectedTabIndex(2);
      setOpen(true);
    }
  });

  return (
    <div>
      <Dialog
        keepMounted
        id="settings-dialog"
        scroll="paper"
        PaperComponent={DraggablePaper}
        aria-labelledby="draggable-dialog-title"
        fullWidth
        maxWidth="md"
        open={open}
        onClose={handleClose}
        // disableEscapeKeyDown
      >
        <DialogTitle style={{ cursor: "move" }} id="draggable-dialog-title">
          Settings editor
        </DialogTitle>

        <DialogContent>
          <Box sx={{ borderBottom: 1, borderColor: "divider" }}>
            <Tabs
              value={selectedTabIndex}
              onChange={(event, newValue) => {
                setSelectedTabIndex(newValue);
              }}
              aria-label="List of tabs"
            >
              <Tab label="Interface" {...a11yProps(0)} />
              <Tab label="SSH Credentials" {...a11yProps(1)} />
              <Tab label="About" {...a11yProps(2)} />
            </Tabs>
          </Box>
          <TabPanel value={selectedTabIndex} index={0}>
            <GuiPanel />
          </TabPanel>
          <TabPanel value={selectedTabIndex} index={1}>
            <SSHCredentialsPanel />
          </TabPanel>
          <TabPanel value={selectedTabIndex} index={2}>
            <About />
          </TabPanel>{" "}
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
      <Tooltip title="Settings" placement="right" enterDelay={settingsCtx.get("tooltipEnterDelay")}>
        <IconButton
          sx={{
            padding: "0.8em",
            color: settingsCtx.get("useDarkMode") ? "#fff" : "rgba(0, 0, 0, 0.54)",
          }}
          onClick={handleOpen}
        >
          <SettingsIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    </div>
  );
}

export default SettingsModal;
