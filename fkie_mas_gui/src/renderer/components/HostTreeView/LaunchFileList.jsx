import CloseIcon from "@mui/icons-material/Close";
import EditIcon from "@mui/icons-material/Edit";
import RefreshIcon from "@mui/icons-material/Refresh";
import { blue } from "@mui/material/colors";
import IconButton from "@mui/material/IconButton";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemButton from "@mui/material/ListItemButton";
import ListItemText from "@mui/material/ListItemText";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import PropTypes from "prop-types";
import { useCallback, useContext } from "react";
import { emitCustomEvent } from "react-custom-events";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getFileName } from "../../models";
import { LAYOUT_TAB_SETS, LayoutTabConfig } from "../../pages/NodeManager/layout";
import LaunchFilePanel from "../../pages/NodeManager/panels/LaunchFilePanel";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../utils/events";

const compareLaunchFiles = (a, b) => {
  if (getFileName(a.path) < getFileName(b.path)) {
    return -1;
  }
  if (getFileName(a.path) > getFileName(b.path)) {
    return 1;
  }
  return 0;
};

function LaunchFileList({
  providerId,
  launchContentList,
  selectNodesFromLaunch,
  onRemoveLaunch,
  onReloadLaunch,
  onMouseOver = () => {},
}) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  /**
   * Create and open a new panel with a [FileEditorPanel] for a given file path and host
   */
  const createFileEditorPanel = useCallback(
    async (provId, launchContent, external) => {
      rosCtx.openEditor(provId, launchContent.path, launchContent.path, null, external);
    },
    [rosCtx]
  );

  /**
   * Create and open a new panel with a [LaunchFilePanel]
   */
  const createLaunchFilePanel = useCallback(
    (provId, launchContent) => {
      const provider = rosCtx.getProviderById(provId);
      const launchName = getFileName(launchContent.path);
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `launchFileInfo-${launchName}@${provider.name()}`,
          `${launchName}@${provider.name()}`,
          <LaunchFilePanel launchContent={launchContent} />,
          true,
          LAYOUT_TAB_SETS.BORDER_CENTER,
          new LayoutTabConfig(false, "info")
        )
      );
    },
    [rosCtx]
  );

  return (
    <List dense disablePadding onMouseOver={onMouseOver}>
      {launchContentList.length > 0 &&
        launchContentList.sort(compareLaunchFiles).map((lc) => {
          if (!lc.path) return <div key={`${lc.path}`}>Invalid launch file</div>;
          const launchName = getFileName(lc.path);
          return (
            <ListItem
              key={`${providerId}_${lc.path}`}
              disablePadding
              onClick={() => {
                selectNodesFromLaunch(providerId, lc);
              }}
              secondaryAction={
                <Stack direction="row" spacing={0.1}>
                  {/* <Tooltip title="Information about launch file">
                    <IconButton
                      edge="end"
                      aria-label="Information about launch file"
                      onClick={(event) => {
                        createLaunchFilePanel(providerId, lc);
                        event.stopPropagation();
                      }}
                    >
                      <InfoOutlinedIcon style={{ fontSize: 'inherit' }} />
                    </IconButton>
                  </Tooltip> */}

                  <Tooltip title="Reload launch">
                    <IconButton
                      edge="end"
                      aria-label="Reload launch"
                      onClick={(event) => {
                        onReloadLaunch(providerId, lc.path, lc.masteruri);
                        event.stopPropagation();
                      }}
                    >
                      <RefreshIcon sx={{ fontSize: "inherit" }} />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Edit launch file">
                    <IconButton
                      edge="end"
                      aria-label="Edit launch file"
                      onClick={(event) => {
                        createFileEditorPanel(providerId, lc, event.nativeEvent.shiftKey);
                        event.stopPropagation();
                      }}
                    >
                      <EditIcon sx={{ fontSize: "inherit" }} />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Unload launch file">
                    <IconButton
                      edge="end"
                      aria-label="Unload launch file"
                      onClick={(event) => {
                        onRemoveLaunch(providerId, lc.path, lc.masteruri);
                        event.stopPropagation();
                      }}
                    >
                      <CloseIcon sx={{ fontSize: "inherit" }} />
                    </IconButton>
                  </Tooltip>
                </Stack>
              }
            >
              <ListItemButton dense>
                <ListItemText
                  primary={`${launchName} [${lc.nodes.length}]`}
                  sx={{
                    color: settingsCtx.get("useDarkMode") ? blue[300] : blue[800],
                  }}
                />
              </ListItemButton>
            </ListItem>
          );
        })}
    </List>
  );
}

LaunchFileList.propTypes = {
  providerId: PropTypes.string.isRequired,
  launchContentList: PropTypes.array.isRequired,
  selectNodesFromLaunch: PropTypes.func.isRequired,
  onRemoveLaunch: PropTypes.func.isRequired,
  onReloadLaunch: PropTypes.func.isRequired,
  onMouseOver: PropTypes.func,
};

export default LaunchFileList;
