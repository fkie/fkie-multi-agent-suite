import CloseIcon from "@mui/icons-material/Close";
import EditIcon from "@mui/icons-material/Edit";
import RefreshIcon from "@mui/icons-material/Refresh";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import { Box, ListItemAvatar, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";
import IconButton from "@mui/material/IconButton";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemButton from "@mui/material/ListItemButton";
import ListItemText from "@mui/material/ListItemText";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import { forwardRef, useCallback, useContext } from "react";

import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { getFileName, LaunchContent } from "@/renderer/models";
import MuiMarkdown from "mui-markdown";
import { CopyButton } from "../UI";

function compareLaunchFiles(a: LaunchContent, b: LaunchContent): number {
  if (getFileName(a.path) < getFileName(b.path)) {
    return -1;
  }
  if (getFileName(a.path) > getFileName(b.path)) {
    return 1;
  }
  return 0;
}

interface LaunchFileListProps {
  providerId: string;
  launchContentList: LaunchContent[];
  selectNodesFromLaunch: (providerId: string, launch: LaunchContent) => void;
  onRemoveLaunch: (providerId: string, path: string, masteruri: string) => void;
  onReloadLaunch: (providerId: string, path: string, masteruri: string) => void;
  onMouseOver: (event: React.MouseEvent) => void;
}

const LaunchFileList = forwardRef<HTMLDivElement, LaunchFileListProps>(function LaunchFileList(props, ref) {
  const {
    providerId,
    launchContentList,
    selectNodesFromLaunch = (): void => {},
    onRemoveLaunch = (): void => {},
    onReloadLaunch = (): void => {},
    onMouseOver = (): void => {},
  } = props;

  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);

  /**
   * Create and open a new panel with a [FileEditorPanel] for a given file path and host
   */
  const createFileEditorPanel = useCallback(
    async (provId: string, launchContent: LaunchContent, external: boolean): Promise<void> => {
      navCtx.openEditor(provId, launchContent.path, launchContent.path, null, [], external);
    },
    [navCtx]
  );

  /**
   * Create and open a new panel with a [LaunchFilePanel]
   */
  // const createLaunchFilePanel = useCallback(
  //   (provId, launchContent) => {
  //     const provider = rosCtx.getProviderById(provId);
  //     const launchName = getFileName(launchContent.path);
  //     emitCustomEvent(
  //       EVENT_OPEN_COMPONENT,
  //       eventOpenComponent(
  //         `launchFileInfo-${launchName}@${provider.name()}`,
  //         `${launchName}@${provider.name()}`,
  //         <LaunchFilePanel launchContent={launchContent} />,
  //         true,
  //         LAYOUT_TAB_SETS.BORDER_CENTER,
  //         new LayoutTabConfig(false, "info")
  //       )
  //     );
  //   },
  //   [rosCtx]
  // );

  return (
    <Box ref={ref}>
      <List dense disablePadding onMouseOver={(event) => onMouseOver(event)}>
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
                          onReloadLaunch(providerId, lc.path, lc.masteruri || "");
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
                          onRemoveLaunch(providerId, lc.path, lc.masteruri || "");
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
                  {lc.warnings.length > 0 && (
                    <ListItemAvatar sx={{ minWidth: 24 }}>
                      <Tooltip
                        title={
                          <div>
                            <Typography fontWeight="bold" fontSize="inherit">
                              [{lc.warnings.length}] warnings while load launch file:
                            </Typography>
                            <MuiMarkdown>
                              {lc.warnings.join("<br/><br/>").replaceAll("\n", "<br/>").replaceAll("_", "\\_")}
                            </MuiMarkdown>
                          </div>
                        }
                        placement="bottom"
                        disableInteractive
                      >
                        <IconButton
                          edge="end"
                          aria-label="load launch warnings"
                          onClick={(event) => {
                            navigator.clipboard.writeText(lc.warnings.join("\n"));
                            logCtx.info("Warnings copied to clipboard");
                            event.stopPropagation();
                          }}
                        >
                          <WarningAmberIcon color="warning" fontSize="inherit" />
                        </IconButton>
                      </Tooltip>
                    </ListItemAvatar>
                  )}
                  <ListItemText
                    primary={`${launchName} [${lc.nodes ? lc.nodes.length : 0}]`}
                    sx={{
                      color: settingsCtx.get("useDarkMode") ? blue[300] : blue[800],
                    }}
                  />
                </ListItemButton>
              </ListItem>
            );
          })}
      </List>
    </Box>
  );
});

export default LaunchFileList;
