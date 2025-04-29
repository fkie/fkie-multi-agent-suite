import ChangeCircleOutlinedIcon from "@mui/icons-material/ChangeCircleOutlined";
import ComputerIcon from "@mui/icons-material/Computer";
import HideSourceIcon from "@mui/icons-material/HideSource";
import WatchLaterIcon from "@mui/icons-material/WatchLater";
import {
  Box,
  ClickAwayListener,
  Grow,
  IconButton,
  MenuItem,
  MenuList,
  Paper,
  Popper,
  Stack,
  Tooltip,
} from "@mui/material";
import { green, grey, orange, red } from "@mui/material/colors";
import {
  TreeItem2SlotProps,
  UseTreeItem2ContentSlotOwnProps,
  UseTreeItem2IconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { RosNode, RosNodeStatus } from "@/renderer/models";
import { LAYOUT_TAB_SETS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import SingleTerminalPanel from "@/renderer/pages/NodeManager/panels/SingleTerminalPanel";
import { CmdType } from "@/renderer/providers";
import Provider from "@/renderer/providers/Provider";
import { generateUniqueId } from "@/renderer/utils";
import { TTag } from "@/types";
import { colorFromHostname } from "../UI/Colors";
import Tag from "../UI/Tag";
import DateHelpDialog from "./DateHelpDialog";
import SetNTPDateDialog from "./SetNTPDateDialog";
import StyledRootTreeItem from "./StyledRootTreeItem";

interface HostItemProps {
  provider: Provider;
  stopNodes: (nodeIdGlobals: string[]) => void;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
  children: React.ReactNode;
}

const HostItem = forwardRef<HTMLDivElement, HostItemProps>(function HostItem(props, ref) {
  const { provider, stopNodes = (): void => {}, onDoubleClick = (): void => {}, ...children } = props;
  const settingsCtx = useContext(SettingsContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);

  const optionsTimeButton = ["ntpdate", "set date", "sync me to this date", "help"];
  const [openTimeButton, setOpenTimeButton] = useState(false);
  const anchorRef = useRef(null);
  const [showHelpTime, setShowHelpTime] = useState<boolean>(false);
  const [openNtpdateDialog, setOpenNtpdateDialog] = useState<boolean>(false);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [timeDiffThreshold, setTimeDiffThreshold] = useState<number>(settingsCtx.get("timeDiffThreshold") as number);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setTimeDiffThreshold(settingsCtx.get("timeDiffThreshold") as number);
  }, [settingsCtx, settingsCtx.changed]);

  async function updateTime(local = true): Promise<void> {
    if (provider) {
      const localProviders = rosCtx.getLocalProvider();
      if (localProviders.length === 0) {
        logCtx.error("localhost provider not found", "", true);
        return;
      }
      if (local) {
        navCtx.openTerminal(
          CmdType.SET_TIME,
          localProviders[0].id,
          `set_date_localhost-${Date.now()}`,
          "",
          provider.id,
          false,
          false
        );
      } else {
        navCtx.openTerminal(
          CmdType.SET_TIME,
          provider.id,
          `set_date_remote-${Date.now()}`,
          "",
          localProviders[0].id,
          false,
          false
        );
      }
    }
  }

  function handleMenuTimeItemClick(_event, index): void {
    if (index === 0) {
      // set time using ntpdate
      setOpenNtpdateDialog(true);
    } else if (index === 1) {
      // set remote time using date
      if (provider) {
        updateTime(false);
      }
    } else if (index === 2) {
      // set local time using date
      updateTime(true);
    } else if (index === 3) {
      setShowHelpTime(true);
    }
    setOpenTimeButton(false);
  }

  function handleCloseTimeButton(event: MouseEvent | TouchEvent): void {
    if (anchorRef.current && anchorRef.current === event.target) {
      return;
    }
    setOpenTimeButton(false);
  }

  /**
   * Check if provider has master sync on
   */
  const getMasterSyncNode = useCallback(
    (providerId: string): RosNode | undefined => {
      const foundSyncNode = rosCtx.mapProviderRosNodes.get(providerId)?.find((node) => {
        return node.id.includes("/mas_sync") && node.status === RosNodeStatus.RUNNING;
      });
      return foundSyncNode;
    },
    [rosCtx.mapProviderRosNodes]
  );

  const toggleMasterSync = useCallback(
    (provider: Provider): void => {
      const syncNode: RosNode | undefined = getMasterSyncNode(provider.id);
      if (syncNode) {
        stopNodes([syncNode.idGlobal]);
      } else {
        rosCtx.startMasterSync(provider.connection.host, provider.rosVersion, provider.rosState?.masteruri);
      }
    },
    [getMasterSyncNode, stopNodes, rosCtx.startMasterSync]
  );

  /**
   * Get provider tags
   */
  function getProviderTags(provider: Provider): TTag[] {
    const tags: TTag[] = [];
    if (!provider.daemon) {
      tags.push({ id: "no-daemon", data: "No Daemon", tooltip: "", color: "red" });
    }
    if (!provider.discovery) {
      tags.push({ id: "no-discovery", data: "No Discovery", tooltip: "", color: "red" });
    }
    return tags;
  }

  function formatTime(milliseconds): string {
    const sec = (milliseconds / 1000.0).toFixed(3);
    return `${sec}s`;
  }

  const getHostStyle = useCallback(
    function getHostStyle(provider: Provider): object {
      if (settingsCtx.get("colorizeHosts")) {
        // borderLeft: `3px dashed`,
        // borderColor: colorFromHostname(provider.name()),
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(provider.name()),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [settingsCtx.changed, settingsCtx.get]
  );

  // avoid selection if collapse icon was clicked
  let toggled = false;
  const handleContentClick: UseTreeItem2ContentSlotOwnProps["onClick"] = (event) => {
    event.defaultMuiPrevented = toggled;
    toggled = false;
  };

  const handleLabelClick: UseTreeItem2ContentSlotOwnProps["onClick"] = () => {};

  const handleIconContainerClick: UseTreeItem2IconContainerSlotOwnProps["onClick"] = () => {
    toggled = true;
  };

  return (
    <StyledRootTreeItem
      itemId={provider.id}
      slotProps={
        {
          label: { onClick: handleLabelClick },
          content: { onClick: handleContentClick },
          iconContainer: { onClick: handleIconContainerClick },
        } as TreeItem2SlotProps
      }
      sx={getHostStyle(provider)}
      onDoubleClick={(event) => onDoubleClick(event, provider.id)}
      label={
        <Box ref={ref} display="flex" alignItems="center" paddingLeft={0.0}>
          {provider.rosState.ros_version === "1" && (
            <Tooltip
              title="Toggle Master Sync"
              placement="bottom-start"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
              <IconButton
                edge="start"
                aria-label="Toggle Master Sync"
                onClick={(event) => {
                  toggleMasterSync(provider);
                  event.stopPropagation();
                }}
              >
                <ChangeCircleOutlinedIcon
                  sx={{ color: getMasterSyncNode(provider.id) ? green[500] : grey[700], fontSize: "inherit" }}
                />
              </IconButton>
            </Tooltip>
          )}

          {Math.abs(provider.timeDiff) > timeDiffThreshold && (
            <Tooltip title={`Time not in sync for approx. ${formatTime(provider.timeDiff)}`} placement="right-end">
              <Box>
                <IconButton
                  edge="start"
                  aria-label={`Time not in sync for approx. ${formatTime(provider.timeDiff)}`}
                  ref={anchorRef}
                  onClick={() => {
                    setOpenTimeButton(true);
                  }}
                >
                  <WatchLaterIcon sx={{ color: orange[500] }} />
                </IconButton>
                <Popper
                  sx={{
                    zIndex: 1,
                  }}
                  open={openTimeButton}
                  anchorEl={anchorRef.current}
                  transition
                  disablePortal
                >
                  {({ TransitionProps, placement }) => (
                    <Grow
                      {...TransitionProps}
                      style={{
                        transformOrigin: placement === "bottom" ? "center top" : "center bottom",
                      }}
                    >
                      <Paper>
                        <ClickAwayListener onClickAway={handleCloseTimeButton}>
                          <MenuList id="set-time-button-menu" autoFocusItem>
                            {optionsTimeButton.map((option, index) => (
                              <MenuItem
                                key={option}
                                // disabled={index === 2}
                                // selected={index === X}
                                onClick={(event) => handleMenuTimeItemClick(event, index)}
                              >
                                {option}
                              </MenuItem>
                            ))}
                          </MenuList>
                        </ClickAwayListener>
                      </Paper>
                    </Grow>
                  )}
                </Popper>
                <SetNTPDateDialog
                  key="sync-time-menu"
                  open={openNtpdateDialog}
                  onClose={(value: string) => {
                    if (value) {
                      // execute the command in own terminal
                      const id = `cmd-${generateUniqueId()}`;
                      emitCustomEvent(
                        EVENT_OPEN_COMPONENT,
                        eventOpenComponent(
                          id,
                          `${provider?.name()}`,
                          <SingleTerminalPanel id={id} type={CmdType.CMD} providerId={provider.id} cmd={value} />,
                          true,
                          LAYOUT_TAB_SETS.BORDER_BOTTOM
                        )
                      );
                    }
                    setOpenNtpdateDialog(false);
                  }}
                  defaultCmd="sudo ntpdate -v -u -t 1"
                />
                <DateHelpDialog
                  key="show-time-help"
                  open={showHelpTime}
                  onClose={() => {
                    setShowHelpTime(false);
                  }}
                />
              </Box>
            </Tooltip>
          )}
          {provider.isAvailable() ? (
            <ComputerIcon sx={{ mr: 0.5, width: 20, color: grey[700] }} />
          ) : (
            <HideSourceIcon sx={{ mr: 0.5, width: 20, color: red[700] }} />
          )}

          <Stack
            direction="row"
            sx={{ flexGrow: 1, userSelect: "none" }}
            // style={{ pointerEvents: "none" }}
          >
            {provider.name()}

            {getProviderTags(provider).map((tag: TTag) => (
              <Tooltip
                key={tag.id}
                sx={{ marginLeft: "0.1em" }}
                title={`${tag.tooltip}`}
                placement="left"
                disableInteractive
              >
                <Box>
                  {typeof tag.data === "string" ? (
                    <Tag
                      text={tag.data}
                      color={tag.color}
                      onClick={(event) => {
                        if (tag.onClick) {
                          tag.onClick(event);
                        }
                        event.stopPropagation();
                      }}
                      onDoubleClick={(event) => {
                        event.stopPropagation();
                      }}
                    />
                  ) : (
                    tag.data && <tag.data style={{ fontSize: "inherit", color: tag.color }} />
                  )}
                </Box>
              </Tooltip>
            ))}
          </Stack>
        </Box>
      }
      {...children}
    />
  );
});

export default HostItem;
