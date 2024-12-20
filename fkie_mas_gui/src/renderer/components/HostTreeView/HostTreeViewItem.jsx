import DesktopAccessDisabledOutlinedIcon from "@mui/icons-material/DesktopAccessDisabledOutlined";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import StopIcon from "@mui/icons-material/Stop";
import WatchLaterIcon from "@mui/icons-material/WatchLater";
import {
  Box,
  ClickAwayListener,
  Grow,
  IconButton,
  ListItem,
  MenuItem,
  MenuList,
  Paper,
  Popper,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { blue, grey, orange } from "@mui/material/colors";
import { alpha, styled } from "@mui/material/styles";
import { TreeItem, treeItemClasses } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import { useContext, useEffect, useRef, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { CmdType } from "../../providers";
import ContentComponentItemTree from "../ContentComponentItemTree/ContentComponentItemTree";
import OverflowMenu from "../UI/OverflowMenu";
import Tag from "../UI/Tag";
import DateHelpDialog from "./DateHelpDialog";
import SetNTPDateDialog from "./SetNTPDateDialog";

const StyledTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    minHeight: 25,
    borderRadius: theme.spacing(0.9),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    "&.Mui-expanded": {
      fontWeight: theme.typography.fontWeightRegular,
    },
    "&:hover": {
      backgroundColor: theme.palette.action.hover,
    },
    "&.Mui-selected": {
      // backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      // color: 'var(--tree-view-color)',
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: "inherit",
      color: "inherit",
      padding: theme.spacing(0),
    },
    [`& .${treeItemClasses.iconContainer}`]: {
      marginLeft: 0,
      marginRight: 0,
      padding: theme.spacing(0),
      width: 18,
    },
  },
  [`& .${treeItemClasses.group}`]: {
    marginLeft: 16,
    paddingLeft: 5,
    // [`& .${treeItemClasses.content}`]: {
    //   paddingLeft: theme.spacing(0),
    // },
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
    borderColor: grey[800],
  },
}));

function HostTreeViewItem({
  bgColor,
  color,
  labelText,
  paddingLeft,
  itemId = "",
  isNode = false,
  namespacePart = "",
  iconColor = "",
  onClick = () => {},
  onDoubleClick = () => {},
  menuItems = [],
  buttonIcon: ButtonIcon = null,
  buttonIconColor = "",
  buttonIconText = "",
  onButtonIconClick = () => {},
  timeSyncActive = false,
  timeSyncText = "",
  onTimeSync = () => {},
  labelIcon: LabelIcon = null,
  showMultipleScreen = false,
  showNoScreen = false,
  showGhostScreen = false,
  showLoggers = false,
  onShowLoggersClick = null,
  onStartClick = null,
  onStopClick = null,
  onRestartClick = null,
  startTooltipText = "",
  stopTooltipText = "",
  restartTooltipText = "",
  tags = [],
  provider = null,
  countChildren = 0,
  ...other
}) {
  const settingsCtx = useContext(SettingsContext);
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);

  const optionsTimeButton = ["ntpdate", "set date", "sync me to this date", "help"];
  const [focus, setFocus] = useState(false);
  const [openTimeButton, setOpenTimeButton] = useState(false);
  const anchorRef = useRef(null);
  const [showHelpTime, setShowHelpTime] = useState(false);
  const [openNtpdateDialog, setOpenNtpdateDialog] = useState(false);
  const [showFloatingButtons, setShowFloatingButtons] = useState(settingsCtx.get("showFloatingButtons"));

  const onHover = (event) => {
    setFocus(true);
    event.stopPropagation();
  };
  const onHoverOut = () => {
    setFocus(false);
  };

  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const updateTime = async (local = true) => {
    if (provider) {
      const localProviders = rosCtx.getLocalProvider();
      if (localProviders.length === 0) {
        logCtx.error("localhost provider not found", "", true);
        return;
      }
      if (local) {
        rosCtx.openTerminal(
          CmdType.SET_TIME,
          localProviders[0].id,
          `set_date_localhost-${Date.now()}`,
          "",
          provider.id,
          false,
          false
        );
      } else {
        rosCtx.openTerminal(
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
  };

  const handleMenuTimeItemClick = (event, index) => {
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
  };
  const handleCloseTimeButton = (event) => {
    if (anchorRef.current && anchorRef.current.contains(event.target)) {
      return;
    }
    setOpenTimeButton(false);
  };

  useEffect(() => {
    setShowFloatingButtons(settingsCtx.get("showFloatingButtons"));
  }, [settingsCtx, settingsCtx.changed]);

  return (
    <StyledTreeItemRoot
      // ContentComponent={ContentComponentItemTree}
      slots={{ item: ContentComponentItemTree }}
      itemId={itemId}
      // onDoubleClick={(event) => onDoubleClick(event, labelText, itemId)}
      onMouseOver={onHover}
      onMouseOut={onHoverOut}
      label={
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            paddingLeft,
          }}
        >
          {menuItems && menuItems.length > 0 && <OverflowMenu options={menuItems} id={`${itemId}-options"`} />}

          {ButtonIcon && (
            <Tooltip
              title={buttonIconText}
              placement="bottom-start"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
              <IconButton
                edge="start"
                aria-label={buttonIconText}
                onClick={() => {
                  onButtonIconClick();
                }}
              >
                <ButtonIcon sx={{ color: buttonIconColor, fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )}

          {timeSyncActive && (
            <Tooltip title={timeSyncText} placement="right-end">
              <Box>
                <IconButton
                  edge="start"
                  aria-label={timeSyncText}
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
                  role={undefined}
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
                  id="sync-time-menu"
                  keepMounted
                  open={openNtpdateDialog}
                  onClose={(value) => {
                    if (value) {
                      onTimeSync(`${value}`);
                    }
                    setOpenNtpdateDialog(false);
                  }}
                  value="sudo ntpdate -v -u -t 1"
                />
                <DateHelpDialog
                  id="show-time-help"
                  open={showHelpTime}
                  onClose={() => {
                    setShowHelpTime(false);
                  }}
                />
              </Box>
            </Tooltip>
          )}

          {LabelIcon && <LabelIcon sx={{ mr: 0.5, width: 20, color: iconColor }} />}

          <ListItem
            disablePadding
            component="div"
            secondaryAction={
              focus &&
              showFloatingButtons &&
              (onStartClick || onStopClick || onRestartClick) && (
                <Stack
                  sx={{
                    backgroundColor: settingsCtx.get("useDarkMode") ? grey[600] : blue[100],
                    borderRadius: "20px",
                  }}
                  direction="row"
                  spacing={0.1}
                  onClick={(event) => {
                    event.stopPropagation();
                  }}
                >
                  <Tooltip title={startTooltipText} enterDelay={tooltipDelay} enterNextDelay={tooltipDelay}>
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={startTooltipText}
                        onClick={() => {
                          onStartClick(itemId);
                        }}
                        disabled={!onStartClick}
                      >
                        <PlayArrowIcon
                          style={{
                            fontSize: "inherit",
                            pointerEvents: "none",
                          }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>

                  <Tooltip title={stopTooltipText} enterDelay={tooltipDelay} enterNextDelay={tooltipDelay}>
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={stopTooltipText}
                        onClick={() => {
                          onStopClick(itemId);
                        }}
                        disabled={!onStopClick}
                      >
                        <StopIcon
                          style={{
                            fontSize: "inherit",
                            pointerEvents: "none",
                          }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>

                  <Tooltip title={restartTooltipText} enterDelay={tooltipDelay} enterNextDelay={tooltipDelay}>
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={restartTooltipText}
                        onClick={() => {
                          onRestartClick(itemId);
                        }}
                        disabled={!onRestartClick}
                      >
                        <RestartAltIcon
                          style={{
                            fontSize: "inherit",
                            pointerEvents: "none",
                          }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>
                </Stack>
              )
            }
          >
            <Stack
              direction="row"
              onClick={onClick}
              onDoubleClick={(event) => {
                if (onDoubleClick) {
                  onDoubleClick(event, labelText, itemId);
                  event.stopPropagation();
                }
              }}
              sx={{ flexGrow: 1, userSelect: "none" }}
            >
              {isNode ? (
                <>
                  <Typography variant="body2" sx={{ fontSize: "inherit", userSelect: "none" }}>
                    {namespacePart}
                  </Typography>
                  <Typography variant="body2" sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}>
                    {labelText.slice(namespacePart.length)}
                  </Typography>
                </>
              ) : (
                labelText
              )}
            </Stack>
            {(!focus || !showFloatingButtons || (!onStartClick && !onStopClick && !onRestartClick)) &&
              tags.map((tag) => (
                <Tooltip
                  key={tag.id}
                  title={`${tag.tooltip}`}
                  placement="left"
                  disableInteractive
                  onClick={tag.onClick ? (event) => tag.onClick(event) : () => {}}
                >
                  {typeof tag.data === "string" ? (
                    <div>
                      <Tag text={tag.data} color={tag.color} style={{ pointerEvents: "none" }} />
                    </div>
                  ) : (
                    tag.data && <tag.data style={{ fontSize: "inherit", color: tag.color }} />
                  )}
                </Tooltip>
              ))}
          </ListItem>

          {countChildren > 0 && (
            <Typography variant="body2" sx={{ fontWeight: "inherit" }}>
              [{countChildren}]
            </Typography>
          )}
          {showLoggers && (
            <Tooltip title="User changed logging level" placement="left">
              <IconButton
                size="small"
                onClick={() => {
                  if (onShowLoggersClick) {
                    onShowLoggersClick(itemId);
                  }
                }}
              >
                <SettingsInputCompositeOutlinedIcon style={{ fontSize: "inherit", rotate: "90deg" }} />
              </IconButton>
            </Tooltip>
          )}
          {showMultipleScreen && (
            <Tooltip title="Multiple Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {showNoScreen && (
            <Tooltip title="No Screens" placement="left">
              <DesktopAccessDisabledOutlinedIcon style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {showGhostScreen && (
            <Tooltip title="Ghost Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
        </Box>
      }
      style={{
        "--tree-view-color": color,
        "--tree-view-bg-color": bgColor,
      }}
      {...other}
    />
  );
}

HostTreeViewItem.propTypes = {
  itemId: PropTypes.string,
  isNode: PropTypes.bool,
  namespacePart: PropTypes.string,
  iconColor: PropTypes.string,
  bgColor: PropTypes.string.isRequired,
  color: PropTypes.string.isRequired,
  buttonIcon: PropTypes.elementType,
  buttonIconColor: PropTypes.string,
  buttonIconText: PropTypes.string,
  onButtonIconClick: PropTypes.func,
  timeSyncActive: PropTypes.bool,
  timeSyncText: PropTypes.string,
  onTimeSync: PropTypes.func,
  labelIcon: PropTypes.elementType,
  labelText: PropTypes.string.isRequired,
  paddingLeft: PropTypes.number.isRequired,
  provider: PropTypes.any,
  onClick: PropTypes.func,
  onDoubleClick: PropTypes.func,
  menuItems: PropTypes.arrayOf(PropTypes.any),
  showLoggers: PropTypes.bool,
  showMultipleScreen: PropTypes.bool,
  showNoScreen: PropTypes.bool,
  showGhostScreen: PropTypes.bool,
  onShowLoggersClick: PropTypes.func,
  onStartClick: PropTypes.func,
  onStopClick: PropTypes.func,
  onRestartClick: PropTypes.func,
  startTooltipText: PropTypes.string,
  stopTooltipText: PropTypes.string,
  restartTooltipText: PropTypes.string,
  tags: PropTypes.array,
  countChildren: PropTypes.number,
};

export default HostTreeViewItem;
