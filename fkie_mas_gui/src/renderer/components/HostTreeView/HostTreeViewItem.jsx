import { alpha, styled } from '@mui/material/styles';
import { TreeItem, treeItemClasses } from '@mui/x-tree-view';
import PropTypes from 'prop-types';
import { useContext, useRef, useState } from 'react';

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
} from '@mui/material';

import DesktopAccessDisabledOutlinedIcon from '@mui/icons-material/DesktopAccessDisabledOutlined';
import DynamicFeedOutlinedIcon from '@mui/icons-material/DynamicFeedOutlined';
import PlayArrowOutlinedIcon from '@mui/icons-material/PlayArrowOutlined';
import RestartAltOutlinedIcon from '@mui/icons-material/RestartAltOutlined';
import StopOutlinedIcon from '@mui/icons-material/StopOutlined';
import WatchLaterIcon from '@mui/icons-material/WatchLater';
import { blue, grey, orange } from '@mui/material/colors';

import { SettingsContext } from '../../context/SettingsContext';
import ContentComponentItemTree from '../ContentComponentItemTree/ContentComponentItemTree';
import OverflowMenu from '../UI/OverflowMenu';
import Tag from '../UI/Tag';
import SetNTPDateDialog from './SetNTPDateDialog';

const StyledTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    minHeight: 25,
    borderRadius: theme.spacing(0.9),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    '&.Mui-expanded': {
      fontWeight: theme.typography.fontWeightRegular,
    },
    '&:hover': {
      backgroundColor: theme.palette.action.hover,
    },
    '&.Mui-selected': {
      // backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      // color: 'var(--tree-view-color)',
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: 'inherit',
      color: 'inherit',
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
  iconColor,
  bgColor,
  color,
  buttonIcon: ButtonIcon,
  buttonIconColor,
  buttonIconText,
  onButtonIconClick,
  timeSyncActive,
  timeSyncText,
  onTimeSync,
  labelIcon: LabelIcon,
  labelText,
  paddingLeft,
  provider,
  nodeId,
  onClick,
  onDoubleClick,
  menuItems,
  showMultipleScreen,
  showNoScreen,
  showGhostScreen,
  onStartClick,
  onStopClick,
  onRestartClick,
  startTooltipText,
  stopTooltipText,
  restartTooltipText,
  tags,
  countChildren,
  ...other
}) {
  const settingsCtx = useContext(SettingsContext);

  const optionsTimeButton = ['ntpdate', 'set date'];
  const [focus, setFocus] = useState(false);
  const [openTimeButton, setOpenTimeButton] = useState(false);
  const anchorRef = useRef(null);
  const [openNtpdateDialog, setOpenNtpdateDialog] = useState(false);

  const onHover = (event) => {
    setFocus(true);
    event.stopPropagation();
  };
  const onHoverOut = () => {
    setFocus(false);
  };

  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  const handleMenuTimeItemClick = (event, index) => {
    if (index === 0) {
      // set time using ntpdate
      setOpenNtpdateDialog(true);
    } else if (index === 1) {
      // set time using date
      if (provider) {
        onTimeSync(`sudo /bin/date -s ${new Date(Date.now()).toISOString()}`);
      }
    }
    setOpenTimeButton(false);
  };
  const handleCloseTimeButton = (event) => {
    if (anchorRef.current && anchorRef.current.contains(event.target)) {
      return;
    }
    setOpenTimeButton(false);
  };

  return (
    <StyledTreeItemRoot
      ContentComponent={ContentComponentItemTree}
      nodeId={nodeId}
      onDoubleClick={onDoubleClick}
      onMouseOver={onHover}
      onMouseOut={onHoverOut}
      label={
        <Box
          sx={{
            display: 'flex',
            alignItems: 'center',
            paddingLeft,
          }}
        >
          {menuItems && menuItems.length > 0 && (
            <OverflowMenu options={menuItems} id={`${nodeId}-options"`} />
          )}

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
                <ButtonIcon sx={{ color: buttonIconColor }} />
              </IconButton>
            </Tooltip>
          )}

          {timeSyncActive && (
            <Tooltip
              title={timeSyncText}
              placement="right-end"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
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
                        transformOrigin:
                          placement === 'bottom'
                            ? 'center top'
                            : 'center bottom',
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
                                onClick={(event) =>
                                  handleMenuTimeItemClick(event, index)
                                }
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
              </Box>
            </Tooltip>
          )}

          {LabelIcon && (
            <LabelIcon sx={{ mr: 0.5, width: 20, color: iconColor }} />
          )}

          <ListItem
            disablePadding
            component="div"
            secondaryAction={
              focus &&
              (onStartClick || onStopClick || onRestartClick) && (
                <Stack
                  sx={{
                    backgroundColor: settingsCtx.get('useDarkMode')
                      ? grey[600]
                      : blue[100],
                    borderRadius: '20px',
                  }}
                  direction="row"
                  spacing={0.1}
                  onClick={(event) => {
                    event.stopPropagation();
                  }}
                >
                  <Tooltip
                    title={startTooltipText}
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={startTooltipText}
                        onClick={() => {
                          onStartClick(nodeId);
                        }}
                        disabled={!onStartClick}
                      >
                        <PlayArrowOutlinedIcon
                          style={{ fontSize: 'inherit', pointerEvents: 'none' }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>

                  <Tooltip
                    title={stopTooltipText}
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={stopTooltipText}
                        onClick={() => {
                          onStopClick(nodeId);
                        }}
                        disabled={!onStopClick}
                      >
                        <StopOutlinedIcon
                          style={{ fontSize: 'inherit', pointerEvents: 'none' }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>

                  <Tooltip
                    title={restartTooltipText}
                    enterDelay={tooltipDelay}
                    enterNextDelay={tooltipDelay}
                  >
                    <Box>
                      <IconButton
                        edge="end"
                        aria-label={restartTooltipText}
                        onClick={() => {
                          onRestartClick(nodeId);
                        }}
                        disabled={!onRestartClick}
                      >
                        <RestartAltOutlinedIcon
                          style={{ fontSize: 'inherit', pointerEvents: 'none' }}
                        />
                      </IconButton>
                    </Box>
                  </Tooltip>
                </Stack>
              )
            }
          >
            <Typography
              onClick={onClick}
              onDoubleClick={() => {
                if (onDoubleClick) onDoubleClick(labelText, nodeId);
              }}
              variant="body2"
              sx={{ fontWeight: 'inherit', flexGrow: 1 }}
            >
              {labelText}
            </Typography>
            {(!focus || (!onStartClick && !onStopClick && !onRestartClick)) &&
              tags.map((tag) => (
                <Tag
                  key={tag.text}
                  text={tag.text}
                  color={tag.color}
                  style={{ pointerEvents: 'none' }}
                />
              ))}
          </ListItem>

          {countChildren > 0 && (
            <Typography variant="body2" sx={{ fontWeight: 'inherit' }}>
              [{countChildren}]
            </Typography>
          )}
          {showMultipleScreen && (
            <Tooltip title="Multiple Screens" placement="left">
              <DynamicFeedOutlinedIcon
                color="warning"
                style={{ fontSize: 'inherit' }}
              />
            </Tooltip>
          )}
          {showNoScreen && (
            <Tooltip title="No Screens" placement="left">
              <DesktopAccessDisabledOutlinedIcon
                style={{ fontSize: 'inherit' }}
              />
            </Tooltip>
          )}
          {showGhostScreen && (
            <Tooltip title="Ghost Screens" placement="left">
              <DynamicFeedOutlinedIcon
                color="warning"
                style={{ fontSize: 'inherit' }}
              />
            </Tooltip>
          )}
        </Box>
      }
      style={{
        '--tree-view-color': color,
        '--tree-view-bg-color': bgColor,
      }}
      {...other}
    />
  );
}

HostTreeViewItem.defaultProps = {
  nodeId: '',
  iconColor: '',
  onClick: () => {},
  onDoubleClick: () => {},
  menuItems: [],
  buttonIcon: null,
  buttonIconColor: '',
  buttonIconText: '',
  onButtonIconClick: () => {},
  timeSyncActive: false,
  timeSyncText: '',
  onTimeSync: () => {},
  labelIcon: null,
  showMultipleScreen: false,
  showNoScreen: false,
  showGhostScreen: false,
  onStartClick: null,
  onStopClick: null,
  onRestartClick: null,
  startTooltipText: '',
  stopTooltipText: '',
  restartTooltipText: '',
  tags: [],
  provider: null,
  countChildren: 0,
};

HostTreeViewItem.propTypes = {
  nodeId: PropTypes.string,
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
  showMultipleScreen: PropTypes.bool,
  showNoScreen: PropTypes.bool,
  showGhostScreen: PropTypes.bool,
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
