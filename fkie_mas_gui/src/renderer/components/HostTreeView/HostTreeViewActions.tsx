import { ButtonGroup, Divider, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import React from "react";

import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import AddToQueueIcon from "@mui/icons-material/AddToQueue";
import BorderColorIcon from "@mui/icons-material/BorderColor";
import CancelPresentationIcon from "@mui/icons-material/CancelPresentation";
import DangerousOutlinedIcon from "@mui/icons-material/DangerousOutlined";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import DeleteSweepIcon from "@mui/icons-material/DeleteSweep";
import DvrIcon from "@mui/icons-material/Dvr";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import SettingsSuggestIcon from "@mui/icons-material/SettingsSuggest";
import StopIcon from "@mui/icons-material/Stop";
import TerminalIcon from "@mui/icons-material/Terminal";
import TuneIcon from "@mui/icons-material/Tune";
import WysiwygIcon from "@mui/icons-material/Wysiwyg";

export interface HostTreeViewActionsProps {
  selectedNodesCount: number;
  hasDynamicReconfigure: boolean;
  canUnregisterSelectedNodes: boolean;
  tooltipDelay: number;
  showButtonsForKeyModifiers: boolean;

  onStartClick: (options: { ignoreTimer: boolean }) => void;
  onStopClick: (options: { kill: boolean; unregister: boolean }) => void;
  onRestartClick: (options: { ignoreTimer: boolean }) => void;
  onKillClick: () => void;
  onUnregisterClick: () => void;
  onEditClick: (options: { external: boolean }) => void;
  onParametersClick: () => void;
  onDynamicReconfigureClick: () => void;
  onScreensClick: (options: { external: boolean; openInTerminal: boolean }) => void;
  onLogsClick: (options: { external: boolean; openInTerminal: boolean }) => void;
  onLoggersClick: () => void;
  onClearLogsClick: () => void;
  onOpenTerminalOnHostsClick: (options: { external: boolean; openInTerminal: boolean }) => void;
  onShutdownRosClick: (options: { killRos2: boolean }) => void;
}

/**
 * Pure presentational component for the vertical button group.
 * All stateful logic stays in the parent and is injected via props.
 */
const HostTreeViewActions: React.FC<HostTreeViewActionsProps> = (props) => {
  const {
    selectedNodesCount,
    hasDynamicReconfigure,
    canUnregisterSelectedNodes,
    tooltipDelay,
    showButtonsForKeyModifiers,
    onStartClick,
    onStopClick,
    onRestartClick,
    onKillClick,
    onUnregisterClick,
    onEditClick,
    onParametersClick,
    onDynamicReconfigureClick,
    onScreensClick,
    onLogsClick,
    onLoggersClick,
    onClearLogsClick,
    onOpenTerminalOnHostsClick,
    onShutdownRosClick,
  } = props;

  const hasNodeSelection = selectedNodesCount > 0;

  const navCtx = useNavigationContext();
  const hasSelectedProviders = navCtx.selection.selectedProviders.length > 0;
  const hasNodesOrProvidersSelection = hasNodeSelection || hasSelectedProviders;

  const handleStart = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onStartClick({ ignoreTimer: event.nativeEvent.shiftKey });
  };

  const handleStop = (event: React.MouseEvent<HTMLButtonElement>): void => {
    const isShift = event.nativeEvent.shiftKey;
    const isCtrl = event.nativeEvent.ctrlKey;

    onStopClick({
      kill: isShift,
      unregister: isShift && isCtrl,
    });
  };

  const handleRestart = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onRestartClick({ ignoreTimer: event.nativeEvent.shiftKey });
  };

  const handleEdit = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onEditClick({ external: event.nativeEvent.shiftKey });
  };

  const handleScreens = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onScreensClick({
      external: event.nativeEvent.shiftKey,
      openInTerminal: event.nativeEvent.ctrlKey,
    });
  };

  const handleLogs = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onLogsClick({
      external: event.nativeEvent.shiftKey,
      openInTerminal: event.nativeEvent.ctrlKey,
    });
  };

  const handleOpenTerminalOnHosts = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onOpenTerminalOnHostsClick({
      external: event.nativeEvent.shiftKey,
      openInTerminal: event.nativeEvent.ctrlKey,
    });
  };

  const handleShutdownRos = (event: React.MouseEvent<HTMLButtonElement>): void => {
    onShutdownRosClick({
      killRos2: event.nativeEvent.shiftKey,
    });
  };

  return (
    <ButtonGroup orientation="vertical" aria-label="ros node control group">
      {/* Start */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Start selected nodes
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift:
              </Typography>
              <Typography fontSize="inherit">ignore start timer</Typography>
            </Stack>
          </div>
        }
        placement="left"
        enterDelay={tooltipDelay}
        enterNextDelay={tooltipDelay}
        disableInteractive
      >
        <span>
          <IconButton size="medium" aria-label="Start" onClick={handleStart} disabled={!hasNodeSelection}>
            <PlayArrowIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Stop / Kill / Unregister via modifiers */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Stop selected nodes
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift:
              </Typography>
              <Typography fontSize="inherit">kill</Typography>
            </Stack>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Ctrl+Shift:
              </Typography>
              <Typography fontSize="inherit">Unregister ROS1 nodes</Typography>
            </Stack>
          </div>
        }
        placement="left"
        enterDelay={tooltipDelay}
        enterNextDelay={tooltipDelay}
        disableInteractive
      >
        <span>
          <IconButton size="medium" aria-label="Stop" onClick={handleStop} disabled={!hasNodeSelection}>
            <StopIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Restart */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Restart selected nodes
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift:
              </Typography>
              <Typography fontSize="inherit">ignore start timer</Typography>
            </Stack>
          </div>
        }
        placement="left"
        enterDelay={tooltipDelay}
        enterNextDelay={tooltipDelay}
        disableInteractive
      >
        <span>
          <IconButton size="medium" aria-label="Restart" onClick={handleRestart} disabled={!hasNodeSelection}>
            <RestartAltIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      <Divider />

      {/* Explicit Kill / Unregister buttons when configured */}
      {showButtonsForKeyModifiers && (
        <Stack>
          <Tooltip title="Kill" placement="left" disableInteractive>
            <span>
              <IconButton size="medium" aria-label="Kill" onClick={onKillClick} disabled={!hasNodeSelection}>
                <CancelPresentationIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
          <Tooltip title="Unregister ROS1 nodes" placement="left" disableInteractive>
            <span>
              <IconButton
                size="medium"
                aria-label="Unregister"
                onClick={onUnregisterClick}
                disabled={!canUnregisterSelectedNodes}
              >
                <DeleteForeverIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
          <Divider />
        </Stack>
      )}

      {/* Edit */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Edit
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift+click:
              </Typography>
              <Typography fontSize="inherit">alternative open location</Typography>
            </Stack>
          </div>
        }
        placement="left"
        enterDelay={tooltipDelay}
        enterNextDelay={tooltipDelay}
        disableInteractive
      >
        <span>
          <IconButton size="medium" aria-label="Edit" onClick={handleEdit} disabled={!hasNodeSelection}>
            <BorderColorIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Parameters */}
      <Tooltip title="Parameters" placement="left" disableInteractive>
        <span>
          <IconButton
            size="medium"
            aria-label="Parameters"
            onClick={onParametersClick}
            disabled={!hasNodesOrProvidersSelection}
          >
            <TuneIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Dynamic reconfigure */}
      {hasDynamicReconfigure && (
        <Tooltip title="Dynamic reconfigure for ROS1 nodes" placement="left" disableInteractive>
          <span>
            <IconButton size="medium" aria-label="dynamic reconfigure" onClick={onDynamicReconfigureClick}>
              <SettingsSuggestIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
      )}

      <Divider />

      {/* Screens */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Open screen of the node
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift+click:
              </Typography>
              <Typography fontSize="inherit">alternative open location</Typography>
            </Stack>
          </div>
        }
        placement="left"
        disableInteractive
      >
        <span>
          <IconButton
            size="medium"
            aria-label="Screen"
            disabled={!hasNodesOrProvidersSelection}
            onClick={handleScreens}
          >
            {hasSelectedProviders ? <AddToQueueIcon fontSize="inherit" /> : <DvrIcon fontSize="inherit" />}
          </IconButton>
        </span>
      </Tooltip>

      {/* Logs */}
      <Tooltip
        title={
          <div>
            <Typography fontWeight="bold" fontSize="inherit">
              Open log of the node
            </Typography>
            <Stack direction="row" spacing={"0.2em"}>
              <Typography fontWeight="bold" fontSize="inherit">
                Shift+click:
              </Typography>
              <Typography fontSize="inherit">alternative open location</Typography>
            </Stack>
          </div>
        }
        placement="left"
        disableInteractive
      >
        <span>
          <IconButton size="medium" aria-label="Log" disabled={!hasNodesOrProvidersSelection} onClick={handleLogs}>
            <WysiwygIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Log level (logger) */}
      <Tooltip title="Change log level" placement="left" disableInteractive>
        <span>
          <IconButton size="medium" aria-label="Log Level" disabled={!hasNodeSelection} onClick={onLoggersClick}>
            <SettingsInputCompositeOutlinedIcon fontSize="inherit" sx={{ rotate: "90deg" }} />
          </IconButton>
        </span>
      </Tooltip>

      {/* Clear logs / rosclean purge */}
      <Tooltip title={hasSelectedProviders ? "ros clean purge" : "Clear Logs"} placement="left" disableInteractive>
        <span>
          <IconButton
            size="medium"
            aria-label="Clear Logs"
            disabled={!hasNodesOrProvidersSelection}
            onClick={onClearLogsClick}
          >
            <DeleteSweepIcon fontSize="inherit" />
          </IconButton>
        </span>
      </Tooltip>

      {/* Terminal on selected hosts */}
      {hasSelectedProviders && <Divider />}
      {hasSelectedProviders && (
        <Tooltip
          title="Open Terminal on selected host (external terminal with shift+click)"
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="medium"
              aria-label="Open Terminal on selected host"
              disabled={!hasSelectedProviders}
              onClick={handleOpenTerminalOnHosts}
            >
              <TerminalIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
      )}
      {navCtx.selection.selectedProviders.length === 1 && (
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                First, a SIGTERM is sent to all nodes started via MAS. After that, a SIGKILL is sent to all screen child
                processes.
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight="bold" fontSize="inherit">
                  Shift:
                </Typography>
                <Typography fontSize="inherit">Terminate all ros2 processes</Typography>
              </Stack>
            </div>
          }
          placement="left"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <IconButton
            size="medium"
            aria-label="Stop all screen and ros2 nodes on selected host"
            onClick={handleShutdownRos}
          >
            <DangerousOutlinedIcon fontSize="inherit" />
          </IconButton>
        </Tooltip>
      )}
    </ButtonGroup>
  );
};

export default React.memo(HostTreeViewActions);
