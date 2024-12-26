import { nameWithoutNamespace } from "@/renderer/utils";
import CircleIcon from "@mui/icons-material/Circle";
import DesktopAccessDisabledOutlinedIcon from "@mui/icons-material/DesktopAccessDisabledOutlined";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
import NewReleasesTwoToneIcon from "@mui/icons-material/NewReleasesTwoTone";
import ReportIcon from "@mui/icons-material/Report";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import WarningIcon from "@mui/icons-material/Warning";
import { Box, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import PropTypes from "prop-types";
import { useContext, useEffect, useState } from "react";
import { SettingsContext } from "../../context/SettingsContext";
import { DiagnosticLevel, RosNodeStatus } from "../../models";
import ContentComponentItemTree from "../ContentComponentItemTree/ContentComponentItemTree";
import Tag from "../UI/Tag";
import StyledTreeItem from "./StyledTreeItem";

function NodeItem({
  itemId = "",
  node,
  namespacePart = "",
  onDoubleClick = () => {},
  onShowLoggersClick = () => {},
  ...other
}) {
  const settingsCtx = useContext(SettingsContext);
  const [labelText, setLabelText] = useState(nameWithoutNamespace(node));

  const getColorFromDiagnostic = (diagnosticLevel, isDarkMode = false) => {
    switch (diagnosticLevel) {
      case DiagnosticLevel.OK:
        return isDarkMode ? green[600] : green[500];
      case DiagnosticLevel.WARN:
        return isDarkMode ? orange[600] : orange[400];
      case DiagnosticLevel.ERROR:
        return isDarkMode ? red[600] : red[500];
      case DiagnosticLevel.STALE:
        return isDarkMode ? yellow[700] : yellow[600];
      default:
        return isDarkMode ? blue[600] : blue[500];
    }
  };

  const getNodeIcon = (node, isDarkMode = false) => {
    switch (node.status) {
      case RosNodeStatus.RUNNING: {
        const color = getColorFromDiagnostic(node.diagnosticLevel, isDarkMode);
        return <CircleIcon style={{ mr: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.DEAD: {
        const color = isDarkMode ? orange[600] : orange[400];
        return <WarningIcon style={{ mr: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.NOT_MONITORED: {
        const color = isDarkMode ? blue[700] : blue[500];
        return <ReportIcon style={{ mr: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.INACTIVE: {
        const color = isDarkMode ? grey[600] : grey[500];
        let cmdWithRos2 = null;
        node.launchInfo.forEach((launchInfo) => {
          if (launchInfo.cmd?.includes("ros2 run")) {
            cmdWithRos2 = launchInfo;
          }
        })
        if (cmdWithRos2) {
          return (
            <Tooltip
              key={`icon-${node.id}`}
              title={`Executable '${cmdWithRos2.executable}' or package '${cmdWithRos2.package_name}' not found`}
              placement="left"
              disableInteractive
            >
              <NewReleasesTwoToneIcon style={{ mr: 0.5, width: 20, color: color }} />
            </Tooltip>
          );
        }
        return <CircleIcon style={{ mr: 0.5, width: 20, color: color }} />;
      }

      default: {
        const color = isDarkMode ? red[600] : red[500];
        return <CircleIcon style={{ mr: 0.5, width: 20, color: color }} />;
      }
    }
  };

  const [isDarkMode, setIsDarkMode] = useState(settingsCtx.get("useDarkMode"));
  const [nodeIcon, setNodeIcon] = useState(getNodeIcon(node, isDarkMode));

  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode"));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    setNodeIcon(getNodeIcon(node, isDarkMode));
    setLabelText(nameWithoutNamespace(node));
  }, [node, isDarkMode]);

  return (
    <StyledTreeItem
      // ContentComponent={ContentComponentItemTree}
      slots={{ item: ContentComponentItemTree }}
      itemId={itemId}
      // onDoubleClick={(event) => onDoubleClick(event, labelText, itemId)}
      label={
        <Box display="flex" alignItems="center" paddingLeft={0.0}>
          {nodeIcon}

          <Stack
            direction="row"
            // onClick={onClick}
            onDoubleClick={(event) => {
              if (onDoubleClick) {
                onDoubleClick(event, labelText, itemId);
                event.stopPropagation();
              }
            }}
            flexGrow="1"
            paddingLeft={0.5}
            sx={{ userSelect: "none" }}
          >
            <Typography variant="body2" sx={{ fontSize: "inherit", userSelect: "none" }}>
              {namespacePart}
            </Typography>
            <Typography variant="body2" sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}>
              {labelText}
            </Typography>
          </Stack>
          {node.tags.map((tag) => (
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
          {node.getRosLoggersCount() > 0 && (
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
          {node.status === RosNodeStatus.RUNNING && node.screens?.length > 1 && (
            <Tooltip title="Multiple Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status === RosNodeStatus.RUNNING && node.screens?.length < 1 && (
            <Tooltip title="No Screens" placement="left">
              <DesktopAccessDisabledOutlinedIcon style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status !== RosNodeStatus.RUNNING && node.screens?.length > 0 && (
            <Tooltip title="Ghost Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
        </Box>
      }
      style={{
        "--tree-view-color": blue[700],
        "--tree-view-bg-color": blue[200],
      }}
      {...other}
    />
  );
}

NodeItem.propTypes = {
  itemId: PropTypes.string.isRequired,
  node: PropTypes.object.isRequired,
  namespacePart: PropTypes.string,
  onDoubleClick: PropTypes.func,
  onShowLoggersClick: PropTypes.func,
};

export default NodeItem;
