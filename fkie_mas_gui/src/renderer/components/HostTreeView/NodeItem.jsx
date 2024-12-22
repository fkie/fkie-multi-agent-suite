import CircleIcon from "@mui/icons-material/Circle";
import DesktopAccessDisabledOutlinedIcon from "@mui/icons-material/DesktopAccessDisabledOutlined";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
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
  const [labelText, setLabelText] = useState(node.name[0] === "/" ? node.name.slice(1) : node.name);

  const getNodeIconColor = (node, isDarkMode = false) => {
    switch (node.status) {
      case RosNodeStatus.RUNNING:
        switch (node.diagnosticLevel) {
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
      case RosNodeStatus.DEAD:
        return isDarkMode ? orange[600] : orange[400];

      case RosNodeStatus.NOT_MONITORED:
        return isDarkMode ? blue[700] : blue[500];

      case RosNodeStatus.INACTIVE:
        return isDarkMode ? grey[600] : grey[500];

      default:
        return isDarkMode ? red[600] : red[500];
    }
  };

  const getNodeIcon = (node, isDarkMode = false) => {
    switch (node.status) {
      case RosNodeStatus.RUNNING:
        return <CircleIcon style={{ mr: 0.5, width: 20, color: getNodeIconColor(node, isDarkMode) }} />;

      case RosNodeStatus.DEAD:
        return <WarningIcon style={{ mr: 0.5, width: 20, color: getNodeIconColor(node, isDarkMode) }} />;

      case RosNodeStatus.NOT_MONITORED:
        return <ReportIcon style={{ mr: 0.5, width: 20, color: getNodeIconColor(node, isDarkMode) }} />;

      case RosNodeStatus.INACTIVE:
        return <CircleIcon style={{ mr: 0.5, width: 20, color: getNodeIconColor(node, isDarkMode) }} />;

      default:
        return <CircleIcon style={{ mr: 0.5, width: 20, color: getNodeIconColor(node, isDarkMode) }} />;
    }
  };

  const [isDarkMode, setIsDarkMode] = useState(settingsCtx.get("useDarkMode"));
  const [nodeIcon, setNodeIcon] = useState(getNodeIcon(node, isDarkMode));

  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode"));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    setNodeIcon(getNodeIcon(node, isDarkMode));
    setLabelText(node.name[0] === "/" ? node.name.slice(1) : node.name);
  }, [node, isDarkMode]);

  // useEffect(() => {
  //   const sepIdx = labelText.lastIndexOf("/");
  //   if (sepIdx >= 0) {
  //     setNamespacePart(labelText.substring(0, sepIdx));
  //   } else {
  //     setNamespacePart("");
  //   }
  // }, [labelText]);

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
              {labelText.slice(namespacePart.length)}
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
