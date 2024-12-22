import CircleIcon from "@mui/icons-material/Circle";
import MuiContrastIcon from "@mui/icons-material/Contrast";
import { Box, Stack, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import PropTypes from "prop-types";
import { DiagnosticLevel, RosNodeStatus, getMaxDiagnosticLevel } from "../../models";
import ContentComponentItemTree from "../ContentComponentItemTree/ContentComponentItemTree";
import StyledTreeItem from "./StyledTreeItem";

const GroupStatus = {
  ALL_INACTIVE: 0,
  SOME_RUNNING: 1,
  ALL_RUNNING: 2,
};

const ContrastIcon = styled((props) => <MuiContrastIcon {...props} />)(() => ({
  transform: "rotate(-90deg)",
}));

const getGroupStatus = (treeItems) => {
  let groupStatus = GroupStatus.ALL_INACTIVE;
  let allRunning = true;
  treeItems.forEach((treeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenStatus = getGroupStatus(treeItem.children);
      if (childrenStatus !== GroupStatus.ALL_INACTIVE) {
        groupStatus = GroupStatus.SOME_RUNNING;
      }
      if (childrenStatus !== GroupStatus.ALL_RUNNING) {
        allRunning = false;
      }
    }
    if (treeItem.node && treeItem.node.status === RosNodeStatus.RUNNING) {
      groupStatus = GroupStatus.SOME_RUNNING;
    } else if (treeItem.node) {
      allRunning = false;
    }
  });
  if (groupStatus === GroupStatus.SOME_RUNNING && allRunning) {
    groupStatus = GroupStatus.ALL_RUNNING;
  }
  return groupStatus;
};

const getGroupDiagnosticLevel = (treeItems) => {
  let groupLevel = DiagnosticLevel.OK;
  treeItems.forEach((treeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenLevel = getGroupDiagnosticLevel(treeItem.children);
      groupLevel = getMaxDiagnosticLevel(groupLevel, childrenLevel);
    }
    if (treeItem.node) {
      groupLevel = getMaxDiagnosticLevel(groupLevel, treeItem.node.diagnosticLevel);
    }
  });
  return groupLevel;
};

const getGroupIconColor = (treeItems, isDarkMode = false) => {
  const groupStatus = getGroupStatus(treeItems);
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      switch (getGroupDiagnosticLevel(treeItems)) {
        case DiagnosticLevel.OK:
          return isDarkMode ? green[600] : green[500];
        case DiagnosticLevel.WARN:
          return isDarkMode ? orange[600] : orange[400];
        case DiagnosticLevel.ERROR:
          return isDarkMode ? red[600] : red[500];
        case DiagnosticLevel.STALE:
          return isDarkMode ? yellow[700] : yellow[500];
        default:
          return isDarkMode ? blue[600] : blue[500];
      }

    case GroupStatus.SOME_RUNNING:
      return isDarkMode ? green[700] : green[500];

    case GroupStatus.ALL_INACTIVE:
      return isDarkMode ? grey[600] : grey[500];

    default:
      return isDarkMode ? red[600] : red[500];
  }
};

export const getGroupIcon = (treeItems, isDarkMode = false) => {
  const groupStatus = getGroupStatus(treeItems);
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      return <CircleIcon style={{ mr: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    case GroupStatus.SOME_RUNNING:
      return <ContrastIcon style={{ mr: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    case GroupStatus.ALL_INACTIVE:
      return <CircleIcon style={{ mr: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    default:
      return <CircleIcon style={{ mr: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;
  }
};

/** Returns count of nodes for given group */
export const getNodesCount = (children) => {
  let result = 0;
  let itemsCount = 0;
  children.forEach((treeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      result += getNodesCount(treeItem.children);
    } else {
      itemsCount += 1;
    }
  });
  return result + itemsCount;
};

function GroupItem({ itemId, groupName, icon = <></>, countChildren = 0, onDoubleClick = () => {}, ...other }) {
  return (
    <StyledTreeItem
      slots={{ item: ContentComponentItemTree }}
      itemId={itemId}
      // onDoubleClick={(event) => onDoubleClick(event, groupName, itemId)}
      label={
        <Box display="flex" alignItems="center" paddingLeft={0.0}>
          {icon}
          <Stack
            direction="row"
            onDoubleClick={(event) => {
              if (onDoubleClick) {
                onDoubleClick(event, groupName, itemId);
                event.stopPropagation();
              }
            }}
            paddingLeft={0.5}
            flexGrow={1}
            sx={{ userSelect: "none" }}
          >
            {groupName}
          </Stack>

          {countChildren > 0 && <Typography variant="body2">[{countChildren}]</Typography>}
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

GroupItem.propTypes = {
  itemId: PropTypes.string.isRequired,
  groupName: PropTypes.string.isRequired,
  icon: PropTypes.object,
  countChildren: PropTypes.number,
  onDoubleClick: PropTypes.func,
};

export default GroupItem;
