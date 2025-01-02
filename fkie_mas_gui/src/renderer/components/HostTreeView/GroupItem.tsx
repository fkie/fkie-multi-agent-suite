import CircleIcon from "@mui/icons-material/Circle";
import MuiContrastIcon from "@mui/icons-material/Contrast";
import { Box, Stack, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { styled } from "@mui/material/styles";
import { SvgIconProps } from "@mui/material/SvgIcon";
import { TreeItem2SlotProps } from "@mui/x-tree-view/TreeItem2";
import { UseTreeItem2ContentSlotOwnProps } from "@mui/x-tree-view/useTreeItem2";
import { UseTreeItem2IconContainerSlotOwnProps } from "@mui/x-tree-view/useTreeItem2/useTreeItem2.types";
import { forwardRef } from "react";
import { DiagnosticLevel, getMaxDiagnosticLevel, RosNodeStatus } from "../../models";
import StyledTreeItem from "./StyledTreeItem";
import { NodeTreeItem } from "./types";

const GroupStatus = {
  ALL_INACTIVE: 0,
  SOME_RUNNING: 1,
  ALL_RUNNING: 2,
};

const ContrastIcon = styled((props: SvgIconProps) => <MuiContrastIcon {...props} />)(() => ({
  transform: "rotate(-90deg)",
}));

const getGroupStatus: (treeItems: NodeTreeItem[]) => number = (treeItems) => {
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

const getGroupDiagnosticLevel: (treeItems: NodeTreeItem[]) => DiagnosticLevel = (treeItems) => {
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

const getGroupIconColor = (treeItems: NodeTreeItem[], isDarkMode: boolean = false) => {
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

export const GroupIcon = (treeItems: NodeTreeItem[], isDarkMode = false) => {
  const groupStatus = getGroupStatus(treeItems);
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      return <CircleIcon style={{ marginRight: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    case GroupStatus.SOME_RUNNING:
      return <ContrastIcon style={{ marginRight: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    case GroupStatus.ALL_INACTIVE:
      return <CircleIcon style={{ marginRight: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;

    default:
      return <CircleIcon style={{ marginRight: 0.5, width: 20, color: getGroupIconColor(treeItems, isDarkMode) }} />;
  }
};

/** Returns count of nodes for given group */
export const NodesCount = (children: NodeTreeItem[]) => {
  let result = 0;
  let itemsCount = 0;
  children.forEach((treeItem: NodeTreeItem) => {
    if (treeItem.children && treeItem.children.length > 0) {
      result += NodesCount(treeItem.children);
    } else {
      itemsCount += 1;
    }
  });
  return result + itemsCount;
};

interface GroupItemProps {
  itemId: string;
  groupName: string;
  icon: React.ReactNode;
  countChildren: number;
  children: React.ReactNode;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
}

const GroupItem = forwardRef<HTMLDivElement, GroupItemProps>(function GroupItem(props, ref) {
  const { itemId, groupName, icon = <></>, countChildren = 0, onDoubleClick = () => {}, ...children } = props;

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
    <StyledTreeItem
      itemId={itemId}
      slotProps={
        {
          label: { onClick: handleLabelClick },
          content: { onClick: handleContentClick },
          iconContainer: { onClick: handleIconContainerClick },
        } as TreeItem2SlotProps
      }
      onDoubleClick={(event) => onDoubleClick(event, itemId)}
      label={
        <Box ref={ref} display="flex" alignItems="center" paddingLeft={0.0}>
          {icon}
          <Stack
            direction="row"
            onDoubleClick={(event) => {
              if (onDoubleClick) {
                onDoubleClick(event, itemId);
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
      {...children}
    />
  );
});

export default GroupItem;
