import CircleIcon from "@mui/icons-material/Circle";
import { default as ContrastIcon } from "@mui/icons-material/Contrast";
import ReportIcon from "@mui/icons-material/Report";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { blue, green, grey, red, yellow } from "@mui/material/colors";
import { treeItemClasses, TreeItemSlotProps } from "@mui/x-tree-view/TreeItem";
import { UseTreeItemContentSlotOwnProps } from "@mui/x-tree-view/useTreeItem";
import { UseTreeItemIconContainerSlotOwnProps } from "@mui/x-tree-view/useTreeItem/useTreeItem.types";
import { forwardRef, LegacyRef, useEffect, useMemo, useState } from "react";

import { DiagnosticLevel, getMaxDiagnosticLevel, RosNodeStatus } from "@/renderer/models";
import { EVENT_NODE_DIAGNOSTIC } from "@/renderer/providers/eventTypes";
import { EventNodeDiagnostic } from "@/renderer/providers/events";
import { useCustomEventListener } from "react-custom-events";
import { averageColor, getDiagnosticColor } from "../UI/Colors";
import StyledTreeItem from "./StyledTreeItem";
import { NodeTreeItem } from "./types";

const GroupStatus = {
  ALL_INACTIVE: 0,
  SOME_RUNNING: 1,
  ALL_RUNNING: 2,
};

const GroupStatusLocal = {
  ALL: 1,
  SOME: 2,
  NONE: 3,
};

const GroupLifecycleStatus = {
  NOT_CONFIGURED: 0,
  INACTIVE: 1,
  ACTIVE: 2,
  FINALIZED: 3,
  NO_ONE: 10,
};

const getGroupLifecycleStatus: (treeItems: NodeTreeItem[]) => number = (treeItems) => {
  let groupStatus = GroupLifecycleStatus.NO_ONE;
  for (const treeItem of treeItems) {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenStatus = getGroupLifecycleStatus(treeItem.children);
      if (childrenStatus < groupStatus) {
        groupStatus = childrenStatus;
      }
    }
    if (treeItem.node?.lifecycle_state) {
      switch (treeItem.node.lifecycle_state) {
        case "unconfigured":
          groupStatus = GroupLifecycleStatus.NOT_CONFIGURED;
          break;
        case "inactive":
          if (groupStatus > GroupLifecycleStatus.INACTIVE) {
            groupStatus = GroupLifecycleStatus.INACTIVE;
          }
          break;
        case "active":
          if (groupStatus > GroupLifecycleStatus.ACTIVE) {
            groupStatus = GroupLifecycleStatus.ACTIVE;
          }
          break;
        case "finalized":
          if (groupStatus > GroupLifecycleStatus.FINALIZED) {
            groupStatus = GroupLifecycleStatus.FINALIZED;
          }
          break;
      }
    }
  }

  return groupStatus;
};

const getGroupStatus: (treeItems: NodeTreeItem[]) => number = (treeItems) => {
  let groupStatus = GroupStatus.ALL_INACTIVE;
  let allRunning = true;
  for (const treeItem of treeItems) {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenStatus = getGroupStatus(treeItem.children);
      if (childrenStatus !== GroupStatus.ALL_INACTIVE) {
        groupStatus = GroupStatus.SOME_RUNNING;
      }
      if (childrenStatus !== GroupStatus.ALL_RUNNING) {
        allRunning = false;
      }
    }
    if (
      treeItem.node &&
      (treeItem.node.status === RosNodeStatus.RUNNING || treeItem.node.status === RosNodeStatus.ONLY_SCREEN)
    ) {
      groupStatus = GroupStatus.SOME_RUNNING;
    } else if (treeItem.node) {
      allRunning = false;
    }
  }
  if (groupStatus === GroupStatus.SOME_RUNNING && allRunning) {
    groupStatus = GroupStatus.ALL_RUNNING;
  }
  return groupStatus;
};

const getGroupStatusLocal: (treeItems: NodeTreeItem[]) => number = (treeItems) => {
  let allLocal = true;
  let hasLocal = false;
  for (const treeItem of treeItems) {
    if (treeItem.node) {
      if (treeItem.node?.isLocal) {
        hasLocal = true;
      } else {
        allLocal = false;
      }
    } else {
      const groupStatus = getGroupStatusLocal(treeItem.children);
      if (groupStatus === GroupStatusLocal.ALL) {
        hasLocal = true;
      } else if (groupStatus === GroupStatusLocal.NONE) {
        allLocal = false;
      }
    }
  }
  if (allLocal) {
    return GroupStatusLocal.ALL;
  }
  if (!hasLocal) {
    return GroupStatusLocal.NONE;
  }
  return GroupStatusLocal.SOME;
};

const getGroupDiagnosticLevel: (treeItems: NodeTreeItem[]) => DiagnosticLevel = (treeItems) => {
  let groupLevel = DiagnosticLevel.OK;
  for (const treeItem of treeItems) {
    if (treeItem.children && treeItem.children.length > 0) {
      const childrenLevel = getGroupDiagnosticLevel(treeItem.children);
      groupLevel = getMaxDiagnosticLevel(groupLevel, childrenLevel) || DiagnosticLevel.OK;
    }
    if (treeItem.node) {
      groupLevel = getMaxDiagnosticLevel(groupLevel, treeItem.node.diagnosticLevel) || DiagnosticLevel.OK;
    }
  }
  return groupLevel;
};

function getGroupIconColor(treeItems: NodeTreeItem[], isDarkMode: boolean = false): string {
  const groupStatus = getGroupStatus(treeItems);
  const groupDiagLevel = getGroupDiagnosticLevel(treeItems);
  let groupColor = getDiagnosticColor(groupDiagLevel, isDarkMode);
  // get color from diagnosticColor of the nodes
  const nodeColors: string[] = [];
  for (const nodeItem of treeItems) {
    if (nodeItem.node?.diagnosticColor) {
      nodeColors.push(nodeItem.node?.diagnosticColor);
    }
  }
  if (nodeColors.length > 0) {
    groupColor = averageColor(nodeColors, isDarkMode);
  }
  switch (groupStatus) {
    case GroupStatus.ALL_RUNNING:
      return groupColor;
    case GroupStatus.SOME_RUNNING:
      return groupColor;

    case GroupStatus.ALL_INACTIVE:
      return isDarkMode ? grey[600] : grey[500];

    default:
      return isDarkMode ? red[600] : red[500];
  }
}

function getColorFromLifecycle(state: number, isDarkMode: boolean = false): string {
  switch (state) {
    case GroupLifecycleStatus.NOT_CONFIGURED:
      return isDarkMode ? blue[600] : blue[500];
    case GroupLifecycleStatus.INACTIVE:
      return isDarkMode ? grey[600] : grey[400];
    case GroupLifecycleStatus.ACTIVE:
      return isDarkMode ? green[600] : green[500];
    case GroupLifecycleStatus.FINALIZED:
      return isDarkMode ? grey[700] : grey[900];
    default:
      return isDarkMode ? yellow[600] : yellow[500];
  }
}

function getNameFromLifecycle(state: number): string {
  switch (state) {
    case GroupLifecycleStatus.NOT_CONFIGURED:
      return "unconfigured";
    case GroupLifecycleStatus.INACTIVE:
      return "inactive";
    case GroupLifecycleStatus.ACTIVE:
      return "active";
    case GroupLifecycleStatus.FINALIZED:
      return "finalized";
    default:
      return "unknown";
  }
}

interface GroupIconProps {
  treeItems: NodeTreeItem[];
  groupName: string;
  isDarkMode: boolean;
}

export const GroupIcon = forwardRef<HTMLDivElement, GroupIconProps>(function GroupIcon(props, ref) {
  const { treeItems, groupName, isDarkMode = false } = props;
  const [groupLifecycleStatus, setGroupLifecycleStatus] = useState<number>(getGroupLifecycleStatus(treeItems));
  const [groupStatus, setGroupStatus] = useState<number>(getGroupStatus(treeItems));
  const [groupStatusLocal, setGroupStatusLocal] = useState<number>(getGroupStatusLocal(treeItems));
  const [color, setColor] = useState<string>(getGroupIconColor(treeItems, isDarkMode));
  const [colorBorder, setColorBorder] = useState<string>(getColorFromLifecycle(groupLifecycleStatus, isDarkMode));

  useEffect(() => {
    setGroupLifecycleStatus(getGroupLifecycleStatus(treeItems));
    setGroupStatus(getGroupStatus(treeItems));
    setColor(getGroupIconColor(treeItems, isDarkMode));
    setGroupStatusLocal(getGroupStatusLocal(treeItems));
  }, [treeItems, isDarkMode]);

  useEffect(() => {
    setColorBorder(getColorFromLifecycle(groupLifecycleStatus, isDarkMode));
  }, [groupLifecycleStatus, isDarkMode]);

  useCustomEventListener(EVENT_NODE_DIAGNOSTIC, (data: EventNodeDiagnostic) => {
    // update group icon if the name of the node contains the group name
    if (
      data.node.name.indexOf(`${groupName}/`) > -1 ||
      data.node.capabilityGroup.name === groupName ||
      (data.node.system_node && groupName.startsWith("{"))
    ) {
      setColor(getGroupIconColor(treeItems, isDarkMode));
    }
  });

  const createIcon = useMemo(() => {
    if (groupLifecycleStatus === GroupLifecycleStatus.NO_ONE) {
      switch (groupStatus) {
        case GroupStatus.ALL_RUNNING: {
          if (groupStatusLocal === GroupStatusLocal.NONE) {
            return (
              <Tooltip
                title={
                  <div>
                    <Typography fontWeight="bold" fontSize="inherit">
                      The processes of all nodes are not found on the local host.
                    </Typography>
                    <Typography fontSize={"inherit"}>
                      There is no screen with the name of the node, nor was the ROS node started with the __node:=,
                      __ns:= parameter, nor is the GID of the node detected by mas-discovery.
                    </Typography>
                    <Typography fontSize={"inherit"}>
                      Note: no checks for life cycle, composable node or other service calls are performed!
                    </Typography>
                  </div>
                }
                placement="left"
                disableInteractive
              >
                <ReportIcon style={{ marginRight: 0.5, width: 20, color: color }} />
              </Tooltip>
            );
          }
          return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
        }

        case GroupStatus.SOME_RUNNING:
          return (
            <ContrastIcon
              sx={{
                transform: "rotate(-90deg)",
              }}
              style={{
                marginRight: 0.5,
                width: 20,
                color: color,
              }}
            />
          );

        case GroupStatus.ALL_INACTIVE:
          return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;

        default:
          return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }
    }
    if (groupStatus === GroupStatus.SOME_RUNNING) {
      return (
        <Tooltip
          key={`tooltip-icon-${groupLifecycleStatus}`}
          title={`Lifecycle state: '${getNameFromLifecycle(groupLifecycleStatus)}'`}
          placement="left"
          disableInteractive
        >
          <ContrastIcon
            style={{ marginRight: 0.5, width: 20, height: 20, color: color, borderColor: colorBorder }}
            sx={{
              transform: "rotate(-90deg)",
              border: 3,
              borderRadius: "100%",
              borderColor: colorBorder,
            }}
          />
        </Tooltip>
      );
    }
    return (
      <Tooltip
        ref={ref}
        key={`tooltip-icon-${groupLifecycleStatus}`}
        title={`Lifecycle state: '${getNameFromLifecycle(groupLifecycleStatus)}'`}
        placement="left"
        disableInteractive
      >
        <CircleIcon
          style={{ marginRight: 0.5, width: 20, height: 20, color: color, borderColor: colorBorder }}
          sx={{
            border: 3,
            borderRadius: "100%",
            borderColor: colorBorder,
          }}
        />
      </Tooltip>
    );
  }, [groupLifecycleStatus, colorBorder, color, groupStatus, groupStatusLocal]);

  return createIcon;
});

/** Returns count of nodes for given group */
export function NodesCount(children: NodeTreeItem[]): number {
  let result = 0;
  let itemsCount = 0;
  for (const treeItem of children) {
    if (treeItem.children && treeItem.children.length > 0) {
      result += NodesCount(treeItem.children);
    } else {
      itemsCount += 1;
    }
  }
  return result + itemsCount;
}

interface GroupItemProps {
  itemId: string;
  groupName: string;
  icon: React.ReactNode;
  countChildren: number;
  children: React.ReactNode;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
}

const GroupItem = forwardRef<HTMLDivElement, GroupItemProps>(function GroupItem(props, ref) {
  const { itemId, groupName, icon = <></>, countChildren = 0, onDoubleClick = (): void => {}, ...children } = props;

  // avoid selection if collapse icon was clicked
  let toggled = false;
  const handleContentClick: UseTreeItemContentSlotOwnProps["onClick"] = (event) => {
    event.defaultMuiPrevented = toggled;
    toggled = false;
  };

  const handleLabelClick: UseTreeItemContentSlotOwnProps["onClick"] = () => {};

  const handleIconContainerClick: UseTreeItemIconContainerSlotOwnProps["onClick"] = () => {
    toggled = true;
  };

  const createGroupItem = useMemo(() => {
    return (
      <StyledTreeItem
        itemId={itemId}
        ref={ref as LegacyRef<HTMLLIElement>}
        slotProps={
          {
            label: { onClick: handleLabelClick },
            content: { onClick: handleContentClick },
            iconContainer: { onClick: handleIconContainerClick },
          } as TreeItemSlotProps
        }
        onDoubleClick={(event) => onDoubleClick(event, itemId)}
        sx={{
          [`& .${treeItemClasses.content}`]: {
            paddingLeft: "7px",
          },
        }}
        label={
          <Box display="flex" alignItems="center" paddingLeft={0.0}>
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
  }, [itemId, countChildren, icon, handleIconContainerClick]);

  return createGroupItem;
});

export default GroupItem;
