import AutoDeleteIcon from "@mui/icons-material/AutoDelete";
import CircleIcon from "@mui/icons-material/Circle";
import DesktopAccessDisabledOutlinedIcon from "@mui/icons-material/DesktopAccessDisabledOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
import NewReleasesTwoToneIcon from "@mui/icons-material/NewReleasesTwoTone";
import ReportIcon from "@mui/icons-material/Report";
import ScheduleSendIcon from "@mui/icons-material/ScheduleSend";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import WarningIcon from "@mui/icons-material/Warning";
import { Box, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { forwardRef, useContext, useEffect, useState } from "react";

import RosContext from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { DiagnosticLevel, LaunchCallService, LaunchNodeInfo, RosNode, RosNodeStatus } from "@/renderer/models";
import { TRosMessageStruct } from "@/renderer/models/TRosMessageStruct";
import { EventNodeDiagnostic } from "@/renderer/providers/events";
import { EVENT_NODE_DIAGNOSTIC } from "@/renderer/providers/eventTypes";
import { nodeNameWithoutNamespace } from "@/renderer/utils";
import { TTag } from "@/types";
import { treeItemClasses } from "@mui/x-tree-view";
import { useCustomEventListener } from "react-custom-events";
import { OverflowMenu } from "../UI";
import { getDiagnosticColor } from "../UI/Colors";
import Tag from "../UI/Tag";
import StyledTreeItem from "./StyledTreeItem";

interface NodeItemProps {
  itemId: string;
  node: RosNode;
  namespacePart: string;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
  onMiddleClick: (event: React.MouseEvent, id: string) => void;
  onShowLoggersClick: (event: React.MouseEvent, itemId: string) => void;
}

const NodeItem = forwardRef<HTMLDivElement, NodeItemProps>(function NodeItem(props, ref) {
  const {
    itemId,
    node,
    namespacePart = "",
    onDoubleClick = (): void => {},
    onMiddleClick = (): void => {},
    onShowLoggersClick = (): void => {},
    ...other
  } = props;

  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [labelText, setLabelText] = useState(nodeNameWithoutNamespace(node));
  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);
  const [nodeIcon, setNodeIcon] = useState(getNodeIcon(node, isDarkMode));
  const [timerPeriod, setTimerPeriod] = useState<number[]>([]);
  const [sigKillTimeout, setSigKillTimeout] = useState<number[]>([]);

  function getColorFromLifecycle(state: string, isDarkMode = false): string {
    switch (state) {
      case "unconfigured":
        return isDarkMode ? blue[600] : blue[500];
      case "inactive":
        return isDarkMode ? grey[600] : grey[400];
      case "active":
        return isDarkMode ? green[600] : green[500];
      case "finalized":
        return isDarkMode ? grey[700] : grey[900];
      default:
        return isDarkMode ? yellow[600] : yellow[500];
    }
  }
  function getNodeIcon(node: RosNode, isDarkMode = false): JSX.Element {
    switch (node.status) {
      case RosNodeStatus.RUNNING: {
        const color = node.diagnosticColor
          ? node.diagnosticColor
          : getDiagnosticColor(node.diagnosticLevel || DiagnosticLevel.OK, isDarkMode);
        if (!node.lifecycle_state) {
          return !node.isLocal ? (
            <Tooltip
              key={`tooltip-icon-${node.id}`}
              title={
                <div>
                  <Typography fontWeight="bold" fontSize="inherit">
                    The process of the node was not found on the local host.
                  </Typography>
                  <Typography fontSize={"inherit"}>
                    There is no screen with the name of the node, nor was the ROS node started with the __node:=, __ns:=
                    parameter, nor is the GID of the node detected by mas-discovery.
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
          ) : (
            <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />
          );
        }
        const colorBorder = getColorFromLifecycle(node.lifecycle_state, isDarkMode);
        const iconState = (
          <Tooltip
            key={`tooltip-icon-${node.id}`}
            title={`Lifecycle state: '${node.lifecycle_state}'`}
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
        // add menu to change the lifecycle state
        return node.lifecycle_available_transitions && node.lifecycle_available_transitions?.length > 0 ? (
          <OverflowMenu
            icon={iconState}
            options={node.lifecycle_available_transitions?.map((item) => {
              return {
                name: item.label,
                key: item.label,
                onClick: async (): Promise<void> => {
                  const provider = rosCtx.getProviderById(node.providerId as string, true);
                  await provider?.callService(
                    new LaunchCallService(`${node.name}/change_state`, "lifecycle_msgs/srv/ChangeState", {
                      type: "lifecycle_msgs/srv/ChangeState",
                      name: "",
                      useNow: false,
                      def: [
                        {
                          name: "transition",
                          useNow: false,
                          def: [
                            {
                              name: "id",
                              useNow: false,
                              def: [],
                              type: "uint8",
                              is_array: false,
                              value: `${item.id}`,
                            },
                            { name: "label", useNow: false, def: [], type: "string", is_array: false },
                          ],
                          type: "lifecycle_msgs/Transition",
                          is_array: false,
                        },
                      ],
                      is_array: false,
                    } as TRosMessageStruct)
                  );
                },
              };
            })}
            id={`lifecycle-menu-${node.id}`}
          />
        ) : (
          iconState
        );
      }

      case RosNodeStatus.DEAD: {
        const color = isDarkMode ? orange[600] : orange[400];
        return <WarningIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.NOT_MONITORED: {
        const color = isDarkMode ? blue[700] : blue[500];
        return <ReportIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.ONLY_SCREEN: {
        const color = isDarkMode ? green[600] : green[500];
        return <DvrIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }

      case RosNodeStatus.INACTIVE: {
        if ((node.screens || []).length === 1) {
          const color = isDarkMode ? green[600] : green[500];
          return <DvrIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
        }
        const color = isDarkMode ? grey[600] : grey[500];
        let icon = <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
        for (const launchInfo of node.launchInfo.values()) {
          if (launchInfo.cmd?.includes("ros2 run")) {
            icon = (
              <Tooltip
                key={`icon-${node.id}`}
                title={`Executable '${launchInfo.executable}' or package '${JSON.stringify(launchInfo.package_name)}' not found`}
                placement="left"
                disableInteractive
              >
                <NewReleasesTwoToneIcon style={{ marginRight: 0.5, width: 20, color: color }} />
              </Tooltip>
            );
          }
        }

        return icon;
      }

      default: {
        const color = isDarkMode ? red[600] : red[500];
        return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }
    }
  }

  useCustomEventListener(EVENT_NODE_DIAGNOSTIC, (data: EventNodeDiagnostic) => {
    if (data.node.name === node.name) {
      setNodeIcon(getNodeIcon(data.node, isDarkMode));
    }
  });

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
  }, [settingsCtx, settingsCtx.changed]);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setNodeIcon(getNodeIcon(node, isDarkMode));
    setLabelText(nodeNameWithoutNamespace(node));
    setTimerPeriod(
      node.launchInfo?.values().reduce((list: number[], li: LaunchNodeInfo) => {
        if (li.timer_period && li.timer_period > 0) {
          list.push(li.timer_period);
        }
        return list;
      }, [])
    );
    setSigKillTimeout(
      node.launchInfo?.values().reduce((list: number[], li: LaunchNodeInfo) => {
        if (li.sigkill_timeout && li.sigkill_timeout > 0) {
          list.push(li.sigkill_timeout);
        }
        return list;
      }, [])
    );
  }, [node, isDarkMode]);

  return (
    <StyledTreeItem
      itemId={itemId}
      // onDoubleClick={(event) => onDoubleClick(event, labelText, itemId)}
      sx={{
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: "8px",
        },
      }}
      label={
        <Box ref={ref} display="flex" alignItems="center" paddingLeft={0.0}>
          {nodeIcon}
          <Stack
            direction="row"
            // onClick={onClick}
            onDoubleClick={(event) => {
              if (onDoubleClick) {
                onDoubleClick(event, itemId);
                event.stopPropagation();
              }
            }}
            onMouseDown={(event) => {
              if (event.button === 1) {
                if (onMiddleClick) {
                  onMiddleClick(event, itemId);
                  event.stopPropagation();
                }
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
          {node.tags.map((tag: TTag) => (
            <Tooltip
              key={tag.id}
              title={`${tag.tooltip}`}
              placement="left"
              disableInteractive
              onClick={(event) => {
                if (tag.onClick) {
                  tag.onClick(event);
                }
              }}
            >
              {typeof tag.data === "string" ? (
                <div>
                  <Tag text={tag.data} color={tag.color} />
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
                onClick={(event) => {
                  if (onShowLoggersClick) {
                    onShowLoggersClick(event, itemId);
                  }
                }}
              >
                <SettingsInputCompositeOutlinedIcon style={{ fontSize: "inherit", rotate: "90deg" }} />
              </IconButton>
            </Tooltip>
          )}
          {node.status !== RosNodeStatus.RUNNING && timerPeriod.length > 0 && (
            <Tooltip title={`Delayed start due to configuration ${JSON.stringify(timerPeriod)} sec`} placement="left">
              <ScheduleSendIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {(node.status === RosNodeStatus.RUNNING || node.status === RosNodeStatus.ONLY_SCREEN) &&
            sigKillTimeout.length > 0 && (
              <Tooltip
                title={`sigkill timeout after stop command ${JSON.stringify(sigKillTimeout)} ms`}
                placement="left"
              >
                <AutoDeleteIcon color="warning" style={{ fontSize: "inherit" }} />
              </Tooltip>
            )}{" "}
          {node.status === RosNodeStatus.RUNNING && (node.screens || []).length > 1 && (
            <Tooltip title="Multiple Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status === RosNodeStatus.RUNNING && (node.screens || []).length < 1 && (
            <Tooltip title="No Screens" placement="left">
              <DesktopAccessDisabledOutlinedIcon style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status !== RosNodeStatus.RUNNING && (node.screens || []).length > 1 && (
            <Tooltip title="Ghost Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
        </Box>
      }
      {...other}
    />
  );
});

export default NodeItem;
