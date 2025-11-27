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
import { Badge, Box, IconButton, Menu, MenuItem, Stack, Tooltip, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { useContext, useEffect, useState } from "react";
import { FileIcon } from "react-file-icon";

import RosContext from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import {
  DiagnosticLevel,
  getFileExtension,
  getFileName,
  LaunchCallService,
  LaunchNodeInfo,
  RosNode,
  RosNodeStatus,
} from "@/renderer/models";
import { EventNodeDiagnostic } from "@/renderer/providers/events";
import { EVENT_NODE_DIAGNOSTIC } from "@/renderer/providers/eventTypes";
import { nodeNameWithoutNamespace } from "@/renderer/utils";
import { TTag } from "@/types";
import { TRosMessageStruct } from "@/types/TRosMessageStruct";
import { treeItemClasses } from "@mui/x-tree-view";
import { useCustomEventListener } from "react-custom-events";
import { OverflowMenu } from "../UI";
import { colorFromHostname, getDiagnosticColor } from "../UI/Colors";
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

export default function NodeItem(props: NodeItemProps): JSX.Element {
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
  const [showLaunchFile, setShowLaunchFile] = useState<boolean>(
    settingsCtx.get("showLaunchFileIndicatorForNodes") as boolean
  );
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);
  const [nodeIcon, setNodeIcon] = useState(getNodeIcon(node, isDarkMode));
  const [timerPeriod, setTimerPeriod] = useState<number[]>([]);
  const [sigKillTimeout, setSigKillTimeout] = useState<number[]>([]);

  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
    setShowLaunchFile(settingsCtx.get("showLaunchFileIndicatorForNodes") as boolean);
  }, [settingsCtx, settingsCtx.changed]);

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

  const callLifecycleService: (node: RosNode, transitionId: number) => void = async (node, transitionId) => {
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
                value: `${transitionId}`,
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
  };

  function getLifecycleMenuItems(node: RosNode): JSX.Element[] {
    // add menu to change the lifecycle state
    return (
      node.lifecycle_available_transitions?.map((item) => {
        return (
          <MenuItem
            key={`context-menu-${item.label}`}
            sx={{ fontSize: "0.8em" }}
            onClick={async (): Promise<void> => {
              setContextMenu(null);
              callLifecycleService(node, item.id);
            }}
          >
            {item.label}
          </MenuItem>
        );
      }) || []
    );
  }

  function getNodeIcon(node: RosNode, isDarkMode = false): JSX.Element {
    switch (node.status) {
      case RosNodeStatus.RUNNING: {
        const color = node.diagnostic?.getColor(isDarkMode) || getDiagnosticColor(DiagnosticLevel.OK, isDarkMode);
        const IconType = node.isLocal ? CircleIcon : ReportIcon;
        if (!node.lifecycle_state) {
          if (node.pid || node.screens) {
            return <IconType style={{ marginRight: 0.5, width: 20, color: color }} />;
          }
          return (
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
                    Note: no status checks for life cycle, composable node or other service calls are performed!
                  </Typography>
                </div>
              }
              placement="left"
              disableInteractive
            >
              <IconType style={{ marginRight: 0.5, width: 20, color: color }} />
            </Tooltip>
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
            <IconType
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
                  callLifecycleService(node, item.id);
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
        return (
          <Tooltip
            key={`tooltip-icon-${node.id}`}
            title={"Only screen, no ROS data available for this process"}
            placement="left"
            disableInteractive
          >
            <DvrIcon style={{ marginRight: 0.5, width: 20, color: color }} />
          </Tooltip>
        );
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
      label={
        <Stack direction="row" display="flex" alignItems="center" justifyItems="center" paddingLeft={0.0}>
          <Stack
            direction="row"
            flexGrow={1}
            alignItems="center"
            onContextMenu={(event) => {
              event.preventDefault();
              setContextMenu(
                contextMenu === null
                  ? {
                      mouseX: event.clientX + 2,
                      mouseY: event.clientY - 6,
                    }
                  : null
              );
            }}
          >
            {nodeIcon}
            <Stack direction="row" paddingLeft={0.5} sx={{ userSelect: "none" }} alignItems="center">
              <Typography variant="body2" sx={{ fontSize: "inherit", userSelect: "none" }}>
                {namespacePart}
              </Typography>
              <Typography variant="body2" sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}>
                {labelText}
              </Typography>
              {node.countSameName > 1 && (
                <Tooltip
                  title={`${node.countSameName} nodes with the same name have been defined. Only the first node will be started on play!`}
                  placement="top"
                  disableInteractive
                >
                  <Badge
                    badgeContent={node.countSameName}
                    color="error"
                    sx={{
                      left: 15,
                    }}
                  />
                </Tooltip>
              )}
            </Stack>
          </Stack>
          <Stack direction="row" display="flex" alignItems="center" justifyItems="center" paddingLeft={0.0}>
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
            {showLaunchFile &&
              node.launchInfo.size > 1 &&
              [...node.launchInfo.values()].map((launchInfo) => {
                const fileExtension = getFileExtension(launchInfo.launch_name as string);
                const color = colorFromHostname(launchInfo.launch_name || "");
                return (
                  <Tooltip
                    key={launchInfo.launch_name}
                    title={
                      <Stack padding={0} margin={0}>
                        <Typography fontWeight="bold" fontSize="inherit">
                          {getFileName(launchInfo.launch_name || "")}
                        </Typography>
                        <Typography fontWeight={"bold"} fontSize={"inherit"}>
                          Path:
                        </Typography>
                        <div>{launchInfo.launch_name}</div>
                        {launchInfo.launch_name?.localeCompare(launchInfo.file_name || "") !== 0 && (
                          <Stack>
                            <Typography fontWeight={"bold"} fontSize={"inherit"}>
                              node within the included file:
                            </Typography>
                            <div>{launchInfo.file_name}</div>
                          </Stack>
                        )}
                      </Stack>
                    }
                    placement="left"
                  >
                    <Box sx={{ ml: "0.5em", width: 15, height: 18 }}>
                      <FileIcon
                        extension={fileExtension}
                        labelUppercase
                        {...{
                          color: grey[300],
                          labelColor: color,
                          type: fileExtension === "py" ? "code" : "settings",
                        }}
                      />
                    </Box>
                  </Tooltip>
                );
              })}
            {showLaunchFile && node.launchInfo.size === 0 && !node.system_node && (
              <Tooltip title="There is no launch file available to restart the node." placement="left">
                <Box sx={{ ml: "0.5em", width: 15, height: 18 }}>
                  <FileIcon
                    labelUppercase
                    {...{
                      color: red[300],
                      labelColor: "red",
                      type: "settings",
                    }}
                  />
                </Box>
              </Tooltip>
            )}
          </Stack>
          <Menu
            open={contextMenu != null}
            onClose={() => setContextMenu(null)}
            anchorReference="anchorPosition"
            anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
          >
            {getLifecycleMenuItems(node)}
          </Menu>
        </Stack>
      }
      {...other}
    />
  );
}
