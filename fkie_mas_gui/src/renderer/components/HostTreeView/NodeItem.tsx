import RosContext from "@/renderer/context/RosContext";
import { TRosMessageStruct } from "@/renderer/models/TRosMessageStruct";
import { nodeNameWithoutNamespace } from "@/renderer/utils";
import { TTag } from "@/types";
import CircleIcon from "@mui/icons-material/Circle";
import DesktopAccessDisabledOutlinedIcon from "@mui/icons-material/DesktopAccessDisabledOutlined";
import DvrIcon from "@mui/icons-material/Dvr";
import DynamicFeedOutlinedIcon from "@mui/icons-material/DynamicFeedOutlined";
import NewReleasesTwoToneIcon from "@mui/icons-material/NewReleasesTwoTone";
import ReportIcon from "@mui/icons-material/Report";
import SettingsInputCompositeOutlinedIcon from "@mui/icons-material/SettingsInputCompositeOutlined";
import WarningIcon from "@mui/icons-material/Warning";
import { Box, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { blue, green, grey, orange, red, yellow } from "@mui/material/colors";
import { forwardRef, useContext, useEffect, useState } from "react";
import { SettingsContext } from "../../context/SettingsContext";
import { DiagnosticLevel, LaunchCallService, RosNode, RosNodeStatus } from "../../models";
import { OverflowMenu } from "../UI";
import Tag from "../UI/Tag";
import StyledTreeItem from "./StyledTreeItem";

interface NodeItemProps {
  itemId: string;
  node: RosNode;
  namespacePart: string;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
  onShowLoggersClick: (event: React.MouseEvent, itemId: string) => void;
}

const NodeItem = forwardRef<HTMLDivElement, NodeItemProps>(function NodeItem(props, ref) {
  const { itemId, node, namespacePart = "", onDoubleClick = () => {}, onShowLoggersClick = () => {}, ...other } = props;

  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [labelText, setLabelText] = useState(nodeNameWithoutNamespace(node));

  const getColorFromDiagnostic = (diagnosticLevel: DiagnosticLevel, isDarkMode: boolean = false) => {
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

  const getColorFromLifecycle = (state: string, isDarkMode: boolean = false) => {
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
  };
  const getNodeIcon = (node: RosNode, isDarkMode: boolean = false) => {
    switch (node.status) {
      case RosNodeStatus.RUNNING: {
        const color = getColorFromDiagnostic(node.diagnosticLevel, isDarkMode);
        if (!node.lifecycle_state) {
          return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
        } else {
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
                  onClick: async () => {
                    const provider = rosCtx.getProviderById(node.providerId as string, true);
                    await provider?.callService(
                      new LaunchCallService(`${node.name}/change_state`, "lifecycle_msgs/srv/ChangeState", {
                        type: "lifecycle_msgs/srv/ChangeState",
                        name: "",
                        def: [
                          {
                            name: "transition",
                            def: [
                              { name: "id", def: [], type: "uint8", is_array: false, value: `${item.id}` },
                              { name: "label", def: [], type: "string", is_array: false },
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
        if (node.screens.length === 1) {
          const color = isDarkMode ? green[600] : green[500];
          return <DvrIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
        } else {
          const color = isDarkMode ? grey[600] : grey[500];
          let icon = <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
          node.launchInfo.forEach((launchInfo) => {
            if (launchInfo.cmd?.includes("ros2 run")) {
              icon = (
                <Tooltip
                  key={`icon-${node.id}`}
                  title={`Executable '${launchInfo.executable}' or package '${launchInfo.package_name}' not found`}
                  placement="left"
                  disableInteractive
                >
                  <NewReleasesTwoToneIcon style={{ marginRight: 0.5, width: 20, color: color }} />
                </Tooltip>
              );
            }
          });

          return icon;
        }
      }

      default: {
        const color = isDarkMode ? red[600] : red[500];
        return <CircleIcon style={{ marginRight: 0.5, width: 20, color: color }} />;
      }
    }
  };

  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);
  const [nodeIcon, setNodeIcon] = useState(getNodeIcon(node, isDarkMode));

  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    setNodeIcon(getNodeIcon(node, isDarkMode));
    setLabelText(nodeNameWithoutNamespace(node));
  }, [node, isDarkMode]);

  return (
    <StyledTreeItem
      itemId={itemId}
      // onDoubleClick={(event) => onDoubleClick(event, labelText, itemId)}
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
          {node.status === RosNodeStatus.RUNNING && node.screens.length > 1 && (
            <Tooltip title="Multiple Screens" placement="left">
              <DynamicFeedOutlinedIcon color="warning" style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status === RosNodeStatus.RUNNING && node.screens.length < 1 && (
            <Tooltip title="No Screens" placement="left">
              <DesktopAccessDisabledOutlinedIcon style={{ fontSize: "inherit" }} />
            </Tooltip>
          )}
          {node.status !== RosNodeStatus.RUNNING && node.screens.length > 1 && (
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
