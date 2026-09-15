import ChangeCircleOutlinedIcon from "@mui/icons-material/ChangeCircleOutlined";
import ComputerIcon from "@mui/icons-material/Computer";
import DeveloperBoardIcon from "@mui/icons-material/DeveloperBoard";
import LinkIcon from "@mui/icons-material/Link";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import MemoryIcon from "@mui/icons-material/Memory";
import MoreVertIcon from "@mui/icons-material/MoreVert";
import NetworkCheckIcon from "@mui/icons-material/NetworkCheck";
import StorageIcon from "@mui/icons-material/Storage";
import WatchLaterIcon from "@mui/icons-material/WatchLater";
import {
  Box,
  ClickAwayListener,
  Grow,
  IconButton,
  Menu,
  MenuItem,
  MenuList,
  Paper,
  Popper,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { green, grey, orange, red } from "@mui/material/colors";
import {
  treeItemClasses,
  TreeItemSlotProps,
  UseTreeItemContentSlotOwnProps,
  UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { useCallback, useEffect, useState } from "react";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "@/renderer/components/layout";
import { emitOpenComponent } from "@/renderer/components/layout/events";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { RosNode, RosNodeStatus } from "@/renderer/models";
import Provider from "@/renderer/providers/Provider";
import { EVENT_SYSTEM_DIAGNOSTICS } from "@/renderer/providers/eventTypes";
import { TEventDiagnostics } from "@/renderer/providers/events";
import { generateUniqueId } from "@/renderer/utils";
import { CmdTypes, TTag } from "@/types";
import { useCustomEventListener } from "react-custom-events";
import Tag from "../UI/Tag";
import DateHelpDialog from "./DateHelpDialog";
import SetNTPDateDialog from "./SetNTPDateDialog";
import StyledRootTreeItem from "./StyledRootTreeItem";

type TDiagValue = { key: string; value: string };
type TDiagStatus = {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: TDiagValue[];
};
type TDiagEntry = { status: TDiagStatus; receivedAt: number };

/** Maximum age of a diagnostic entry to be displayed */
const DIAG_MAX_AGE_MS = 15_000;

/** Only global (system) diagnostics, no node diagnostics */
const SYSTEM_DIAG_NAMES = ["cpu", "memory", "hdd", "disk", "network"];

function isSystemDiagnostic(status: TDiagStatus): boolean {
  const name = (status.name || "").toLowerCase();
  return !name.includes("/") && SYSTEM_DIAG_NAMES.some((n) => name.includes(n));
}

/** True if the status contains any real measurement (not only a timestamp) */
function hasDiagValues(status: TDiagStatus): boolean {
  return (status.values || []).some((v) => v.key !== "Timestamp");
}

function diagValue(status: TDiagStatus, key: string): string | undefined {
  return status.values?.find((v) => v.key === key)?.value;
}

/** Extract the bandwidth limit in bytes/s from a message like "warn at >90.00% at 6MBit" */
/** Extract the bandwidth limit in bytes/s from a message like "warn at >90.00% at 6MBit" */
function networkLimitBytes(status: TDiagStatus): number | undefined {
  // case-insensitive: the daemon writes "MBit", "Mbit" or "mbit"
  const match = /([\d.]+)\s*([kmg])?\s*bit(?:\/s)?/i.exec(status.message || "");
  if (!match) return undefined;
  const value = Number.parseFloat(match[1]);
  if (Number.isNaN(value)) return undefined;
  const factor = { k: 1e3, m: 1e6, g: 1e9 }[(match[2] || "").toLowerCase()] ?? 1;
  return (value * factor) / 8.0;
}

/** Format a byte rate as human readable string */
function formatRate(bytesPerSec: number): string {
  const units = ["B/s", "KiB/s", "MiB/s", "GiB/s"];
  let value = bytesPerSec;
  let index = 0;
  while (value >= 1024 && index < units.length - 1) {
    value /= 1024;
    index += 1;
  }
  return `${value.toFixed(2)} ${units[index]}`;
}

/** Add a percentage (and human readable rate) to network values like "enp0: sent [1s]" */
function diagValueDisplay(status: TDiagStatus, item: TDiagValue): string {
  if (!item.key.includes("[1s]")) return item.value;
  const rate = Number.parseFloat(item.value);
  if (Number.isNaN(rate)) return item.value;
  const limit = networkLimitBytes(status);
  const percent = limit && limit > 0 ? ` (${((rate / limit) * 100.0).toFixed(2)}%)` : "";
  return `${formatRate(rate)}${percent}`;
}

function diagUsagePercent(status: TDiagStatus): number | undefined {
  const free = diagValue(status, "Free [%]");
  if (free !== undefined) return 100.0 - Number.parseFloat(free);
  const max = diagValue(status, "Max [%]");
  if (max !== undefined) return Number.parseFloat(max);
  const avg = diagValue(status, "Avg [%]");
  if (avg !== undefined) return Number.parseFloat(avg);
  // network load: values are bytes/s per interface, compare against the limit from the message
  const limit = networkLimitBytes(status);
  if (limit && limit > 0) {
    const rates = (status.values || [])
      .filter((v) => v.key.includes("[1s]"))
      .map((v) => Number.parseFloat(v.value))
      .filter((v) => !Number.isNaN(v));
    if (rates.length > 0) return (Math.max(...rates) / limit) * 100.0;
  }
  return undefined;
}

function diagIcon(status: TDiagStatus, color: string): JSX.Element {
  const name = (status.name || "").toLowerCase();
  const sx = { fontSize: "inherit", color: color };
  if (name.includes("cpu")) return <DeveloperBoardIcon sx={sx} />;
  if (name.includes("memory")) return <MemoryIcon sx={sx} />;
  if (name.includes("hdd") || name.includes("disk")) return <StorageIcon sx={sx} />;
  return <NetworkCheckIcon sx={sx} />;
}

interface HostItemProps {
  provider: Provider;
  stopNodes: (nodeIdGlobals: string[]) => void;
  onDoubleClick: (event: React.MouseEvent, id: string) => void;
  nodeCount: number;
  nodeRunningCount: number;
  children: React.ReactNode;
}

export default function HostItem(props: HostItemProps): JSX.Element {
  const {
    provider,
    stopNodes = (): void => {},
    onDoubleClick = (): void => {},
    nodeCount,
    nodeRunningCount,
    ...children
  } = props;
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();

  const optionsTimeButton = ["ntpdate", "set date", "sync me to this date", "help"];
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [showHelpTime, setShowHelpTime] = useState<boolean>(false);
  const [openNtpdateDialog, setOpenNtpdateDialog] = useState<boolean>(false);
  const [colorizeHosts] = useSetting<boolean>("colorizeHosts");
  const [timeDiffThreshold] = useSetting<number>("timeDiffThreshold");
  // anchor for the options menu, which provides the time options independent of the time difference
  const [optionsAnchorEl, setOptionsAnchorEl] = useState<null | HTMLElement>(null);
  const [systemDiagnostics, setSystemDiagnostics] = useState<TDiagEntry[]>([]);

  async function updateTime(local = true): Promise<void> {
    if (provider) {
      const localProviders = rosCtx.getLocalProvider();
      if (localProviders.length === 0) {
        logCtx.error("localhost provider not found", "", "localhost provider not found");
        return;
      }
      if (local) {
        navCtx.openTerminal({
          type: CmdTypes.SET_TIME,
          providerId: localProviders[0].id,
          node: `set-date-localhost-${Date.now()}`,
          cmd: provider.id,
        });
      } else {
        navCtx.openTerminal({
          type: CmdTypes.SET_TIME,
          providerId: provider.id,
          node: `set-date-remote-${Date.now()}`,
          cmd: localProviders[0].id,
        });
      }
    }
  }

  function handleMenuTimeItemClick(_event, index): void {
    if (index === 0) {
      // set time using ntpdate
      setOpenNtpdateDialog(true);
    } else if (index === 1) {
      // set remote time using date
      if (provider) {
        updateTime(false);
      }
    } else if (index === 2) {
      // set local time using date
      updateTime(true);
    } else if (index === 3) {
      setShowHelpTime(true);
    }
  }

  useCustomEventListener(EVENT_SYSTEM_DIAGNOSTICS, (data: TEventDiagnostics) => {
    if (data.provider.id !== provider.id) return;
    const status = (data.diagnostics?.status || []) as unknown as TDiagStatus[];
    const now = Date.now();
    setSystemDiagnostics((prev) => {
      const merged = new Map<string, TDiagEntry>(prev.map((entry) => [entry.status.name, entry]));
      for (const item of status) {
        if (!isSystemDiagnostic(item)) continue;
        merged.set(item.name, { status: item, receivedAt: now });
      }
      // drop entries which are older than DIAG_MAX_AGE_MS
      return [...merged.values()].filter((entry) => now - entry.receivedAt <= DIAG_MAX_AGE_MS);
    });
  });

  // remove outdated entries even if no new diagnostics arrive (e.g. provider stopped sending)
  useEffect(() => {
    if (systemDiagnostics.length === 0) return undefined;
    const timer = setInterval(() => {
      const now = Date.now();
      setSystemDiagnostics((prev) => {
        const fresh = prev.filter((entry) => now - entry.receivedAt <= DIAG_MAX_AGE_MS);
        // keep the same reference if nothing changed to avoid unnecessary re-renders
        return fresh.length === prev.length ? prev : fresh;
      });
    }, 1000);
    return () => clearInterval(timer);
  }, [systemDiagnostics.length]);

  /**
   * Check if provider has master sync on
   */
  const getMasterSyncNode = useCallback(
    (providerId: string): RosNode | undefined => {
      const foundSyncNode = rosCtx.mapProviderRosNodes.get(providerId)?.find((node) => {
        return node.id.includes("/mas_sync") && node.status === RosNodeStatus.RUNNING;
      });
      return foundSyncNode;
    },
    [rosCtx.mapProviderRosNodes]
  );

  const toggleMasterSync = useCallback(
    (provider: Provider): void => {
      const syncNode: RosNode | undefined = getMasterSyncNode(provider.id);
      if (syncNode) {
        stopNodes([syncNode.idGlobal]);
      } else {
        rosCtx.startMasterSync(provider.connection.host, provider.rosVersion, provider.rosState?.masteruri);
      }
    },
    [getMasterSyncNode, stopNodes, rosCtx.startMasterSync]
  );

  /**
   * Get provider tags
   */
  function getProviderTags(provider: Provider): TTag[] {
    const tags: TTag[] = [];
    if (!provider.daemon) {
      tags.push({ id: "no-daemon", data: "No Daemon", tooltip: "", color: "red" });
    }
    if (!provider.discovery) {
      const rmwImplementation = provider.systemEnv.RMW_IMPLEMENTATION as string;
      if (rmwImplementation === "rmw_zenoh_cpp") {
        tags.push({
          id: "no-discovery-with-zenoh",
          data: "discovery issues",
          tooltip:
            "For zenoh, not all nodes can be assigned to hosts  and not all updates can be detected automatically.",
          color: "orange",
        });
      } else {
        tags.push({ id: "no-discovery", data: "No Discovery", tooltip: "", color: "orange" });
      }
    }
    return tags;
  }

  function formatTime(milliseconds): string {
    const sec = (milliseconds / 1000.0).toFixed(3);
    return `${sec}s`;
  }

  const getDiagnosticColor = useCallback((status: TDiagStatus): string => {
    if (status.level >= 3) return grey[600];
    if (status.level >= 2) return red[700];
    if (status.level === 1) return orange[500];
    // const usage = diagUsagePercent(status);
    // if (usage === undefined || Number.isNaN(usage)) return green[600];
    // if (usage >= 80) return red[700];
    // if (usage >= 60) return orange[500];
    return green[600];
  }, []);

  const generateDiagnosticsView = useCallback((): JSX.Element => {
    if (!provider.isAvailable() || systemDiagnostics.length === 0) return <></>;
    const now = Date.now();
    // show only values not older than 15s and with a real measurement or a non-OK state
    const visible = systemDiagnostics
      .filter((entry) => now - entry.receivedAt <= DIAG_MAX_AGE_MS)
      .map((entry) => entry.status)
      .filter((status) => hasDiagValues(status) || status.level > 0);
    if (visible.length === 0) return <></>;
    return (
      <Stack direction="row" alignItems="center" spacing="0.2em" sx={{ marginLeft: "0.3em", fontSize: "1rem" }}>
        {visible.map((status) => {
          const usage = diagUsagePercent(status);
          return (
            <Tooltip
              key={status.name}
              placement="bottom"
              disableInteractive
              title={
                <div>
                  <Typography fontWeight="bold" fontSize="inherit">
                    {status.name}
                    {usage !== undefined && !Number.isNaN(usage) ? `: ${usage.toFixed(1)}%` : ""}
                  </Typography>
                  {status.message && <Typography fontSize="inherit">{status.message}</Typography>}
                  {status.values?.map((item) => (
                    <Stack key={item.key} direction="row" spacing="0.2em">
                      <Typography fontSize="inherit" fontWeight="bold">
                        {item.key}:
                      </Typography>
                      <Typography fontSize="inherit">{diagValueDisplay(status, item)}</Typography>
                    </Stack>
                  ))}
                </div>
              }
            >
              <Box
                display="flex"
                alignItems="center"
                onClick={(event) => event.stopPropagation()}
                onDoubleClick={(event) => event.stopPropagation()}
              >
                {diagIcon(status, getDiagnosticColor(status))}
              </Box>
            </Tooltip>
          );
        })}
      </Stack>
    );
  }, [provider, systemDiagnostics]);

  const getHostStyle = useCallback(
    function getHostStyle(provider: Provider): object {
      if (colorizeHosts) {
        return {
          borderLeftStyle: "solid",
          borderLeftColor: rosCtx.providerColor(provider.id),
          borderLeftWidth: "0.6em",
          [`& .${treeItemClasses.content}`]: {
            paddingLeft: "8px",
          },
        };
      }
      return {
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: "8px",
        },
      };
    },
    [colorizeHosts, rosCtx.providerColor]
  );

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

  return (
    <StyledRootTreeItem
      itemId={provider.id}
      slotProps={
        {
          label: { onClick: handleLabelClick },
          content: { onClick: handleContentClick },
          iconContainer: { onClick: handleIconContainerClick },
        } as TreeItemSlotProps
      }
      sx={getHostStyle(provider)}
      onDoubleClick={(event) => onDoubleClick(event, provider.id)}
      label={
        <Box display="flex" alignItems="center" paddingLeft={0.0}>
          {provider.rosState.ros_version === "1" && (
            <Tooltip title="Toggle Master Sync" placement="bottom-start">
              <IconButton
                edge="start"
                aria-label="Toggle Master Sync"
                onClick={(event) => {
                  toggleMasterSync(provider);
                  event.stopPropagation();
                }}
              >
                <ChangeCircleOutlinedIcon
                  sx={{ color: getMasterSyncNode(provider.id) ? green[500] : grey[700], fontSize: "inherit" }}
                />
              </IconButton>
            </Tooltip>
          )}

          {Math.abs(provider.timeDiff) > timeDiffThreshold && (
            <Tooltip title={`Time not in sync for approx. ${formatTime(provider.timeDiff)}`} placement="right-end">
              <Box>
                <IconButton
                  edge="start"
                  aria-label={`Time not in sync for approx. ${formatTime(provider.timeDiff)}`}
                  onClick={(event) => {
                    setAnchorEl(anchorEl ? null : event.currentTarget);
                    event.stopPropagation();
                  }}
                  onDoubleClick={(event) => {
                    event.stopPropagation();
                  }}
                >
                  <WatchLaterIcon sx={{ color: orange[500] }} />
                </IconButton>
                <Popper
                  sx={{
                    zIndex: 999,
                  }}
                  open={Boolean(anchorEl)}
                  anchorEl={anchorEl}
                  transition
                  disablePortal={false}
                >
                  {({ TransitionProps, placement }) => (
                    <Grow
                      {...TransitionProps}
                      style={{
                        transformOrigin: placement === "bottom" ? "center top" : "center bottom",
                      }}
                    >
                      <Paper>
                        <ClickAwayListener onClickAway={() => setAnchorEl(null)}>
                          <MenuList id="set-time-button-menu" autoFocusItem>
                            {optionsTimeButton.map((option, index) => (
                              <MenuItem
                                key={option}
                                // disabled={index === 2}
                                // selected={index === X}
                                onClick={(event) => {
                                  setAnchorEl(null);
                                  handleMenuTimeItemClick(event, index);
                                }}
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
              </Box>
            </Tooltip>
          )}
          {provider.currentDelay > 3 ? (
            <LinkIcon sx={{ mr: 0.5, width: 20, color: red[700] }} />
          ) : provider.isAvailable() ? (
            <ComputerIcon sx={{ mr: 0.5, width: 20, color: grey[700] }} />
          ) : (
            <LinkOffIcon sx={{ mr: 0.5, width: 20, color: red[700] }} />
          )}

          {/* three dots button: provides the time options always, also if the time is in sync */}
          <Tooltip title="Host options" placement="bottom-start" disableInteractive>
            <IconButton
              size="small"
              aria-label="Host options"
              sx={{ mr: 0.5, padding: "2px" }}
              onClick={(event) => {
                setOptionsAnchorEl(optionsAnchorEl ? null : event.currentTarget);
                event.stopPropagation();
              }}
              onDoubleClick={(event) => {
                event.stopPropagation();
              }}
            >
              <MoreVertIcon sx={{ fontSize: "1rem", color: grey[700] }} />
            </IconButton>
          </Tooltip>
          <Menu
            id="host-item-options-menu"
            open={Boolean(optionsAnchorEl)}
            anchorEl={optionsAnchorEl}
            onClose={() => setOptionsAnchorEl(null)}
            anchorOrigin={{ vertical: "bottom", horizontal: "left" }}
            transformOrigin={{ vertical: "top", horizontal: "left" }}
            onClick={(event) => event.stopPropagation()}
            onDoubleClick={(event) => event.stopPropagation()}
          >
            {optionsTimeButton.map((option, index) => (
              <MenuItem
                key={`options-${option}`}
                dense
                onClick={(event) => {
                  setOptionsAnchorEl(null);
                  handleMenuTimeItemClick(event, index);
                }}
              >
                {option}
              </MenuItem>
            ))}
          </Menu>

          <Stack direction="row" display="flex" alignItems="center" sx={{ flexGrow: 1, userSelect: "none" }}>
            <Typography variant="body1" alignItems="center" marginRight={1}>
              {provider.name()}
            </Typography>
            {provider.isLocalHost && (
              <Typography variant="body2" color="grey">
                (localhost)
              </Typography>
            )}
            {generateDiagnosticsView()}

            <Typography variant="body1" alignItems="center" flexGrow={1} marginRight={1} />

            {nodeCount > 0 && (
              <Tooltip title="count of running / total nodes" disableInteractive>
                <Typography variant="body2" sx={{ marginLeft: 1 }}>
                  [{nodeRunningCount}/{nodeCount}]
                </Typography>
              </Tooltip>
            )}
            {getProviderTags(provider).map((tag: TTag) => (
              <Tooltip
                key={tag.id}
                sx={{ marginLeft: "0.1em" }}
                title={`${tag.tooltip}`}
                placement="left"
                disableInteractive
              >
                <Box>
                  {typeof tag.data === "string" ? (
                    <Tag
                      text={tag.data}
                      color={tag.color}
                      onClick={(event) => {
                        if (tag.onClick) {
                          tag.onClick(event);
                        }
                        event.stopPropagation();
                      }}
                      onDoubleClick={(event) => {
                        event.stopPropagation();
                      }}
                    />
                  ) : (
                    tag.data && <tag.data style={{ fontSize: "inherit", color: tag.color }} />
                  )}
                </Box>
              </Tooltip>
            ))}
          </Stack>

          <SetNTPDateDialog
            key="sync-time-menu"
            open={openNtpdateDialog}
            onClose={(value: string) => {
              if (value) {
                // execute the command in own terminal
                const id = `cmd-${generateUniqueId()}`;
                emitOpenComponent({
                  id: id,
                  title: `${provider?.name()}`,
                  closable: true,
                  component: LAYOUT_TABS.TERMINAL,
                  toNodeId: LAYOUT_TAB_SETS.BORDER_BOTTOM,
                  config: {
                    terminalConfig: {
                      id,
                      cmdType: CmdTypes.CMD,
                      providerId: provider.id,
                      host: provider.connection.host,
                      port: provider.connection.port,
                      node: "",
                      screen: "",
                      env: [],
                      cmd: value,
                    },
                  },
                });
              }
              setOpenNtpdateDialog(false);
            }}
            defaultCmd="sudo ntpdate -v -u -t 1"
          />

          <DateHelpDialog
            key="show-time-help"
            open={showHelpTime}
            onClose={() => {
              setShowHelpTime(false);
            }}
          />
        </Box>
      }
      {...children}
    />
  );
}
