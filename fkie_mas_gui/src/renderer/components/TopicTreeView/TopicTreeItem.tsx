import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Chip, Menu, MenuItem, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { useCallback, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { IncompatibleQos, TopicExtendedInfo } from "@/renderer/models";
import { durabilityToString, livelinessToString, reliabilityToString } from "@/renderer/models/RosQos";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { removeDDSuid } from "@/renderer/utils/index";
import { colorFromHostname } from "../UI/Colors";

interface TopicTreeItemProps {
  itemId: string;
  rootPath: string;
  topicInfo: TopicExtendedInfo;
  selectedItem: string | null;
  selected: boolean; // ignore prop from parent, we use our own selection logic
  depth: number;
  onSelect: () => void;
}

export default function TopicTreeItem({
  itemId,
  rootPath,
  topicInfo,
  selectedItem,
  depth,
  onSelect,
}: TopicTreeItemProps): JSX.Element {
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const settingsCtx = useSettingsContext();

  const [name, setName] = useState("");
  const [namespace, setNamespace] = useState("");
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selectedLocal, setSelectedLocal] = useState(false);
  const [incompatibleQos, setIncompatibleQos] = useState<IncompatibleQos[]>([]);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  // update colorize setting when context value changes
  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  // Original logic: control selection / extended info via selectedItem
  useEffect(() => {
    if (selectedItem !== itemId) {
      if (selectedLocal) {
        setSelectedLocal(false);
        setIgnoreNextClick(true);
      }
    } else {
      if (selectedLocal) {
        // already selected -> toggle extended info
        setShowExtendedInfo((prev) => !prev);
      } else {
        // first selection -> only set selected
        setSelectedLocal(true);
      }
    }
  }, [selectedItem, itemId]);

  // parse topic name, namespace, incompatible qos
  useEffect(() => {
    const nameParts = topicInfo.name.split("/");
    setName(`${nameParts.pop()}`);
    setNamespace(rootPath ? `${rootPath}/` : "");
    const inQos: IncompatibleQos[] = [];

    const subscribersWithInCQos = topicInfo?.subscribers.filter(
      (sub) => sub.info.incompatible_qos && sub.info.incompatible_qos.length > 0
    );
    for (const sub of subscribersWithInCQos) {
      if (sub.info.incompatible_qos) {
        for (const item of sub.info.incompatible_qos) {
          inQos.push(item);
        }
      }
    }
    setIncompatibleQos(inQos);
  }, [topicInfo, rootPath]);

  const getHostStyle = useCallback(
    (providerName: string): object => {
      // If host coloring is enabled and providerName is set, add a colored left border for the host
      if (providerName && colorizeHosts) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(providerName),
          borderLeftWidth: "0.6em",
        };
      }
      // default layout if no coloring
      return { flexGrow: 1, alignItems: "center", paddingLeft: 0 };
    },
    [colorizeHosts]
  );

  function addQosValue(
    value: string | number | undefined,
    nodeId: string,
    type: string,
    values: { [key: string]: { nodeId: string; type: string }[] }
  ): void {
    if (value !== undefined) {
      const val: string = `${value}`;
      if (!values[val]) {
        values[val] = [];
      }
      values[val].push({
        nodeId: nodeId,
        type: type,
      });
    }
  }

  const createQosItem = useCallback(
    (
      label: string,
      qosKey: "reliability" | "durability" | "liveliness",
      converter: (v: number) => string
    ): JSX.Element => {
      const values: { [key: string]: { nodeId: string; type: string }[] } = {};

      // collect QoS values from publishers
      for (const pub of topicInfo.publishers) addQosValue(pub.info.qos?.[qosKey], pub.info.node_id, "pub", values);
      // collect QoS values from subscribers
      for (const sub of topicInfo.subscribers) addQosValue(sub.info.qos?.[qosKey], sub.info.node_id, "sub", values);

      let index = 0;
      return (
        <Stack direction="row" spacing="0.5em" paddingLeft="1em">
          <Typography variant="caption" color="inherit">
            {label}:
          </Typography>
          {Object.entries(values).map(([key, value]) => {
            index += 1;
            return (
              <Stack key={`${qosKey}-${key}`} direction="row" spacing="0.2em">
                {index > 1 && (
                  <Typography variant="body2" color="inherit">
                    +
                  </Typography>
                )}
                <Typography variant="body2" color="inherit">
                  {converter(Number.parseInt(key))}
                </Typography>
                {index > 1 && (
                  <Typography variant="body2" color="inherit">
                    {JSON.stringify(value.map((item) => removeDDSuid(item.nodeId)))}
                  </Typography>
                )}
              </Stack>
            );
          })}
        </Stack>
      );
    },
    [topicInfo]
  );

  const handleContextMenu = useCallback(
    (event: React.MouseEvent) => {
      event.preventDefault();
      event.stopPropagation();
      setContextMenu(
        contextMenu === null
          ? {
              mouseX: event.clientX + 2,
              mouseY: event.clientY - 6,
            }
          : null
      );
    },
    [contextMenu]
  );

  const handleCloseMenu = useCallback((event) => {
    setContextMenu(null);
    event.stopPropagation();
  }, []);

  const handleDoubleClickCopy = useCallback(
    (e: React.MouseEvent, value: string, label: string) => {
      // copy on double click
      if (e.detail === 2) {
        navigator.clipboard.writeText(value);
        logCtx.success(`${value} copied!`, "", `${label} copied`);
        e.stopPropagation();
      }
    },
    [logCtx]
  );

  const handleRowClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    // Original logic: first click only selects, second click toggles extendedInfo
    if (ignoreNextClick) {
      setIgnoreNextClick(false);
    } else {
      setShowExtendedInfo((prev) => !prev);
    }
    onSelect();
  };

  const isSelected = selectedItem === itemId;
  const lineKeys = Array.from({ length: depth }, (_, i) => `${itemId}-line-${i}`);

  return (
    <Box
      sx={{
        display: "flex",
        alignItems: "stretch",
        cursor: "pointer",
        borderRadius: 0,
        bgcolor: isSelected ? "var(--color-select-bg)" : "transparent",
        color: "text.secondary",
      }}
      onClick={handleRowClick}
      onContextMenu={handleContextMenu}
    >
      {/* Line column: one box per depth level */}
      {lineKeys.map((key) => (
        <Box
          key={key}
          sx={{
            ml: 0.9,
            width: "0.9em",
            borderLeft: `1px dashed ${alpha(grey[600], 0.4)}`,
          }}
        />
      ))}

      {/* Content column (header + extended info + context menu) */}
      <Box sx={{ flexGrow: 1, py: 0.2, pr: 1 }}>
        {/* Header row */}
        <Box
          sx={{
            ml: 0.7,
            display: "flex",
            alignItems: "center",
            py: 0.2,
            pr: 0,
          }}
          onClick={handleRowClick}
        >
          <Stack spacing={1} direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
            <Stack direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
              <Typography
                variant="body2"
                sx={{ fontSize: "inherit", userSelect: "none" }}
                onClick={(e) => handleDoubleClickCopy(e, topicInfo.name, "topic name")}
              >
                {namespace}
              </Typography>
              <Typography
                variant="body2"
                sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}
                onClick={(e) => handleDoubleClickCopy(e, topicInfo.name, "topic name")}
              >
                {name}
              </Typography>
              {incompatibleQos.length > 0 && (
                <Tooltip title="There are subscribers with incompatible QoS" placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
                </Tooltip>
              )}
            </Stack>
          </Stack>
          <Stack direction="row" spacing={1} sx={{ alignItems: "center" }}>
            {topicInfo.msgType && (
              <Typography
                variant="caption"
                color="inherit"
                padding={0.2}
                onClick={(e) => handleDoubleClickCopy(e, topicInfo.msgType, "topic type")}
              >
                {topicInfo.msgType}
              </Typography>
            )}
            {topicInfo && (
              <Stack direction="row" spacing={1}>
                <Chip
                  size="small"
                  title="publishers"
                  color={topicInfo.publishers.length > 0 ? "default" : "warning"}
                  label={topicInfo.publishers.length}
                />
                <Chip
                  size="small"
                  title="subscribers"
                  color={topicInfo.subscribers.length > 0 ? "default" : "warning"}
                  label={topicInfo.subscribers.length}
                />
              </Stack>
            )}
          </Stack>
        </Box>

        {/* Extended info – remains visible until topic is clicked again */}
        {showExtendedInfo && topicInfo && (
          <Stack paddingLeft={3}>
            {/* Publisher / Subscriber / QoS details */}
            <Typography fontWeight="bold" fontSize="small">
              Publisher [{topicInfo.publishers.length}]:
            </Typography>
            {topicInfo.publishers.map((item: EndpointExtendedInfo) => {
              const pubNodeName = removeDDSuid(item.info.node_id);
              return (
                <Stack
                  key={`${item.providerId}-${item.info.node_id}`}
                  paddingLeft={3}
                  direction="row"
                  sx={getHostStyle(item.providerName)}
                >
                  <Typography
                    fontSize="small"
                    onClick={(event) => {
                      event.stopPropagation();
                      const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                      navCtx.setSelectedProviders([]);
                      navCtx.setSelectedNodes([id], false);
                      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.DETAILS, "default"));
                    }}
                  >
                    {pubNodeName}
                  </Typography>
                </Stack>
              );
            })}
            <Typography fontWeight="bold" fontSize="small">
              Subscriber [{topicInfo.subscribers.length}]:
            </Typography>
            {topicInfo.subscribers.map((item: EndpointExtendedInfo) => {
              const subNodeName = removeDDSuid(item.info.node_id);
              return (
                <Stack
                  key={`${item.providerId}-${item.info.node_id}`}
                  paddingLeft={3}
                  direction="row"
                  sx={getHostStyle(item.providerName)}
                >
                  <Typography
                    fontSize="small"
                    onClick={(event) => {
                      event.stopPropagation();
                      const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                      navCtx.setSelectedProviders([]);
                      navCtx.setSelectedNodes([id], false);
                      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.DETAILS, "default"));
                    }}
                  >
                    {subNodeName}
                  </Typography>
                  {item.info.incompatible_qos && item.info.incompatible_qos.length > 0 && (
                    <Tooltip
                      title={`Incompatible QoS: ${JSON.stringify(item.info.incompatible_qos)}`}
                      placement="right"
                      disableInteractive
                    >
                      <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                    </Tooltip>
                  )}
                </Stack>
              );
            })}
            {topicInfo?.hasQos && (
              <>
                <Typography fontWeight="bold" fontSize="small">
                  Qos profiles:
                </Typography>
                {createQosItem("Reliability", "reliability", reliabilityToString)}
                {createQosItem("Durability", "durability", durabilityToString)}
                {createQosItem("Liveliness", "liveliness", livelinessToString)}
              </>
            )}
          </Stack>
        )}

        {/* Context menu */}
        <Menu
          open={contextMenu != null}
          onClose={handleCloseMenu}
          anchorReference="anchorPosition"
          anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
        >
          <MenuItem
            sx={{ fontSize: "0.8em" }}
            onClick={async (event) => {
              navigator.clipboard.writeText(topicInfo.name);
              handleCloseMenu(event);
            }}
          >
            Copy topic name
          </MenuItem>
          <MenuItem
            sx={{ fontSize: "0.8em" }}
            onClick={async (event) => {
              navigator.clipboard.writeText(topicInfo.msgType || "");
              handleCloseMenu(event);
            }}
          >
            Copy message type
          </MenuItem>
        </Menu>
      </Box>
    </Box>
  );
}
