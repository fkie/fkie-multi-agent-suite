import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Chip, Menu, MenuItem, Stack, Tooltip, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import { NavigationContext } from "@/renderer/context/NavigationContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { IncompatibleQos, TopicExtendedInfo } from "@/renderer/models";
import { durabilityToString, livelinessToString, reliabilityToString } from "@/renderer/models/RosQos";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { removeDDSuid } from "@/renderer/utils/index";
import { colorFromHostname } from "../UI/Colors";
import StyledTreeItem from "./StyledTreeItem";

interface TopicTreeItemProps {
  itemId: string;
  rootPath: string;
  topicInfo: TopicExtendedInfo;
  selectedItem: string | null;
}

const TopicTreeItem = forwardRef<HTMLDivElement, TopicTreeItemProps>(function TopicTreeItem(props, ref) {
  const { itemId, rootPath, topicInfo, selectedItem } = props;

  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const [name, setName] = useState<string>("");
  const [namespace, setNamespace] = useState("");
  // state variables to show/hide extended info
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selected, setSelected] = useState(false);
  const [incompatibleQos, setIncompatibleQos] = useState<IncompatibleQos[]>([]);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);

  const getHostStyle = useCallback(
    function getHostStyle(providerName: string): object {
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(providerName),
          borderLeftWidth: "0.6em",
        };
      }
      return { flexGrow: 1, alignItems: "center" };
    },
    [settingsCtx.changed]
  );

  useEffect(() => {
    // update state variables to show/hide extended info
    if (selectedItem !== itemId) {
      if (selected) {
        setSelected(false);
        setIgnoreNextClick(true);
      }
    } else {
      if (selected) {
        setShowExtendedInfo(!showExtendedInfo);
      }
      setSelected(true);
    }
  }, [selectedItem]);

  useEffect(() => {
    const nameParts = topicInfo.name.split("/");
    setName(`${nameParts.pop()}`);
    setNamespace(rootPath ? `${rootPath}/` : rootPath ? "" : "");
    const inQos: IncompatibleQos[] = [];
    topicInfo?.subscribers
      .filter((sub) => sub.info.incompatible_qos && sub.info.incompatible_qos.length > 0)
      .forEach((sub) => {
        sub.info.incompatible_qos?.forEach((item) => inQos.push(item));
      });
    setIncompatibleQos(inQos);
  }, [topicInfo]);

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

  const createReliabilityItem = useMemo((): JSX.Element => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    topicInfo.publishers.forEach((pub) => {
      addQosValue(pub.info.qos?.reliability, pub.info.node_id, "pub", values);
    });
    topicInfo.subscribers.forEach((sub) => {
      addQosValue(sub.info.qos?.reliability, sub.info.node_id, "sub", values);
    });
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" color="inherit">
          Reliability:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-reliability-${key}`} direction="row" spacing={"0.2em"}>
              {index > 1 && (
                <Typography variant="body2" color="inherit">
                  +
                </Typography>
              )}
              <Typography variant="body2" color="inherit">
                {reliabilityToString(parseInt(key))}
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
  }, [topicInfo]);

  const createDurabilityItem = useMemo((): JSX.Element => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    topicInfo.publishers.forEach((pub) => {
      addQosValue(pub.info.qos?.durability, pub.info.node_id, "pub", values);
    });
    topicInfo.subscribers.forEach((sub) => {
      addQosValue(sub.info.qos?.durability, sub.info.node_id, "sub", values);
    });
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" color="inherit">
          Durability:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-durability-${key}`} direction="row" spacing={"0.2em"}>
              <Typography variant="body2" color="inherit">
                {durabilityToString(parseInt(key))}
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
  }, [topicInfo]);

  const createLivelinessItem = useMemo((): JSX.Element => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    topicInfo.publishers.forEach((pub) => {
      addQosValue(pub.info.qos?.liveliness, pub.info.node_id, "pub", values);
    });
    topicInfo.subscribers.forEach((sub) => {
      addQosValue(sub.info.qos?.liveliness, sub.info.node_id, "sub", values);
    });
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" color="inherit">
          Liveliness:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-liveliness-${key}`} direction="row" spacing={"0.2em"}>
              <Typography variant="body2" color="inherit">
                {livelinessToString(parseInt(key))}
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
  }, [topicInfo]);

  return (
    <StyledTreeItem
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
      label={
        <Stack
          direction="column"
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
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
              // p: 0.3,
              pr: 0,
            }}
            onClick={() => {
              if (ignoreNextClick) {
                setIgnoreNextClick(false);
              } else {
                setShowExtendedInfo(!showExtendedInfo);
              }
            }}
          >
            <Stack spacing={1} direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
              <Stack direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
                <Typography
                  variant="body2"
                  sx={{ fontSize: "inherit", userSelect: "none" }}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(topicInfo.name);
                      logCtx.success(`${topicInfo.name} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {namespace}
                </Typography>
                <Typography
                  variant="body2"
                  sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(topicInfo.name);
                      logCtx.success(`${topicInfo.name} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {name}
                </Typography>
                {incompatibleQos.length > 0 && (
                  <Tooltip title={`There are subscribers with incompatible QoS`} placement="right" disableInteractive>
                    <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
                  </Tooltip>
                )}
              </Stack>
              {/* {requestData && <CircularProgress size="1em" />} */}
            </Stack>
            <Stack
              direction="row"
              spacing={1}
              sx={{
                alignItems: "center",
              }}
            >
              {topicInfo.msgType && (
                <Typography
                  variant="caption"
                  color="inherit"
                  padding={0.2}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(topicInfo.msgType);
                      logCtx.success(`${topicInfo.msgType} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {topicInfo.msgType}
                </Typography>
              )}
              {topicInfo && (
                <Stack direction="row" spacing={1}>
                  <Chip
                    size="small"
                    title="publishers"
                    // showZero={true}
                    color={topicInfo.publishers.length > 0 ? "default" : "warning"}
                    label={topicInfo.publishers.length}
                  />

                  <Chip
                    size="small"
                    title="subscribers"
                    // showZero={true}
                    color={topicInfo.subscribers.length > 0 ? "default" : "warning"}
                    label={topicInfo.subscribers.length}
                  />
                </Stack>
              )}
            </Stack>
          </Box>
          {showExtendedInfo && topicInfo && (
            <Stack paddingLeft={3}>
              <Typography fontWeight="bold" fontSize="small">
                Publisher [{topicInfo.publishers.length}]:
              </Typography>
              {topicInfo.publishers.map((item: EndpointExtendedInfo) => {
                const pubNodeName = removeDDSuid(item.info.node_id);
                return (
                  <Stack key={item.info.node_id} paddingLeft={3} direction="row" sx={getHostStyle(item.providerName)}>
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                        navCtx.setSelectedNodes([id]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                      }}
                    >
                      {pubNodeName}
                    </Typography>
                    {/* <CopyButton value={pubNodeName} fontSize="0.7em" /> */}
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
                    key={item.info.node_id}
                    paddingLeft={3}
                    spacing={1}
                    direction="row"
                    justifyItems="center"
                    alignItems="center"
                    sx={getHostStyle(item.providerName)}
                  >
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                        navCtx.setSelectedNodes([id]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
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
                    {/* <CopyButton value={subNodeName} fontSize="0.7em" /> */}
                  </Stack>
                );
              })}
              {topicInfo?.hasQos && (
                <Typography fontWeight="bold" fontSize="small">
                  Qos profiles:
                </Typography>
              )}
              {topicInfo?.hasQos && createReliabilityItem}
              {topicInfo?.hasQos && createDurabilityItem}
              {topicInfo?.hasQos && createLivelinessItem}
            </Stack>
          )}
          <Menu
            open={contextMenu != null}
            onClose={() => setContextMenu(null)}
            anchorReference="anchorPosition"
            anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
          >
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(topicInfo.name);
                setContextMenu(null);
              }}
            >
              Copy topic name
            </MenuItem>
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(topicInfo.msgType);
                setContextMenu(null);
              }}
            >
              Copy message type
            </MenuItem>
          </Menu>
        </Stack>
      }
    />
  );
});

export default TopicTreeItem;
