import { TopicExtendedInfo } from "@/renderer/models";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Chip, Stack, Tooltip, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { LoggingContext } from "../../context/LoggingContext";
import { NavigationContext } from "../../context/NavigationContext";
import { SettingsContext } from "../../context/SettingsContext";
import { LAYOUT_TABS } from "../../pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../pages/NodeManager/layout/events";
import { normalizeNameWithPrefix, removeDDSuid } from "../../utils/index";
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
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);

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
    const nameParts = normalizeNameWithPrefix(topicInfo.name, rootPath).split("/");
    setName(`${nameParts.pop()}`);
    const ns = nameParts.join("/");
    setNamespace(ns ? `${ns}/` : rootPath ? "" : "/");
  }, []);

  return (
    <StyledTreeItem
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
      label={
        <Stack direction="column">
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
              </Stack>
              {/* {requestData && <CircularProgress size="1em" />} */}
              {topicInfo &&
                topicInfo.subscribers.filter((sub) => sub.info.incompatible_qos && sub.info.incompatible_qos.length > 0)
                  .length > 0 && (
                  <Tooltip title={`There are subscribers with incompatible QoS`} placement="right" disableInteractive>
                    <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                  </Tooltip>
                )}
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
            </Stack>
          )}
        </Stack>
      }
    />
  );
});

export default TopicTreeItem;
