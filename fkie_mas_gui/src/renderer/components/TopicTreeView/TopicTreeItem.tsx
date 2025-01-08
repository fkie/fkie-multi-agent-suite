import { EndpointInfo, TopicExtendedInfo } from "@/renderer/models";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Chip, Stack, Tooltip, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { LoggingContext } from "../../context/LoggingContext";
import { NavigationContext } from "../../context/NavigationContext";
import { SettingsContext } from "../../context/SettingsContext";
import { LAYOUT_TABS } from "../../pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../pages/NodeManager/layout/events";
import { removeDDSuid } from "../../utils/index";
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
  const [label, setLabel] = useState(topicInfo.name);
  // state variables to show/hide extended info
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selected, setSelected] = useState(false);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);

  const getHostStyle = () => {
    if (topicInfo.providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        alignItems: "center",
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(topicInfo.providerName),
        borderLeftWidth: "0.6em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  useEffect(() => {
    if (!rootPath) return;
    if (!topicInfo) return;

    if (topicInfo.name === rootPath) {
      setLabel(topicInfo.providerName);
    } else {
      setLabel(topicInfo.name.slice(rootPath.length + 1));
    }
  }, [rootPath, topicInfo.name, topicInfo]);

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
            <Stack spacing={1} direction="row" sx={getHostStyle()}>
              <Typography
                variant="body2"
                sx={{ fontWeight: "inherit" }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(label);
                    logCtx.success(`${label} copied!`);
                    e.stopPropagation();
                  }
                }}
              >
                {label}
              </Typography>
              {/* {requestData && <CircularProgress size="1em" />} */}
              {topicInfo &&
                topicInfo.subscribers.filter((sub) => sub.incompatible_qos && sub.incompatible_qos.length > 0).length >
                  0 && (
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
              {topicInfo.publishers.map((item: EndpointInfo) => {
                const pubNodeName = removeDDSuid(item.node_id);
                return (
                  <Stack key={item.node_id} paddingLeft={3} direction="row">
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        const id: string = `${topicInfo.providerId}${item.node_id.replaceAll("/", "#")}`;
                        navCtx.setSelectedNodes([id]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default")
                        );
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
              {topicInfo.subscribers.map((item: EndpointInfo) => {
                const subNodeName = removeDDSuid(item.node_id);
                return (
                  <Stack
                    key={item.node_id}
                    paddingLeft={3}
                    spacing={1}
                    direction="row"
                    justifyItems="center"
                    alignItems="center"
                  >
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        const id: string = `${topicInfo.providerId}${item.node_id.replaceAll("/", "#")}`;
                        navCtx.setSelectedNodes([id]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default")
                        );
                      }}
                    >
                      {subNodeName}
                    </Typography>
                    {item.incompatible_qos && item.incompatible_qos.length > 0 && (
                      <Tooltip
                        title={`Incompatible QoS: ${JSON.stringify(item.incompatible_qos)}`}
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
