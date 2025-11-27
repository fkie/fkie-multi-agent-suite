import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import RosContext from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { RosTopic, RosTopicId, TopicExtendedInfo } from "@/renderer/models";
import { durabilityToString, livelinessToString, reliabilityToString } from "@/renderer/models/RosQos";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { EVENT_PROVIDER_ROS_TOPICS } from "@/renderer/providers/eventTypes";
import { removeDDSuid } from "@/renderer/utils";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import { Button, Chip, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { ReactNode, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { colorFromHostname, CopyButton } from "../UI";

type TopicDetailsItemsProps = {
  providerId: string | undefined;
  topicId: RosTopicId;
  showConnections: boolean;
  nodeName: string;
};

export default function TopicDetailsItem(props: TopicDetailsItemsProps): JSX.Element {
  const { providerId, topicId, showConnections = true, nodeName = "" } = props;

  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [topicInfo, setTopicInfo] = useState<TopicExtendedInfo | undefined>(undefined);
  const [showInfo, setShowInfo] = useState<boolean>(false);
  const [hasIncompatibleQos, setHasIncompatibleQos] = useState<boolean>(false);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx, settingsCtx.changed]);

  function onEchoClick(topic: TopicExtendedInfo, external: boolean = false, openInTerminal: boolean = false): void {
    navCtx.openSubscriber(providerId || "", topic.name, true, false, external, openInTerminal);
  }

  function onPublishClick(topic: TopicExtendedInfo, external: boolean = false, openInTerminal: boolean = false): void {
    navCtx.startPublisher(providerId || "", topic.name, topic.msgType, external, openInTerminal);
  }

  function updateTopicList(): void {
    if (providerId) {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        const rosTopic: RosTopic | undefined = provider?.getTopic(topicId);
        if (!rosTopic) {
          setTopicInfo(undefined);
          return;
        }
        const newTopicInfo: TopicExtendedInfo = new TopicExtendedInfo(rosTopic);
        // Get topics from the ros node list of each provider.
        for (const provider of rosCtx.providers) {
          for (const rosNode of provider.rosNodes) {
            // add node to publisher and subscriber
            newTopicInfo.add(rosNode);
          }
        }
        setTopicInfo(newTopicInfo);
        // check for incompatible qos
        let foundIncompatibleQos = false;
        for (const sub of rosTopic.subscriber || []) {
          if (sub.incompatible_qos) {
            foundIncompatibleQos = true;
            break;
          }
        }
        for (const pub of rosTopic.publisher || []) {
          if (pub.incompatible_qos) {
            foundIncompatibleQos = true;
            break;
          }
        }
        setHasIncompatibleQos(foundIncompatibleQos);
      }
    }
  }

  const getHostStyle = useCallback(
    function getHostStyle(providerName: string): object {
      if (providerName && colorizeHosts) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(providerName),
          borderLeftWidth: "0.5em",
        };
      }
      return { flexGrow: 1, alignItems: "center" };
    },
    [settingsCtx.changed]
  );

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, () => {
    updateTopicList();
  });

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    updateTopicList();
  }, [providerId, rosCtx.providers]);

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

  const createReliabilityItem: (topicInfo: TopicExtendedInfo) => ReactNode = (topicInfo) => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    for (const pub of topicInfo.publishers) {
      addQosValue(pub.info.qos?.reliability, pub.info.node_id, "pub", values);
    }
    for (const sub of topicInfo.subscribers) {
      addQosValue(sub.info.qos?.reliability, sub.info.node_id, "sub", values);
    }
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" fontFamily="monospace" color="inherit">
          Reliability:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-reliability-${key}`} direction="row" spacing={"0.2em"}>
              {index > 1 && (
                <Typography variant="body2" fontFamily="monospace" color="inherit">
                  +
                </Typography>
              )}
              <Typography variant="body2" fontFamily="monospace" color="inherit">
                {reliabilityToString(Number.parseInt(key))}
              </Typography>
              {index > 1 && (
                <Typography variant="body2" fontFamily="monospace" color="inherit">
                  {JSON.stringify(value.map((item) => removeDDSuid(item.nodeId)))}
                </Typography>
              )}
            </Stack>
          );
        })}
      </Stack>
    );
  };

  const createDurabilityItem: (topicInfo: TopicExtendedInfo) => ReactNode = (topicInfo) => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    for (const pub of topicInfo.publishers) {
      addQosValue(pub.info.qos?.durability, pub.info.node_id, "pub", values);
    }
    for (const sub of topicInfo.subscribers) {
      addQosValue(sub.info.qos?.durability, sub.info.node_id, "sub", values);
    }
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" fontFamily="monospace" color="inherit">
          Durability:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-durability-${key}`} direction="row" spacing={"0.2em"}>
              <Typography variant="body2" fontFamily="monospace" color="inherit">
                {durabilityToString(Number.parseInt(key))}
              </Typography>
              {index > 1 && (
                <Typography variant="body2" fontFamily="monospace" color="inherit">
                  {JSON.stringify(value.map((item) => removeDDSuid(item.nodeId)))}
                </Typography>
              )}
            </Stack>
          );
        })}
      </Stack>
    );
  };

  const createLivelinessItem: (topicInfo: TopicExtendedInfo) => ReactNode = (topicInfo) => {
    const values: { [key: string]: { nodeId: string; type: string }[] } = {};

    for (const pub of topicInfo.publishers) {
      addQosValue(pub.info.qos?.liveliness, pub.info.node_id, "pub", values);
    }
    for (const sub of topicInfo.subscribers) {
      addQosValue(sub.info.qos?.liveliness, sub.info.node_id, "sub", values);
    }
    let index = 0;
    return (
      <Stack direction="row" spacing={"0.5em"} paddingLeft={"1em"}>
        <Typography variant="caption" fontFamily="monospace" color="inherit">
          Liveliness:
        </Typography>
        {Object.entries(values).map(([key, value]) => {
          index += 1;
          return (
            <Stack key={`qos-liveliness-${key}`} direction="row" spacing={"0.2em"}>
              <Typography variant="body2" fontFamily="monospace" color="inherit">
                {livelinessToString(Number.parseInt(key))}
              </Typography>
              {index > 1 && (
                <Typography variant="body2" fontFamily="monospace" color="inherit">
                  {JSON.stringify(value.map((item) => removeDDSuid(item.nodeId)))}
                </Typography>
              )}
            </Stack>
          );
        })}
      </Stack>
    );
  };

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const createInfo = useMemo(() => {
    if (!topicInfo) return <></>;
    let topicName = topicId.name;
    const fullNodeName = nodeName ? `${nodeName}/` : "";
    if (fullNodeName && topicName.startsWith(fullNodeName)) {
      topicName = topicName.replace(fullNodeName, "");
    }

    return (
      <Stack direction="row" alignItems="center" spacing={0}>
        <Stack
          key={`pub-sub-${topicInfo.id}`}
          alignItems="center"
          direction="row"
          margin={0}
          spacing={"0.1em"}
          style={{ display: "flex", flexGrow: 1, borderBottom: `1px solid ${alpha(grey[600], 0.4)}` }}
        >
          <Chip
            size="small"
            onClick={(event) => {
              onPublishClick(
                topicInfo,
                event.nativeEvent.shiftKey,
                event.nativeEvent.ctrlKey && event.nativeEvent.shiftKey
              );
            }}
            avatar={
              <Tooltip
                title={
                  <Stack padding={0} margin={0}>
                    <Typography fontWeight="bold" fontSize="inherit">
                      Create a publisher
                    </Typography>
                    <Stack direction="row" spacing={"0.2em"}>
                      <Typography fontWeight={"bold"} fontSize={"inherit"}>
                        Shift:
                      </Typography>
                      <Typography fontSize={"inherit"}>alternative open location</Typography>
                    </Stack>
                    <Stack direction="row" spacing={"0.2em"}>
                      <Typography fontWeight={"bold"} fontSize={"inherit"}>
                        Ctrl+Shift:
                      </Typography>
                      <Typography fontSize={"inherit"}>open in a terminal</Typography>
                    </Stack>
                  </Stack>
                }
                placement="left"
                enterDelay={tooltipDelay}
                disableInteractive
              >
                <PlayArrowRoundedIcon style={{ padding: 1, color: "#09770fff" }} fontSize="inherit" />
              </Tooltip>
            }
            label={
              showConnections && (
                <Typography
                  fontSize="inherit"
                  color={(topicInfo.publishers || []).length === 0 ? "warning" : "default"}
                >
                  {topicInfo.publishers ? topicInfo.publishers.length : 0}
                </Typography>
              )
            }
          />
          <Chip
            size="small"
            onClick={(event) => {
              onEchoClick(
                topicInfo,
                event.nativeEvent.shiftKey,
                event.nativeEvent.ctrlKey && event.nativeEvent.shiftKey
              );
            }}
            avatar={
              <Tooltip
                title={
                  <Stack padding={0} margin={0}>
                    <Typography fontWeight="bold" fontSize="inherit">
                      Create a subscriber
                    </Typography>
                    <Stack direction="row" spacing={"0.2em"}>
                      <Typography fontWeight={"bold"} fontSize={"inherit"}>
                        Shift:
                      </Typography>
                      <Typography fontSize={"inherit"}>alternative open location</Typography>
                    </Stack>
                    <Stack direction="row" spacing={"0.2em"}>
                      <Typography fontWeight={"bold"} fontSize={"inherit"}>
                        Ctrl+Shift:
                      </Typography>
                      <Typography fontSize={"inherit"}>open in a terminal</Typography>
                    </Stack>
                  </Stack>
                }
                placement="left"
                enterDelay={tooltipDelay}
                disableInteractive
              >
                <ChatBubbleOutlineIcon
                  style={{ paddingLeft: 3, paddingRight: 0, color: "#6c50e9ff" }}
                  fontSize="inherit"
                />
              </Tooltip>
            }
            label={
              showConnections && (
                <Typography fontSize="inherit" color={(topicInfo.subscribers || []).length > 0 ? "default" : "warning"}>
                  {topicInfo.subscribers ? topicInfo.subscribers.length : 0}
                </Typography>
              )
            }
          />
          {hasIncompatibleQos && (
            <Tooltip title={"There are subscribers with incompatible QoS"} placement="right" disableInteractive>
              <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
            </Tooltip>
          )}
          <Button
            size="small"
            style={{
              marginLeft: "0.2em",
              textTransform: "none",
              justifyContent: "left",
            }}
            onClick={() => setShowInfo((prev) => !prev)}
            onDoubleClick={() => {
              navigator.clipboard.writeText(topicId.name);
              logCtx.info(`${topicId.name} copied`, "", `${topicId.name} copied`);
            }}
          >
            {`${topicName}`}
          </Button>
          {showInfo && <CopyButton value={topicId.name} fontSize="0.7em" />}
        </Stack>
      </Stack>
    );
  }, [topicInfo, showConnections, showInfo]);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const createExtendedInfo = useMemo(() => {
    if (!topicInfo) return <></>;
    return (
      <Stack
        key={`info-${topicInfo.id}`}
        style={{ marginLeft: 15, paddingLeft: 5, borderLeft: `1px dashed ${alpha(grey[600], 0.4)}` }}
      >
        <Stack direction="row" alignItems="center" spacing="0.3em">
          <Typography fontWeight="500" fontStyle="italic" fontSize="small">
            Type:
          </Typography>
          <Typography fontSize="small">{topicInfo.msgType}</Typography>
          <CopyButton value={topicInfo.msgType} fontSize="0.7em" />
        </Stack>
        <Typography fontWeight="500" fontStyle="italic" fontSize="small">
          Publisher [{topicInfo.publishers?.length || 0}]:
        </Typography>
        {topicInfo.publishers?.map((item: EndpointExtendedInfo) => {
          const pubNodeName = removeDDSuid(item.info.node_id);
          return (
            <Stack
              key={item.info.node_id}
              paddingLeft={"0.5em"}
              alignItems="center"
              direction="row"
              spacing="0.5em"
              style={getHostStyle(item.providerName)}
            >
              {item.info.incompatible_qos && item.info.incompatible_qos.length > 0 && (
                <Tooltip
                  title={`Incompatible QoS: ${JSON.stringify(item.info.incompatible_qos)}`}
                  placement="right"
                  disableInteractive
                >
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                </Tooltip>
              )}
              <Button
                size="small"
                style={{
                  marginLeft: "0.3em",
                  textTransform: "none",
                  justifyContent: "left",
                  padding: 0,
                  color: "#09770fff",
                }}
                onClick={() => {
                  const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                  navCtx.setSelectedNodes([id], true);
                  // inform details panel tab about selected nodes by user
                  emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.DETAILS, "default"));
                }}
              >
                {pubNodeName}
              </Button>
              <CopyButton value={pubNodeName} fontSize="0.7em" />
            </Stack>
          );
        })}
        <Typography fontWeight="500" fontStyle="italic" fontSize="small">
          Subscriber [{topicInfo.subscribers.length || 0}]:
        </Typography>
        {topicInfo.subscribers.map((item: EndpointExtendedInfo) => {
          const subNodeName = removeDDSuid(item.info.node_id);
          return (
            <Stack
              key={item.info.node_id}
              paddingLeft={"0.5em"}
              alignItems="center"
              direction="row"
              spacing="0.5em"
              style={getHostStyle(item.providerName)}
            >
              {item.info.incompatible_qos && item.info.incompatible_qos.length > 0 && (
                <Tooltip
                  title={`Incompatible QoS: ${JSON.stringify(item.info.incompatible_qos)}`}
                  placement="right"
                  disableInteractive
                >
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                </Tooltip>
              )}
              <Button
                size="small"
                style={{
                  marginLeft: "0.3em",
                  textTransform: "none",
                  justifyContent: "left",
                  padding: 0,
                  color: "#6c50e9ff",
                }}
                onClick={() => {
                  // ${item.providerId}
                  const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                  navCtx.setSelectedNodes([id], true);
                  // inform details panel tab about selected nodes by user
                  emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.DETAILS, "default"));
                }}
              >
                {subNodeName}
              </Button>
              <CopyButton value={subNodeName} fontSize="0.7em" />
            </Stack>
          );
        })}
        {topicInfo?.hasQos && (
          <Typography fontWeight="500" fontStyle="italic" fontSize="small">
            Qos profiles:
          </Typography>
        )}
        {topicInfo?.hasQos && createReliabilityItem(topicInfo)}
        {topicInfo?.hasQos && createDurabilityItem(topicInfo)}
        {topicInfo?.hasQos && createLivelinessItem(topicInfo)}
      </Stack>
    );
  }, [topicInfo]);

  return (
    <Stack direction="column" alignItems="left" spacing={0}>
      {createInfo}
      {showInfo && createExtendedInfo}
    </Stack>
  );
}
