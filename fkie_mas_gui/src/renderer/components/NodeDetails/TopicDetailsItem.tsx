import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import RosContext from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { RosNode, RosTopic, RosTopicId, TopicExtendedInfo } from "@/renderer/models";
import { durabilityToString, livelinessToString, reliabilityToString } from "@/renderer/models/RosQos";
import { EndpointExtendedInfo } from "@/renderer/models/TopicExtendedInfo";
import { LAYOUT_TAB_SETS, LAYOUT_TABS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import TopicPublishPanel from "@/renderer/pages/NodeManager/panels/TopicPublishPanel";
import { EVENT_PROVIDER_ROS_TOPICS } from "@/renderer/providers/eventTypes";
import { removeDDSuid } from "@/renderer/utils";
import ChatBubbleOutlineIcon from "@mui/icons-material/ChatBubbleOutline";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import { Button, Chip, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { forwardRef, ReactNode, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

type TopicDetailsItemsProps = {
  providerId: string | undefined;
  topicId: RosTopicId;
  showConnections: boolean;
};

const TopicDetailsItem = forwardRef<HTMLDivElement, TopicDetailsItemsProps>(function TopicDetailsItem(props, ref) {
  const { providerId, topicId, showConnections = true } = props;

  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [allTopics, setAllTopics] = useState<TopicExtendedInfo[]>([]);
  const [showInfo, setShowInfo] = useState<boolean>(false);
  const [hasIncompatibleQos, setHasIncompatibleQos] = useState<boolean>(false);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx, settingsCtx.changed]);

  function onEchoClick(topic: TopicExtendedInfo, external: boolean = false, openInTerminal: boolean = false): void {
    navCtx.openSubscriber(providerId || "", topic.name, true, false, external, openInTerminal);
  }

  function onPublishClick(topic: TopicExtendedInfo): void {
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `publish-${topic.name}-${providerId}`,
        topic.name || "undefined",
        <TopicPublishPanel topicName={topic.name} topicType={topic.msgType} providerId={providerId} />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(true, "publish")
      )
    );
  }

  function updateTopicList(): void {
    if (providerId) {
      const provider = rosCtx.getProviderById(providerId);
      if (provider) {
        const rosTopics: RosTopic[] = provider?.rosTopics.filter(
          (item) => item.name === topicId.name && item.msg_type === topicId.msg_type
        );
        // setAllTopics(rosTopics);
        setAllTopics(
          rosTopics.map((rt: RosTopic) => {
            let topicInfo: TopicExtendedInfo | undefined = undefined;
            function addNode(rosTopic: RosTopic, rosNode: RosNode): void {
              if (topicInfo) {
                topicInfo.add(rosTopic, rosNode);
              } else {
                topicInfo = new TopicExtendedInfo(rt, rosNode);
              }
            }
            // Get topics from the ros node list of each provider.
            for (const provider of rosCtx.providers) {
              for (const pub of rt.publisher || []) {
                const rosNode = provider.rosNodes.find((node: RosNode) => node.id === pub.node_id);
                if (rosNode) {
                  addNode(rt, rosNode);
                }
              }
              for (const sub of rt.subscriber || []) {
                const rosNode = provider.rosNodes.find((node: RosNode) => node.id === sub.node_id);
                if (rosNode) {
                  addNode(rt, rosNode);
                }
              }
            }
            return (
              topicInfo || new TopicExtendedInfo(rt, { providerId: providerId, providerName: providerId } as RosNode)
            );
          })
        );
        // check for incompatible qos
        let foundIncompatibleQos = false;
        for (const rt of rosTopics) {
          for (const pub of rt.publisher || []) {
            if (pub.incompatible_qos) {
              foundIncompatibleQos = true;
              break;
            }
          }
          if (foundIncompatibleQos) break;
          for (const pub of rt.publisher || []) {
            if (pub.incompatible_qos) {
              foundIncompatibleQos = true;
              break;
            }
          }
          if (foundIncompatibleQos) break;
        }
        setHasIncompatibleQos(foundIncompatibleQos);
      }
    }
  }

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
    return (
      <Stack direction="row" alignItems="center" spacing={0}>
        {allTopics.map((rt: TopicExtendedInfo, index: number) => {
          return (
            <Stack
              key={`pub-sub-${rt.name}-${index}`}
              alignItems="center"
              direction="row"
              margin={0}
              spacing={"0.1em"}
              style={{ display: "flex", flexGrow: 1, borderBottom: `1px solid ${alpha(grey[600], 0.4)}` }}
            >
              <Chip
                size="small"
                onClick={() => {
                  onPublishClick(rt);
                }}
                avatar={
                  <Tooltip title="Create a publisher" placement="left" enterDelay={tooltipDelay} disableInteractive>
                    <PlayArrowRoundedIcon style={{ padding: 1, color: "#09770fff" }} fontSize="inherit" />
                  </Tooltip>
                }
                label={
                  showConnections && (
                    <Typography fontSize="inherit" color={(rt.publishers || []).length === 0 ? "warning" : "default"}>
                      {rt.publishers ? rt.publishers.length : 0}
                    </Typography>
                  )
                }
              />
              <Chip
                size="small"
                onClick={(event) => {
                  onEchoClick(rt, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey && event.nativeEvent.shiftKey);
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
                    <Typography fontSize="inherit" color={(rt.subscribers || []).length > 0 ? "default" : "warning"}>
                      {rt.subscribers ? rt.subscribers.length : 0}
                    </Typography>
                  )
                }
              />
              <Button
                size="small"
                style={{
                  marginLeft: 1,
                  textTransform: "none",
                  justifyContent: "left",
                }}
                onClick={() => setShowInfo((prev) => !prev)}
                onDoubleClick={() => {
                  navigator.clipboard.writeText(topicId.name);
                  logCtx.success(`${topicId.name} copied`);
                }}
              >
                {`${topicId.name}`}
              </Button>
              {hasIncompatibleQos && (
                <Tooltip title={"There are subscribers with incompatible QoS"} placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
                </Tooltip>
              )}
            </Stack>
          );
        })}
      </Stack>
    );
  }, [allTopics, showConnections]);

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const createExtendedInfo = useMemo(() => {
    return allTopics.map((rt: TopicExtendedInfo, index: number) => {
      return (
        <Stack
          key={`info-${rt.name}-${index}`}
          style={{ marginLeft: 15, paddingLeft: 5, borderLeft: `1px dashed ${alpha(grey[600], 0.4)}` }}
        >
          <Stack direction="row" alignItems="center" spacing="1em">
            <Typography fontWeight="500" fontStyle="italic" fontSize="small">
              Type:
            </Typography>
            <Button
              size="small"
              style={{
                marginLeft: 1,
                textTransform: "none",
                justifyContent: "left",
                padding: 0,
                color: "inherit",
              }}
              onClick={() => {
                navigator.clipboard.writeText(rt.msgType);
                logCtx.success(`${rt.msgType} copied`);
              }}
            >
              {rt.msgType}
            </Button>
          </Stack>
          <Typography fontWeight="500" fontStyle="italic" fontSize="small">
            Publisher [{rt.publishers?.length || 0}]:
          </Typography>
          {rt.publishers?.map((item: EndpointExtendedInfo) => {
            const pubNodeName = removeDDSuid(item.info.node_id);
            return (
              <Stack key={item.info.node_id} paddingLeft={"1em"} direction="row">
                <Button
                  size="small"
                  style={{
                    marginLeft: 1,
                    textTransform: "none",
                    justifyContent: "left",
                    padding: 0,
                    color: "#09770fff",
                  }}
                  onClick={() => {
                    const id: string = `${item.providerId}${item.info.node_id.replaceAll("/", "#")}`;
                    navCtx.setSelectedNodes([id], true);
                    // inform details panel tab about selected nodes by user
                    emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                  }}
                >
                  {pubNodeName}
                </Button>
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
          <Typography fontWeight="500" fontStyle="italic" fontSize="small">
            Subscriber [{rt.subscribers.length || 0}]:
          </Typography>
          {rt.subscribers.map((item: EndpointExtendedInfo) => {
            const pubNodeName = removeDDSuid(item.info.node_id);
            return (
              <Stack key={item.info.node_id} paddingLeft={"1em"} direction="row">
                <Button
                  size="small"
                  style={{
                    marginLeft: 1,
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
                    emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                  }}
                >
                  {pubNodeName}
                </Button>
                {item.info.incompatible_qos && item.info.incompatible_qos.length > 0 && (
                  <Tooltip
                    title={`Incompatible QoS: ${JSON.stringify(item.info.incompatible_qos)}`}
                    placement="right"
                    disableInteractive
                  >
                    <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                  </Tooltip>
                )}
                {/* <CopyButton value={pubNodeName} fontSize="0.7em" /> */}
              </Stack>
            );
          })}
          {rt?.hasQos && (
            <Typography fontWeight="500" fontStyle="italic" fontSize="small">
              Qos profiles:
            </Typography>
          )}
          {rt?.hasQos && createReliabilityItem(rt)}
          {rt?.hasQos && createDurabilityItem(rt)}
          {rt?.hasQos && createLivelinessItem(rt)}
        </Stack>
      );
    });
  }, [allTopics]);

  return (
    <Stack direction="column" alignItems="left" spacing={0} ref={ref}>
      {createInfo}
      {showInfo && createExtendedInfo}
    </Stack>
  );
});

export default TopicDetailsItem;
