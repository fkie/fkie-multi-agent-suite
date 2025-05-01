import {
  Alert,
  AlertTitle,
  Box,
  Button,
  Chip,
  Paper,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  Tooltip,
  Typography,
} from "@mui/material";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import JsonView from "react18-json-view";

import { colorFromHostname, getDiagnosticStyle } from "@/renderer/components/UI/Colors";
import CopyButton from "@/renderer/components/UI/CopyButton";
import Tag from "@/renderer/components/UI/Tag";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { NavigationContext } from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { RosNode, RosNodeStatus, RosTopic, RosTopicId, getDiagnosticLevelName, getFileName } from "@/renderer/models";
import { EVENT_PROVIDER_ROS_SERVICES, EVENT_PROVIDER_ROS_TOPICS } from "@/renderer/providers/eventTypes";
import { generateUniqueId } from "@/renderer/utils";
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import OverflowMenuService, { EMenuService } from "./OverflowMenuService";
import OverflowMenuTopic, { EMenuTopic } from "./OverflowMenuTopic";
import ServiceCallerPanel from "./ServiceCallerPanel";
import ServicesPanel from "./ServicesPanel";
import TopicPublishPanel from "./TopicPublishPanel";
import TopicsPanel from "./TopicsPanel";

function compareTopics(a: RosTopicId | RosTopic, b: RosTopicId | RosTopic): number {
  if (a.name < b.name) {
    return -1;
  }
  if (a.name > b.name) {
    return 1;
  }
  return 0;
}

export default function NodesDetailsPanel(): JSX.Element {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const navCtx = useContext(NavigationContext);

  const [nodesShow, setNodesShow] = useState<RosNode[]>([]);
  const [logPaths, setLogPaths] = useState({}); // {node.idGlobal: LogPathItem}

  const [showNodeInfo] = useLocalStorage("NodesDetailsPanel:showNodeInfo", true);
  const [showPublishers] = useLocalStorage("NodesDetailsPanel:showPublishers", true);
  const [showSubscribers] = useLocalStorage("NodesDetailsPanel:showSubscribers", true);
  const [showServices] = useLocalStorage("NodesDetailsPanel:showServices", true);
  const [showConnections] = useLocalStorage("NodesDetailsPanel:showConnections", true);
  const [showPaths] = useLocalStorage("NodesDetailsPanel:showPaths", true);
  const [showLaunchParameter] = useLocalStorage("NodesDetailsPanel:showLaunchParameter", true);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // TODO: Make a parameter or config for [maxNodes]
    const maxNodes = 1;
    const idsToShow = navCtx.selectedNodes.slice(
      0,
      navCtx.selectedNodes.length > maxNodes ? maxNodes : navCtx.selectedNodes.length
    );
    const nodes: RosNode[] = [];
    for (const id of idsToShow) {
      const n = rosCtx.nodeMap.get(id);
      if (n) {
        nodes.push(n);
      }
    }
    setNodesShow(nodes);
  }, [navCtx.selectedNodes, rosCtx.nodeMap]);

  useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, () => {
    setNodesShow((prev) => [...prev]);
  });

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, () => {
    setNodesShow((prev) => [...prev]);
  });

  async function onTopicClick(
    rosTopicType: EMenuTopic,
    topic: string,
    messageType: string,
    providerId: string,
    external: boolean = false,
    openInTerminal: boolean = false
  ): Promise<void> {
    if (rosTopicType === EMenuTopic.clipboard) {
      if (navigator?.clipboard) {
        navigator.clipboard.writeText(topic);
        logCtx.success(`${topic} copied!`);
      }
      return;
    }
    if (rosTopicType === EMenuTopic.INFO) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `topics-${generateUniqueId()}`,
          `${topic}`,
          <TopicsPanel initialSearchTerm={topic} />,
          true,
          LAYOUT_TABS.NODES,
          new LayoutTabConfig(false, "info")
        )
      );
      return;
    }

    if (rosTopicType === EMenuTopic.PUBLISH) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `publish-${topic}-${providerId}`,
          topic,
          <TopicPublishPanel topicName={topic} topicType={messageType} providerId={providerId} />,
          true,
          LAYOUT_TAB_SETS[settingsCtx.get("publisherOpenLocation") as string],
          new LayoutTabConfig(false, "publish")
        )
      );
      return;
    }

    let defaultNoData = true;
    if (rosTopicType === EMenuTopic.ECHO) {
      defaultNoData = false;
    }
    navCtx.openSubscriber(providerId, topic, true, defaultNoData, external, openInTerminal);
  }

  function onServiceClick(rosServiceType: EMenuService, service: string, msgType: string, providerId: string): void {
    if (rosServiceType === EMenuService.clipboard) {
      if (navigator?.clipboard) {
        navigator.clipboard.writeText(service);
        logCtx.success(`${service} copied!`);
      }
      return;
    }
    if (rosServiceType === EMenuService.SERVICE_CALL) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `call-service-${generateUniqueId()}`,
          service,
          <ServiceCallerPanel serviceName={service} serviceType={msgType} providerId={providerId} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, LAYOUT_TABS.SERVICES)
        )
      );
      return;
    }
    if (rosServiceType === EMenuService.INFO) {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `service-${generateUniqueId()}`,
          `${service}`,
          <ServicesPanel initialSearchTerm={service} />,
          true,
          LAYOUT_TABS.NODES,
          new LayoutTabConfig(false, "info")
        )
      );
    }
  }

  const getHostStyle = useCallback(
    (providerName: string | undefined) => {
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          borderTopStyle: "solid",
          borderTopColor: colorFromHostname(providerName),
          borderTopWidth: "0.4em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  const createProviderDetailsView = useMemo(() => {
    const providerId = navCtx.selectedProviders?.[0];
    const provider = rosCtx.getProviderById(providerId);
    if (!provider) return;
    const rmwImplementation = provider.systemEnv.RMW_IMPLEMENTATION as string;
    return (
      <Stack
        key={providerId}
        // spacing={1}
        alignItems="left"
        marginBottom={1.5}
      >
        <Stack paddingTop={0} marginBottom={0.5} sx={getHostStyle(provider.name())}>
          <Typography
            variant="subtitle1"
            style={{
              // cursor: "pointer",
              color: "#fff",
              backgroundColor: "#16a085",
            }}
            align="center"
          >
            <Stack spacing={0} sx={{ fontWeight: "bold", m: 0, paddingTop: "0.2em" }}>
              <Typography variant="subtitle2" align="center">
                {provider.connection.uri}
              </Typography>
              <Box>{provider.name()}</Box>
            </Stack>
          </Typography>
        </Stack>
        <Stack spacing={0.5}>
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={provider.daemon ? "default" : "error"}
              title="daemon:"
              // title={`${RosNodeStatusInfo[node.status]}`}
              text={provider.daemon ? "running" : "not running"}
              wrap
            />
          </Stack>
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={provider.discovery ? "default" : "error"}
              title="discovery:"
              // title={`${RosNodeStatusInfo[node.status]}`}
              text={provider.discovery ? "running" : "not running"}
              wrap
            />
          </Stack>
          {rmwImplementation && (
            <Stack direction="row" spacing={0.5}>
              <Tag
                color={"default"}
                title="RMW_IMPLEMENTATION:"
                // title={`${RosNodeStatusInfo[node.status]}`}
                text={rmwImplementation}
                wrap
              />
            </Stack>
          )}
          {provider.hostnames && (
            <Stack direction="row" spacing={0.5}>
              <Tag
                color={"default"}
                title="host:"
                // title={`${RosNodeStatusInfo[node.status]}`}
                text={JSON.stringify(provider.hostnames)}
                wrap
              />
            </Stack>
          )}
        </Stack>
      </Stack>
    );
  }, [navCtx.selectedProviders]);

  const createNodeDetailsView = useMemo(() => {
    const result = nodesShow.map((node: RosNode) => {
      const provider = rosCtx.getProviderById(node.providerId);
      return (
        <Stack
          key={node.idGlobal}
          // spacing={1}
          alignItems="left"
        >
          <Stack paddingTop={0} marginBottom={0.5} sx={getHostStyle(node.providerName)}>
            <Typography
              variant="subtitle1"
              style={{
                // cursor: "pointer",
                color: "#fff",
                backgroundColor: "#2196f3",
              }}
              align="center"
            >
              <Stack spacing={0} sx={{ fontWeight: "bold", m: 0, paddingTop: "0.2em" }}>
                {node.namespace !== "/" && (
                  <Tooltip title="namespace" placement="bottom" disableInteractive>
                    <Typography variant="subtitle2" align="center">
                      {node?.namespace}
                    </Typography>
                  </Tooltip>
                )}
                <Box>
                  {node.namespace !== "/" ? node.name.replace(node.namespace, "").replace("/", "") : node.name}
                  <CopyButton value={node.name} fontSize={"inherit"} />
                </Box>
              </Stack>
            </Typography>
          </Stack>

          {(node.diagnosticLevel || 0) > 0 && (
            <Typography variant="body1" style={getDiagnosticStyle(node.diagnosticLevel || 0)} marginBottom={1}>
              {getDiagnosticLevelName(node.diagnosticLevel || 0)}: {node.diagnosticMessage}
            </Typography>
          )}

          {showNodeInfo && (
            <Stack spacing={0.5}>
              <Stack direction="row" spacing={0.5}>
                <Tag
                  color={node.status === RosNodeStatus.RUNNING ? "success" : "default"}
                  title=""
                  // title={`${RosNodeStatusInfo[node.status]}`}
                  text={node.status}
                  wrap
                />
              </Stack>
              {node.pid && Math.round(node.pid) > 0 && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="PID:" text={`${node.pid}`} wrap />
                </Stack>
              )}
              {node.lifecycle_state && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="Lifecycle:" text={`${node.lifecycle_state}`} wrap />
                </Stack>
              )}
              {(node.lifecycle_available_transitions || []).length > 0 && (
                <Stack direction="row" spacing={0.5}>
                  <Tag
                    color="default"
                    title="Lifecycle transitions:"
                    text={`${JSON.stringify(node.lifecycle_available_transitions)}`}
                    wrap
                  />
                </Stack>
              )}
              {node.id && node.id !== node.name && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="ID:" text={`${node.id}`} wrap />
                </Stack>
              )}
              {node.node_API_URI && node.node_API_URI.length > 0 && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="URI:" text={node.node_API_URI} wrap />
                </Stack>
              )}
              {node.masteruri && node.masteruri.length > 0 && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="MASTERURI:" text={node.masteruri} wrap />
                </Stack>
              )}
              {node.location && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="Location:" text={`${node.location} - ${node.providerName}`} wrap />
                </Stack>
              )}
              {node.container_name ? (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="Composable Container:" text={`${node.container_name}`} wrap />
                </Stack>
              ) : (
                node.getAllContainers().length > 0 && (
                  <Stack direction="row" spacing={0.5}>
                    <Tag
                      color="default"
                      title="Composable Container:"
                      text={`${JSON.stringify(node.getAllContainers())}`}
                      wrap
                    />
                  </Stack>
                )
              )}
              {node.guid && (
                <Stack direction="row" spacing={0.5}>
                  <Tag color="default" title="GID:" text={`${node.guid}`} wrap />
                </Stack>
              )}
              {node.screens && node.screens.length > 0 && (
                <Stack direction="row" spacing={0.5}>
                  {node.screens.map((screen) => (
                    <Tag
                      key={`screen-${screen}-`}
                      color="default"
                      title="Screen:"
                      text={screen}
                      wrap
                      copyButton={screen}
                    />
                  ))}
                </Stack>
              )}
              {node.launchInfo.size > 0 && (
                <Stack direction="column" spacing={0.5}>
                  {Array.from(node.launchInfo.keys()).map((launchPath) => (
                    <Tag
                      key={launchPath}
                      color="info"
                      title={`${launchPath === node.launchPath ? "*" : ""}Launch:`}
                      text={getFileName(launchPath)}
                      wrap
                      copyButton={launchPath}
                    />
                  ))}
                </Stack>
              )}
            </Stack>
          )}

          {node && (
            <Stack spacing={1}>
              {showSubscribers && node.subscribers && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Subscribed Topics:
                      {` [${node.subscribers.length}]`}
                    </Box>
                  </Typography>
                  {node.subscribers.length > 0 && (
                    <TableContainer component={Paper}>
                      <Table size="small" aria-label="a dense table">
                        <TableBody>
                          {Array.from(node.subscribers.values())
                            .sort(compareTopics)
                            .map((topic) => (
                              <TableRow key={topic.name}>
                                <TableCell style={{ padding: 0 }}>
                                  <Stack direction="row" alignItems="center" spacing={0}>
                                    <OverflowMenuTopic
                                      onClick={onTopicClick}
                                      topicName={topic.name}
                                      messageType={topic.msg_type}
                                      providerId={node.providerId}
                                    />
                                    {showConnections &&
                                      provider?.rosTopics
                                        .filter((item) => item.name === topic.name && item.msg_type === topic.msg_type)
                                        .map((rt: RosTopic) => {
                                          return (
                                            <Stack key={`pub-sub-${rt.name}`} direction="row">
                                              <Chip
                                                size="small"
                                                title="publishers"
                                                // showZero={true}
                                                color={(rt.publisher || []).length === 0 ? "warning" : "default"}
                                                label={rt.publisher ? rt.publisher.length : 0}
                                              />
                                              <Chip
                                                size="small"
                                                title="subscribers"
                                                // showZero={true}
                                                color={(rt.subscriber || []).length > 0 ? "default" : "warning"}
                                                label={rt.subscriber ? rt.subscriber.length : 0}
                                              />
                                            </Stack>
                                          );
                                        })}
                                    <Button
                                      size="small"
                                      style={{
                                        marginLeft: 1,
                                        textTransform: "none",
                                        justifyContent: "left",
                                      }}
                                      onClick={(event) =>
                                        onTopicClick(
                                          event.nativeEvent.ctrlKey && !event.nativeEvent.shiftKey
                                            ? EMenuTopic.PUBLISH
                                            : EMenuTopic.ECHO,
                                          topic.name,
                                          topic.msg_type,
                                          node.providerId,
                                          event.nativeEvent.shiftKey,
                                          event.nativeEvent.ctrlKey && event.nativeEvent.shiftKey
                                        )
                                      }
                                    >
                                      {`${topic.name}`}
                                    </Button>
                                  </Stack>
                                </TableCell>
                              </TableRow>
                            ))}
                        </TableBody>
                      </Table>
                    </TableContainer>
                  )}
                </>
              )}

              {showPublishers && node.publishers && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Published topics:
                      {` [${node.publishers.length}]`}
                    </Box>
                  </Typography>
                  {node.publishers.length > 0 && (
                    // useZebraStyles={false}>
                    <TableContainer component={Paper}>
                      <Table size="small" aria-label="a dense table">
                        <TableBody>
                          {node.publishers.sort(compareTopics).map((topic) => (
                            <TableRow key={topic.name}>
                              <TableCell style={{ padding: 0 }}>
                                <Stack direction="row" alignItems="center" spacing={0}>
                                  <OverflowMenuTopic
                                    onClick={onTopicClick}
                                    topicName={topic.name}
                                    messageType={topic.msg_type}
                                    providerId={node.providerId}
                                  />
                                  {showConnections &&
                                    provider?.rosTopics
                                      .filter((item) => item.name === topic.name && item.msg_type === topic.msg_type)
                                      .map((rt: RosTopic) => {
                                        return (
                                          <Stack key={`pub-sub-${rt.name}`} direction="row">
                                            <Chip
                                              size="small"
                                              title="publishers"
                                              // showZero={true}
                                              color="default"
                                              label={rt.publisher ? rt.publisher.length : 0}
                                            />
                                            <Chip
                                              size="small"
                                              title="subscribers"
                                              // showZero={true}
                                              color={(rt.subscriber || []).length > 0 ? "default" : "warning"}
                                              label={rt.subscriber ? rt.subscriber.length : 0}
                                            />
                                          </Stack>
                                        );
                                      })}
                                  <Button
                                    size="small"
                                    style={{
                                      marginLeft: 1,
                                      textTransform: "none",
                                      justifyContent: "left",
                                    }}
                                    onClick={(event) => {
                                      onTopicClick(
                                        event.nativeEvent.ctrlKey && !event.nativeEvent.shiftKey
                                          ? EMenuTopic.PUBLISH
                                          : EMenuTopic.ECHO,
                                        topic.name,
                                        topic.msg_type,
                                        node.providerId,
                                        event.nativeEvent.shiftKey,
                                        event.nativeEvent.ctrlKey && event.nativeEvent.shiftKey
                                      );
                                    }}
                                  >
                                    {`${topic.name}`}
                                  </Button>
                                </Stack>
                              </TableCell>
                            </TableRow>
                          ))}
                        </TableBody>
                      </Table>
                    </TableContainer>
                  )}
                </>
              )}

              {showServices && node.services && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Available Services:
                      {` [${node.services.length}]`}
                    </Box>
                  </Typography>

                  {node.services.length > 0 && (
                    <TableContainer component={Paper}>
                      <Table size="small" aria-label="a dense table">
                        <TableBody>
                          {node.services.map((service) => (
                            <TableRow key={`${service.name}#${service.msg_type}`}>
                              <TableCell style={{ padding: 0 }}>
                                <Stack direction="row" spacing={1}>
                                  <OverflowMenuService
                                    onClick={onServiceClick}
                                    serviceName={service.name}
                                    serviceType={service.msg_type}
                                    providerId={node.providerId}
                                  />

                                  <Button
                                    size="small"
                                    style={{
                                      padding: 0,
                                      textTransform: "none",
                                      justifyContent: "left",
                                    }}
                                    onClick={() =>
                                      onServiceClick(
                                        EMenuService.SERVICE_CALL,
                                        service.name,
                                        service.msg_type,
                                        node.providerId
                                      )
                                    }
                                  >
                                    {`${service.name}`}
                                  </Button>
                                </Stack>
                              </TableCell>
                            </TableRow>
                          ))}
                        </TableBody>
                      </Table>
                    </TableContainer>
                  )}
                </>
              )}

              {showPaths && (
                <Stack direction="column" spacing={0.5}>
                  {Array.from(node.launchInfo.keys()).map((launchPath) => (
                    <Stack key={`launch-${launchPath}`} direction="row" spacing={0.5}>
                      <Tag
                        key={launchPath}
                        color="default"
                        title="Launch path:"
                        text={launchPath}
                        wrap
                        copyButton={launchPath}
                      />
                    </Stack>
                  ))}
                  {logPaths[node.idGlobal]?.map((logItem) => {
                    return (
                      <Stack key={`log-${logItem.id}`}>
                        <Stack direction="row" spacing={0.5}>
                          <Tag
                            key={logItem.screen_log}
                            color="default"
                            title="Screen log:"
                            text={logItem.screen_log}
                            wrap
                            copyButton={logItem.screen_log}
                          />
                        </Stack>
                        <Stack direction="row" spacing={0.5}>
                          <Tag
                            key={logItem.ros_log}
                            color="default"
                            title="Ros log:"
                            text={logItem.ros_log}
                            wrap
                            copyButton={logItem.ros_log}
                          />
                        </Stack>
                      </Stack>
                    );
                  })}
                  {!logPaths[node.idGlobal] && (
                    <Stack direction="row" spacing={0.5}>
                      <Button
                        type="submit"
                        variant="contained"
                        size="small"
                        color="warning"
                        onClick={async () => {
                          const logs = await rosCtx.getProviderById(node.providerId)?.getLogPaths([node.name]);
                          logPaths[node.idGlobal] = logs;
                          setLogPaths({ ...logPaths });
                        }}
                        style={{ textAlign: "center" }}
                      >
                        get log paths
                      </Button>
                    </Stack>
                  )}
                </Stack>
              )}
              {showLaunchParameter && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>Launch Parameter:</Box>
                  </Typography>
                  {Array.from(node.launchInfo.keys()).map((launchPath: string) => {
                    const launchInfo = node.launchInfo.get(launchPath);
                    if (launchInfo) {
                      return (
                        <Stack key={launchPath} marginTop={"0.5em"}>
                          <Typography variant="caption">
                            <Box sx={{ fontWeight: "bold" }}>
                              {`${launchPath.split("/").slice(-1)} [${launchInfo.parameters?.length}]`}
                            </Box>
                          </Typography>
                          {launchInfo.cmd && (
                            <Stack direction="column" paddingBottom={0.5}>
                              {launchInfo.timer_period && launchInfo.timer_period > 0 && (
                                <Typography variant="caption">
                                  <Stack direction="row" spacing={0.5}>
                                    <Box sx={{ fontWeight: "bold", color: "orange" }}>Delayed start:</Box>
                                    <Box sx={{ fontWeight: "normal" }}>{launchInfo.timer_period} sec</Box>
                                  </Stack>
                                </Typography>
                              )}
                              <Tag
                                color="default"
                                title="CMD:"
                                text={`${launchInfo.cmd}`}
                                wrap
                                copyButton={`${launchInfo.cmd}`}
                              />
                            </Stack>
                          )}
                          {launchInfo.parameters && launchInfo.parameters.length > 0 && (
                            <TableContainer component={Paper}>
                              <Table size="small" aria-label="a dense table">
                                <TableBody>
                                  {launchInfo.parameters.map((parameter) => (
                                    <TableRow key={parameter.name}>
                                      <TableCell style={{ padding: 0 }}>
                                        {parameter.name.startsWith(node.name)
                                          ? parameter.name.slice(node.name.length + 1)
                                          : parameter.name}
                                        {typeof parameter.value === "object" && (
                                          <JsonView
                                            src={parameter.value}
                                            dark={settingsCtx.get("useDarkMode") as boolean}
                                            theme="a11y"
                                            enableClipboard={false}
                                            ignoreLargeArray={false}
                                            collapseObjectsAfterLength={3}
                                            displaySize={"collapsed"}
                                            collapsed={(params: {
                                              node: Record<string, unknown> | Array<unknown>; // Object or array
                                              indexOrName: number | string | undefined;
                                              depth: number;
                                              size: number; // Object's size or array's length
                                            }) => {
                                              if (params.indexOrName === undefined) return true;
                                              if (Array.isArray(params.node) && params.node.length === 0) return true;
                                              if (params.depth > 3) return true;
                                              return false;
                                            }}
                                          />
                                        )}
                                      </TableCell>
                                      {typeof parameter.value !== "object" ? (
                                        <TableCell style={{ padding: 0 }}>{JSON.stringify(parameter.value)}</TableCell>
                                      ) : (
                                        <TableCell style={{ padding: 0 }} />
                                      )}
                                    </TableRow>
                                  ))}
                                </TableBody>
                              </Table>
                            </TableContainer>
                          )}
                        </Stack>
                      );
                    }
                    // biome-ignore lint/correctness/useJsxKeyInIterable: <explanation>
                    return <></>;
                  })}
                </>
              )}
            </Stack>
          )}
        </Stack>
      );
    });
    return result;
  }, [
    nodesShow,
    showNodeInfo,
    showPublishers,
    showConnections,
    showSubscribers,
    showServices,
    showPaths,
    showLaunchParameter,
    // onTopicClick,    <= causes unnecessary rebuilds
    // onServiceClick,  <= causes unnecessary rebuilds
    rosCtx,
    logPaths,
  ]);

  return (
    <Box width="100%" height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
      {navCtx.selectedProviders.length === 1 && createProviderDetailsView}
      {navCtx.selectedNodes.length > 1 && (
        <Stack direction="row" justifyContent="center">
          <Typography color="grey" variant="body2">
            selected: {navCtx.selectedNodes.length}, displayed: {nodesShow?.length}
          </Typography>
        </Stack>
      )}
      {createNodeDetailsView}

      {nodesShow?.length === 0 && (
        <Alert severity="info" style={{ minWidth: 0 }}>
          <AlertTitle>Please select a node</AlertTitle>
        </Alert>
      )}
    </Box>
  );
}
