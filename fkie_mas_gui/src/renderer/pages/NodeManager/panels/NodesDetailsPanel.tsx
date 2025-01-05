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
import { emitCustomEvent } from "react-custom-events";
import { JSONTree } from "react-json-tree";
import { CopyButton, Tag, colorFromHostname, getDiagnosticStyle } from "../../../components";
import { LoggingContext } from "../../../context/LoggingContext";
import { NavigationContext } from "../../../context/NavigationContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { RosNode, RosNodeStatus, getDiagnosticLevelName, getFileName } from "../../../models";
import { darkThemeJson } from "../../../themes/darkTheme";
import { lightThemeJson } from "../../../themes/lightTheme";
import { generateUniqueId } from "../../../utils";
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import OverflowMenuService, { EMenuService } from "./OverflowMenuService";
import OverflowMenuTopic, { EMenuTopic } from "./OverflowMenuTopic";
import ServiceCallerPanel from "./ServiceCallerPanel";
import ServicesPanel from "./ServicesPanel";
import TopicPublishPanel from "./TopicPublishPanel";
import TopicsPanel from "./TopicsPanel";

const compareTopics = (a, b) => {
  if (a.name < b.name) {
    return -1;
  }
  if (a.name > b.name) {
    return 1;
  }
  return 0;
};

function NodesDetailsPanel() {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const navCtx = useContext(NavigationContext);

  const [nodesShow, setNodesShow] = useState<RosNode[]>([]);
  const [logPaths, setLogPaths] = useState({}); // {node.idGlobal: LogPathItem}
  const [currentTheme, setCurrentTheme] = useState(settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson);

  const [showNodeInfo] = useLocalStorage("NodesDetailsPanel:showNodeInfo", false);
  const [showPublishers] = useLocalStorage("NodesDetailsPanel:showPublishers", true);
  const [showSubscribers] = useLocalStorage("NodesDetailsPanel:showSubscribers", true);
  const [showServices] = useLocalStorage("NodesDetailsPanel:showServices", false);
  const [showConnections] = useLocalStorage("NodesDetailsPanel:showConnections", true);
  const [showPaths] = useLocalStorage("NodesDetailsPanel:showPaths", true);
  const [showLaunchParameter] = useLocalStorage("NodesDetailsPanel:showLaunchParameter", true);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setCurrentTheme(settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // TODO: Make a parameter or config for [maxNodes]
    const maxNodes = 1;
    const idsToShow = navCtx.selectedNodes.slice(
      0,
      navCtx.selectedNodes.length > maxNodes ? maxNodes : navCtx.selectedNodes.length
    );
    const nodes: RosNode[] = [];
    idsToShow.forEach((id) => {
      const n = rosCtx.nodeMap.get(id);
      if (n) {
        nodes.push(n);
      }
    });
    setNodesShow(nodes);
  }, [navCtx.selectedNodes, rosCtx.nodeMap]);

  const onTopicClick = useCallback(
    async (
      rosTopicType: EMenuTopic,
      topic: string,
      providerId: string,
      external: boolean = false,
      openInTerminal: boolean = false
    ) => {
      if (rosTopicType === EMenuTopic.clipboard) {
        if (navigator && navigator.clipboard) {
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
            <TopicPublishPanel topicName={topic} providerId={providerId} />,
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
      rosCtx.openSubscriber(providerId, topic, true, defaultNoData, external, openInTerminal);
    },
    [logCtx, rosCtx]
  );

  const onServiceClick = useCallback(
    (rosServiceType: EMenuService, service: string, providerId: string) => {
      if (rosServiceType === EMenuService.clipboard) {
        if (navigator && navigator.clipboard) {
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
            <ServiceCallerPanel serviceName={service} providerId={providerId} />,
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
    },
    [logCtx]
  );

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
    [settingsCtx]
  );

  const createNodeDetailsView = useMemo(() => {
    const result = nodesShow.map((node: RosNode) => {
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

          {node && node.diagnosticLevel > 0 && (
            <Typography variant="body1" style={getDiagnosticStyle(node.diagnosticLevel)} marginBottom={1}>
              {getDiagnosticLevelName(node.diagnosticLevel)}: {node.diagnosticMessage}
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
              {node.lifecycle_available_transitions && (
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
              {node.screens.length > 0 && (
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
              {showSubscribers && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Subscribed Topics:
                      {` [${node.subscribers.size}]`}
                    </Box>
                  </Typography>
                  {node.subscribers.size > 0 && (
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
                                      providerId={node.providerId}
                                    />
                                    {showConnections && (
                                      <Stack direction="row">
                                        <Chip
                                          size="small"
                                          title="publishers"
                                          // showZero={true}
                                          color={(() => {
                                            if (topic.publisher.length === 0) return "warning";
                                            return "default";
                                          })()}
                                          label={topic.publisher.length}
                                        />
                                        <Chip
                                          size="small"
                                          title="subscribers"
                                          // showZero={true}
                                          color={topic.subscriber.length > 0 ? "default" : "warning"}
                                          label={topic.subscriber.length}
                                        />
                                      </Stack>
                                    )}
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

              {showPublishers && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Published topics:
                      {` [${node.publishers.size}]`}
                    </Box>
                  </Typography>
                  {node.publishers.size > 0 && (
                    // useZebraStyles={false}>
                    <TableContainer component={Paper}>
                      <Table size="small" aria-label="a dense table">
                        <TableBody>
                          {Array.from(node.publishers.values())
                            .sort(compareTopics)
                            .map((topic) => (
                              <TableRow key={topic.name}>
                                <TableCell style={{ padding: 0 }}>
                                  <Stack direction="row" alignItems="center" spacing={0}>
                                    <OverflowMenuTopic
                                      onClick={onTopicClick}
                                      topicName={topic.name}
                                      providerId={node.providerId}
                                    />

                                    {showConnections && (
                                      <Stack direction="row">
                                        <Chip
                                          size="small"
                                          title="publishers"
                                          // showZero={true}
                                          color="default"
                                          label={topic.publisher.length}
                                        />

                                        <Chip
                                          size="small"
                                          title="subscribers"
                                          // showZero={true}
                                          color={topic.subscriber.length > 0 ? "default" : "warning"}
                                          label={topic.subscriber.length}
                                        />
                                      </Stack>
                                    )}
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

              {showServices && (
                <>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold", marginTop: 1 }}>
                      Available Services:
                      {` [${Array.from(node.services.values()).length}]`}
                    </Box>
                  </Typography>

                  {Array.from(node.services.values()).length > 0 && (
                    <TableContainer component={Paper}>
                      <Table size="small" aria-label="a dense table">
                        <TableBody>
                          {Array.from(node.services.values()).map((service) => (
                            <TableRow key={service.name}>
                              <TableCell style={{ padding: 0 }}>
                                <Stack direction="row" spacing={1}>
                                  <OverflowMenuService
                                    onClick={onServiceClick}
                                    serviceName={service.name}
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
                                      onServiceClick(EMenuService.SERVICE_CALL, service.name, node.providerId)
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
                            <Stack direction="row" paddingBottom={0.5}>
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
                                          <JSONTree
                                            data={parameter.value}
                                            sortObjectKeys={true}
                                            theme={currentTheme}
                                            invertTheme={false}
                                            hideRoot={false}
                                            shouldExpandNodeInitially={() => {
                                              return true;
                                            }}
                                          />
                                        )}
                                      </TableCell>
                                      {typeof parameter.value !== "object" ? (
                                        <TableCell style={{ padding: 0 }}>{JSON.stringify(parameter.value)}</TableCell>
                                      ) : (
                                        <TableCell style={{ padding: 0 }}></TableCell>
                                      )}
                                    </TableRow>
                                  ))}
                                </TableBody>
                              </Table>
                            </TableContainer>
                          )}
                        </Stack>
                      );
                    } else {
                      return <></>;
                    }
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

export default NodesDetailsPanel;
