import ArrowBackIosIcon from "@mui/icons-material/ArrowBackIos";
import ArrowForwardIosIcon from "@mui/icons-material/ArrowForwardIos";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { Alert, AlertTitle, Box, Button, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { useCallback, useContext, useEffect, useMemo, useReducer, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import JsonView from "react18-json-view";

import ServiceDetailsItem from "@/renderer/components/NodeDetails/ServiceDetailsItem";
import { default as TopicDetailsItem } from "@/renderer/components/NodeDetails/TopicDetailsItem";
import { colorFromHostname, getDiagnosticStyle } from "@/renderer/components/UI/Colors";
import CopyButton from "@/renderer/components/UI/CopyButton";
import Tag from "@/renderer/components/UI/Tag";
import { NavigationContext } from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import {
  LifecycleState,
  RosNode,
  RosNodeStatus,
  RosParameter,
  RosTopic,
  RosTopicId,
  getDiagnosticLevelName,
  getFileName,
} from "@/renderer/models";
import {
  EVENT_NODE_DIAGNOSTIC,
  EVENT_NODE_LIFECYCLE,
  EVENT_PROVIDER_ROS_SERVICES,
  EVENT_PROVIDER_ROS_TOPICS,
} from "@/renderer/providers/eventTypes";
import { EventNodeDiagnostic, TEventNodeLifecycle } from "@/renderer/providers/events";
import SystemInformationPanel from "./SystemInformationPanel";

function compareTopics(a: RosTopicId | RosTopic, b: RosTopicId | RosTopic): number {
  if (a.name < b.name) {
    return -1;
  }
  if (a.name > b.name) {
    return 1;
  }
  return 0;
}

export default function DetailsPanel(): JSX.Element {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const navCtx = useContext(NavigationContext);

  const [indexOfSelected, setIndexOfSelected] = useState<number>(0);
  const [nodeShow, setNodeShow] = useState<RosNode | undefined>(undefined);
  const [logPaths, setLogPaths] = useState({}); // {node.idGlobal: LogPathItem}
  const [lifecycle, setLifecycle] = useState<LifecycleState | undefined>();
  const [updateDiagnostics, forceUpdateDiagnostics] = useReducer((x) => x + 1, 0);
  const [updateServices, forceUpdateServices] = useReducer((x) => x + 1, 0);
  const [updateTopics, forceUpdateTopics] = useReducer((x) => x + 1, 0);

  const [showDiagnosticHistory, setShowDiagnosticHistory] = useLocalStorage(
    "DetailsPanel:showDiagnosticHistory",
    false
  );
  const [showNodeInfo, setShowNodeInfo] = useLocalStorage("DetailsPanel:showNodeInfo", true);
  const [showPublishers, setShowPublishers] = useLocalStorage("DetailsPanel:showPublishers", true);
  const [showSubscribers, setShowSubscribers] = useLocalStorage("DetailsPanel:showSubscribers", true);
  const [showServices, setShowServices] = useLocalStorage("DetailsPanel:showServices", true);
  const [showConnections] = useLocalStorage("DetailsPanel:showConnections", true);
  const [showLaunchParameter, setShowLaunchParameter] = useLocalStorage("DetailsPanel:showLaunchParameter", true);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);

  useEffect(() => {
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    const idToShow = navCtx.selectedNodes[indexOfSelected];
    setNodeShow(rosCtx.nodeMap.get(idToShow));
  }, [navCtx.selectedNodes, rosCtx.nodeMap, indexOfSelected]);

  useEffect(() => {
    setIndexOfSelected(0);
  }, [navCtx.selectedNodes]);

  useEffect(() => {
    if (!nodeShow) return;
    const provider = rosCtx.getProviderById(nodeShow.providerId as string, true);
    setLifecycle(provider?.getLifecycleForNode(nodeShow.id));
  }, [nodeShow]);

  useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, () => {
    forceUpdateServices();
  });

  useCustomEventListener(EVENT_PROVIDER_ROS_TOPICS, () => {
    forceUpdateTopics();
  });

  useCustomEventListener(EVENT_NODE_DIAGNOSTIC, (data: EventNodeDiagnostic) => {
    if (data.node.name === nodeShow?.name) {
      forceUpdateDiagnostics();
    }
  });

  useCustomEventListener(EVENT_NODE_LIFECYCLE, (data: TEventNodeLifecycle) => {
    if (!nodeShow) return;
    if (data.lifecycle.id === nodeShow.id && data.provider.id === nodeShow.providerId) {
      setLifecycle(data.lifecycle);
    }
  });

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

  const createDiagnostics = useMemo(() => {
    if (!nodeShow) return <></>;
    if ((nodeShow.diagnostic?.diagnosticLevel || 0) > 0 || nodeShow.diagnostic?.getColor(isDarkMode)) {
      return (
        <Stack paddingTop="0.5em">
          <Stack
            direction="row"
            alignItems="center"
            onClick={() => {
              setShowDiagnosticHistory((prev) => !prev);
            }}
          >
            <ExpandMoreIcon
              style={{ transform: `${showDiagnosticHistory ? "" : "rotate(-90deg)"}` }}
              fontSize="small"
            />
            <Button
              size="small"
              style={{
                textTransform: "none",
                color: "inherit",
              }}
            >
              <Typography
                variant="body1"
                style={getDiagnosticStyle(nodeShow.diagnostic?.diagnosticLevel || 0)}
                marginBottom={1}
              >
                {getDiagnosticLevelName(nodeShow.diagnostic?.diagnosticLevel || 0)}:{" "}
                {nodeShow.diagnostic?.diagnosticMessage}
              </Typography>
            </Button>
          </Stack>
          {showDiagnosticHistory &&
            nodeShow.diagnostic?.diagnosticArray.map((da, daIndex) => {
              return da.status?.map((ds, dsIndex) => {
                return (
                  <Stack
                    // biome-ignore lint/suspicious/noArrayIndexKey: <explanation>
                    key={`${daIndex}-${dsIndex}-diagnostic-status`}
                    direction="column"
                  >
                    <Stack direction="row" width="100%" display="flex" flexGrow={1} flexDirection="row">
                      <Typography variant="body1" marginLeft="1.5em" marginRight="0.5em">
                        {daIndex + 1}:
                      </Typography>
                      <Typography variant="body1" style={getDiagnosticStyle(ds.level || 0)} paddingLeft="0.5em">
                        {getDiagnosticLevelName(ds.level || 0)}: {ds.message}
                      </Typography>
                    </Stack>
                    {ds.values?.map((dsItem) => {
                      return (
                        <Stack
                          key={`${dsItem.key}-diagnostic-keyvalue`}
                          direction="row"
                          width="100%"
                          display="flex"
                          flexGrow={1}
                          flexDirection="row"
                        >
                          <Typography variant="body1" marginLeft="3em" marginRight="0.5em" marginBottom={1}>
                            {dsItem.key}:
                          </Typography>
                          <Typography
                            variant="body1"
                            paddingLeft="0.5em"
                            marginBottom={1}
                            color={dsItem.key === "color" ? dsItem.value : undefined}
                          >
                            {dsItem.value}
                          </Typography>
                        </Stack>
                      );
                    })}
                  </Stack>
                );
              });
            })}
        </Stack>
      );
    }
    return <></>;
  }, [nodeShow, updateDiagnostics, showDiagnosticHistory]);

  const createNodeDetailsInfo = useMemo(() => {
    if (!nodeShow) return <></>;
    return (
      <Stack
        key={nodeShow.idGlobal}
        // spacing={1}
        alignItems="left"
      >
        <Stack
          direction="row"
          alignItems="center"
          onClick={() => {
            setShowNodeInfo((prev) => !prev);
          }}
        >
          <ExpandMoreIcon style={{ transform: `${showNodeInfo ? "" : "rotate(-90deg)"}` }} fontSize="small" />
          <Button
            size="small"
            style={{
              textTransform: "none",
              color: "inherit",
            }}
          >
            <Typography variant="caption" fontWeight="bold">
              Node Details:
            </Typography>
          </Button>
        </Stack>
        {showNodeInfo && (
          <Stack spacing="0.5em">
            {nodeShow.pid && Math.round(nodeShow.pid) > 0 && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="PID:" text={`${nodeShow.pid}`} wrap />
              </Stack>
            )}
            {nodeShow.processIds.length > 0 && (
              <Stack direction="row" spacing={0.5}>
                <Tag
                  color="default"
                  title={`PID${nodeShow.processIds.length > 1 ? "s" : ""}:`}
                  text={`${nodeShow.processIds.join(", ")}`}
                  tooltip={
                    "Process IDs of all processes found for this node, including the screens in which the nodes were started."
                  }
                  wrap
                />
              </Stack>
            )}
            {lifecycle?.state && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="Lifecycle:" text={`${lifecycle?.state}`} wrap />
              </Stack>
            )}
            {/* {(lifecycle?.available_transitions || []).length > 0 && (
              <Stack direction="row" spacing={0.5}>
                <Tag
                  color="default"
                  title="Lifecycle transitions:"
                  text={`${JSON.stringify(lifecycle?.available_transitions)}`}
                  wrap
                />
              </Stack>
            )} */}
            {nodeShow.id && nodeShow.id !== nodeShow.name && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="ID:" text={`${nodeShow.id}`} wrap />
              </Stack>
            )}
            {nodeShow.node_API_URI && nodeShow.node_API_URI.length > 0 && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="URI:" text={nodeShow.node_API_URI} wrap />
              </Stack>
            )}
            {nodeShow.masteruri && nodeShow.masteruri.length > 0 && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="MASTERURI:" text={nodeShow.masteruri} wrap />
              </Stack>
            )}
            {nodeShow.location && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="Location:" text={`${nodeShow.location} - ${nodeShow.providerName}`} wrap />
              </Stack>
            )}
            {nodeShow.container_name ? (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="Container:" text={`${nodeShow.container_name}`} wrap />
              </Stack>
            ) : (
              nodeShow.getAllContainers().length > 0 && (
                <Stack direction="row" spacing={0.5}>
                  <Tag
                    color="default"
                    title="Container:"
                    text={`${nodeShow.getAllContainers().length === 1 ? nodeShow.getAllContainers()[0] : JSON.stringify(nodeShow.getAllContainers())}`}
                    wrap
                  />
                </Stack>
              )
            )}
            {nodeShow.guid && (
              <Stack direction="row" spacing={0.5}>
                <Tag color="default" title="GID:" text={`${nodeShow.guid}`} wrap />
              </Stack>
            )}
            {nodeShow.screens && nodeShow.screens.length > 0 && (
              <Stack direction="row" spacing={0.5}>
                {nodeShow.screens.map((screen) => (
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
            {nodeShow.launchInfo.size > 0 && (
              <Stack direction="row" spacing={0.5}>
                {Array.from(nodeShow.launchInfo.keys()).map((launchPath) => (
                  <Tag
                    key={launchPath}
                    color="default"
                    title={`${launchPath === nodeShow.launchPath ? "*" : ""}Launch:`}
                    text={getFileName(launchPath)}
                    wrap
                    copyButton={launchPath}
                  />
                ))}
              </Stack>
            )}
            <Stack direction="column" spacing={0.5}>
              {logPaths[nodeShow.idGlobal]?.map((logItem) => {
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
              {!logPaths[nodeShow.idGlobal] && (
                <Stack direction="row" padding={0} spacing={0.5}>
                  <Button
                    type="submit"
                    // variant="contained"
                    size="small"
                    color="warning"
                    onClick={async () => {
                      const logs = await rosCtx.getProviderById(nodeShow.providerId)?.getLogPaths([nodeShow.name]);
                      logPaths[nodeShow.idGlobal] = logs;
                      setLogPaths({ ...logPaths });
                    }}
                  >
                    get log paths
                  </Button>
                </Stack>
              )}
            </Stack>
          </Stack>
        )}
      </Stack>
    );
  }, [nodeShow, showNodeInfo, logPaths, lifecycle]);

  const createTopicsView = useMemo(() => {
    if (!nodeShow) return <></>;
    const provider = rosCtx.getProviderById(nodeShow.providerId);
    return (
      <Stack spacing="0">
        {nodeShow.subscribers && (
          <Stack paddingTop="0.5em" spacing={0}>
            <Stack
              direction="row"
              alignItems="center"
              onClick={() => {
                setShowSubscribers((prev) => !prev);
              }}
            >
              <ExpandMoreIcon style={{ transform: `${showSubscribers ? "" : "rotate(-90deg)"}` }} fontSize="small" />
              <Button
                size="small"
                style={{
                  textTransform: "none",
                  color: "inherit",
                }}
              >
                <Typography variant="caption" fontWeight="bold">
                  Subscribed topics:
                  {` [${nodeShow.subscribers.length}]`}
                </Typography>
              </Button>
            </Stack>
            {showSubscribers &&
              Array.from(nodeShow.subscribers.values())
                .sort(compareTopics)
                .map((topic: RosTopicId) => (
                  <TopicDetailsItem
                    key={`${topic.name}-${topic.msg_type}`}
                    providerId={provider?.id}
                    topicId={topic}
                    showConnections={showConnections}
                    nodeName={nodeShow.name}
                  />
                ))}
          </Stack>
        )}

        {nodeShow.publishers && (
          <Stack paddingTop="0.5em" spacing={0}>
            <Stack
              direction="row"
              alignItems="center"
              onClick={() => {
                setShowPublishers((prev) => !prev);
              }}
            >
              <ExpandMoreIcon style={{ transform: `${showPublishers ? "" : "rotate(-90deg)"}` }} fontSize="small" />
              <Button
                size="small"
                style={{
                  textTransform: "none",
                  color: "inherit",
                }}
              >
                <Typography variant="caption" fontWeight="bold">
                  Published topics:
                  {` [${nodeShow.publishers.length}]`}
                </Typography>
              </Button>
            </Stack>
            {showPublishers &&
              nodeShow.publishers
                .sort(compareTopics)
                .map((topic) => (
                  <TopicDetailsItem
                    key={`${topic.name}-${topic.msg_type}`}
                    providerId={provider?.id}
                    topicId={topic}
                    showConnections={showConnections}
                    nodeName={nodeShow.name}
                  />
                ))}
          </Stack>
        )}
      </Stack>
    );
  }, [nodeShow, updateTopics, showPublishers, showSubscribers, showConnections]);

  const createServicesView = useMemo(() => {
    if (!nodeShow) return <></>;
    const provider = rosCtx.getProviderById(nodeShow.providerId);
    return (
      <Stack paddingTop="0.5em" spacing={0}>
        <Stack
          direction="row"
          alignItems="center"
          onClick={() => {
            setShowServices((prev) => !prev);
          }}
        >
          <ExpandMoreIcon style={{ transform: `${showServices ? "" : "rotate(-90deg)"}` }} fontSize="small" />
          <Button
            size="small"
            style={{
              textTransform: "none",
              color: "inherit",
            }}
          >
            <Typography variant="caption" fontWeight="bold">
              Services:
              {` [${nodeShow.services?.length || 0}]`}
            </Typography>
          </Button>
        </Stack>

        {showServices &&
          nodeShow.services?.sort(compareTopics).map((service) => {
            return (
              <ServiceDetailsItem
                key={`${service.name}-${service.msg_type}`}
                providerId={provider?.id}
                serviceId={service}
                nodeName={nodeShow.name}
              />
            );
          })}
      </Stack>
    );
  }, [nodeShow, updateServices, showServices]);

  const createLaunchView = useMemo(() => {
    if (!nodeShow || !nodeShow.services) return <></>;
    return (
      <Stack paddingTop="0.5em" spacing={0}>
        <Stack
          direction="row"
          alignItems="center"
          onClick={() => {
            setShowLaunchParameter((prev) => !prev);
          }}
        >
          <ExpandMoreIcon style={{ transform: `${showLaunchParameter ? "" : "rotate(-90deg)"}` }} fontSize="small" />
          <Button
            size="small"
            style={{
              textTransform: "none",
              color: "inherit",
            }}
          >
            <Typography variant="caption" fontWeight="bold">
              Launch Parameter:
            </Typography>
          </Button>
        </Stack>
        {showLaunchParameter &&
          Array.from(nodeShow.launchInfo.keys()).map((launchPath: string) => {
            const launchInfo = nodeShow.launchInfo.get(launchPath);
            if (launchInfo) {
              return (
                <Stack key={launchPath} marginTop={"0.5em"}>
                  <Typography variant="caption">
                    <Box sx={{ fontWeight: "bold" }}>
                      {`${launchPath.split("/").slice(-1)} [${launchInfo.parametersJoined?.length}]`}
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
                      {launchInfo.sigkill_timeout && launchInfo.sigkill_timeout > 0 && (
                        <Typography variant="caption">
                          <Stack direction="row" spacing={0.5}>
                            <Box sx={{ fontWeight: "bold", color: "orange" }}>Kill on stop:</Box>
                            <Box sx={{ fontWeight: "normal" }}>{launchInfo.sigkill_timeout} ms</Box>
                          </Stack>
                        </Typography>
                      )}
                      <Tag
                        color="default"
                        title="CMD:"
                        text={`${launchInfo.cmd}`}
                        tooltip={`${launchInfo.cmd}`}
                        wrap
                        copyButton={`${launchInfo.cmd}`}
                      />
                    </Stack>
                  )}
                  {launchInfo.parametersJoined && launchInfo.parametersJoined.length > 0 && (
                    <JsonView
                      src={launchInfo.parametersJoined?.reduce((dictionary, param: RosParameter) => {
                        dictionary[param.name] = param.value;
                        return dictionary;
                      }, {})}
                      dark={isDarkMode}
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
                        if (params.indexOrName === undefined) {
                          // do not collapse root element
                          return false;
                        }
                        if (Array.isArray(params.node) && params.node.length === 0) {
                          return true;
                        }
                        if (params.depth > 3) return true;
                        return false;
                      }}
                    />
                  )}
                </Stack>
              );
            }
            // biome-ignore lint/correctness/useJsxKeyInIterable: <explanation>
            return <></>;
          })}
      </Stack>
    );
  }, [nodeShow, showServices, showLaunchParameter]);

  const createDetailsView = useMemo(() => {
    if (!nodeShow) return <></>;
    return (
      <Stack
        key={nodeShow.idGlobal}
        // spacing={1}
        alignItems="left"
        height="100%"
      >
        {navCtx.selectedNodes.length > 1 && (
          <Stack direction="row" justifyContent="center">
            <Typography color="grey" variant="body2">
              selected: {navCtx.selectedNodes.length}, displayed: 1
            </Typography>
          </Stack>
        )}
        <Stack
          direction="row"
          alignItems="center"
          paddingTop={0}
          marginBottom={0.5}
          sx={getHostStyle(nodeShow.providerName)}
        >
          {navCtx.nodesHistory.length > 0 ? (
            <Tooltip title={"Go back to last node"} placement="bottom">
              <IconButton
                color="error"
                onClick={(event) => {
                  navCtx.setSelectedFromHistory();
                  event?.stopPropagation();
                }}
                size="small"
              >
                <ArrowBackIosIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          ) : (
            indexOfSelected > 0 && (
              <Tooltip title={"Go to previous selected node"} placement="bottom">
                <IconButton
                  color="error"
                  onClick={(event) => {
                    setIndexOfSelected((prev) => prev - 1);
                    event?.stopPropagation();
                  }}
                  size="small"
                >
                  <ArrowBackIosIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )
          )}
          <Typography
            variant="subtitle1"
            style={{
              // cursor: "pointer",
              color: "#fff",
              backgroundColor: "#2196f3",
              flexGrow: 1,
            }}
            align="center"
          >
            <Stack spacing={0} sx={{ fontWeight: "bold", m: 0, paddingTop: "0.2em" }}>
              {nodeShow.namespace !== "/" && (
                <Tooltip title="namespace" placement="bottom" disableInteractive>
                  <Typography variant="subtitle2" align="center" style={{ overflowWrap: "anywhere" }}>
                    {nodeShow?.namespace}
                  </Typography>
                </Tooltip>
              )}
              <Box>
                {nodeShow.namespace !== "/"
                  ? nodeShow.name.replace(nodeShow.namespace, "").replace("/", "")
                  : nodeShow.name}
                <CopyButton value={nodeShow.name} fontSize={"inherit"} />
              </Box>
            </Stack>
          </Typography>
          {navCtx.selectedNodes.length - 1 > indexOfSelected && (
            <Tooltip title={"Go to next selected node"} placement="bottom">
              <IconButton
                color="error"
                onClick={(event) => {
                  setIndexOfSelected((prev) => prev + 1);
                  event?.stopPropagation();
                }}
                size="small"
              >
                <ArrowForwardIosIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          )}
        </Stack>
        <Box height="100%" overflow="auto">
          <Stack direction="row" spacing={0.5}>
            <Tag
              color={nodeShow.status === RosNodeStatus.RUNNING ? "success" : "default"}
              title=""
              // title={`${RosNodeStatusInfo[nodeShow.status]}`}
              text={nodeShow.status}
              wrap
            />
          </Stack>
          {createDiagnostics}
          {createNodeDetailsInfo}
          {createTopicsView}
          {createServicesView}
          {createLaunchView}
        </Box>
      </Stack>
    );
  }, [
    nodeShow,
    updateTopics,
    updateServices,
    updateDiagnostics,
    showNodeInfo,
    showPublishers,
    showConnections,
    showSubscribers,
    showServices,
    showLaunchParameter,
    showDiagnosticHistory,
    lifecycle,
    // onTopicClick,    <= causes unnecessary rebuilds
    // onServiceClick,  <= causes unnecessary rebuilds
    rosCtx,
    logPaths,
    navCtx.nodesHistory,
    navCtx.selectedNodes,
    settingsCtx.changed,
  ]);

  return (
    <Box width="100%" height="100%" sx={{ backgroundColor: backgroundColor }}>
      {nodeShow && navCtx.selectedProviders.length > 0 ? (
        <Stack
          // spacing={1}
          key={nodeShow.idGlobal}
          alignItems="left"
          height="100%"
        >
          {navCtx.selectedProviders.length > 1 && (
            <Stack direction="row" justifyContent="center">
              <Typography color="grey" variant="body2">
                selected: {navCtx.selectedProviders.length}, displayed: 1
              </Typography>
            </Stack>
          )}
          <SystemInformationPanel providerId={navCtx.selectedProviders[0]} />
        </Stack>
      ) : (
        createDetailsView
      )}

      {!nodeShow && (
        <Alert severity="info" style={{ minWidth: 0 }}>
          <AlertTitle>Please select a node or provider</AlertTitle>
        </Alert>
      )}
    </Box>
  );
}
