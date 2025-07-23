import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import RosContext from "@/renderer/context/RosContext";
import {
  RosNode,
  RosService,
  RosTopicId,
  ServiceExtendedInfo,
  TServiceNodeInfo,
} from "@/renderer/models";
import { LAYOUT_TAB_SETS, LAYOUT_TABS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import ServiceCallerPanel from "@/renderer/pages/NodeManager/panels/ServiceCallerPanel";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { generateUniqueId, removeDDSuid } from "@/renderer/utils";
import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import { Button, IconButton, Stack, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { forwardRef, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

type ServiceDetailsItemsProps = {
  providerId: string | undefined;
  serviceId: RosTopicId;
};

const ServiceDetailsItem = forwardRef<HTMLDivElement, ServiceDetailsItemsProps>(
  function ServiceDetailsItem(props, ref) {
    const { providerId, serviceId } = props;

    const logCtx = useContext(LoggingContext);
    const navCtx = useContext(NavigationContext);
    const rosCtx = useContext(RosContext);
    const [allServices, setAllServices] = useState<ServiceExtendedInfo[]>([]);
    const [showInfo, setShowInfo] = useState<boolean>(false);

    function onServiceCallClick(service: ServiceExtendedInfo): void {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `call-service-${generateUniqueId()}`,
          service.name,
          <ServiceCallerPanel serviceName={service.name} serviceType={service.srvType} providerId={providerId || ""} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, LAYOUT_TABS.SERVICES)
        )
      );
    }

    function updateServiceList(): void {
      if (providerId) {
        const provider = rosCtx.getProviderById(providerId);
        if (provider) {
          const rosServices: RosService[] = provider?.rosServices.filter(
            (item) => item.name === serviceId.name && item.srv_type === serviceId.msg_type
          );
          // setAllTopics(rosTopics);
          setAllServices(
            rosServices.map((rs: RosService) => {
              let serviceInfo: ServiceExtendedInfo | undefined = undefined;
              // Get topics from the ros node list of each provider.
              for (const provider of rosCtx.providers) {
                for (const prv of rs.provider || []) {
                  const rosNode = provider.rosNodes.find((node: RosNode) => node.id === prv);
                  if (rosNode) {
                    if (serviceInfo === undefined) {
                      serviceInfo = new ServiceExtendedInfo(rs, provider.id || "", provider.name() || "");
                    }
                    serviceInfo?.addProvider(rosNode.id.split("-")[0], rosNode.id);
                  }
                }
                for (const prv of rs.provider || []) {
                  const rosNode = provider.rosNodes.find((node: RosNode) => node.id === prv);
                  if (rosNode) {
                    if (serviceInfo === undefined) {
                      serviceInfo = new ServiceExtendedInfo(rs, provider.id || "", provider.name() || "");
                    }
                    serviceInfo?.addRequester(rosNode.id.split("-")[0], rosNode.id);
                  }
                }
              }
              return serviceInfo || new ServiceExtendedInfo(rs, provider.id || "", provider.name() || "");
            })
          );
        }
      }
    }

    useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, () => {
      updateServiceList();
    });

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    useEffect(() => {
      updateServiceList();
    }, [providerId, rosCtx.providers]);

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    const createInfo = useMemo(() => {
      return (
        <Stack direction="row" alignItems="center" spacing={0}>
          {allServices.map((rs: ServiceExtendedInfo, index: number) => {
            return (
              <Stack
                key={`service-${rs.name}-${index}`}
                alignItems="center"
                direction="row"
                margin={0}
                spacing={"0.1em"}
                style={{ display: "flex", flexGrow: 1, borderBottom: `1px solid ${alpha(grey[600], 0.4)}` }}
              >
                <IconButton
                  style={{ color: "#09770fff" }}
                  onClick={(event) => {
                    onServiceCallClick(rs);
                    event?.stopPropagation();
                  }}
                  size="small"
                >
                  <PlayArrowRoundedIcon fontSize="inherit" />
                </IconButton>
                <Button
                  size="small"
                  style={{
                    marginLeft: 1,
                    textTransform: "none",
                    justifyContent: "left",
                  }}
                  onClick={() => setShowInfo((prev) => !prev)}
                  onDoubleClick={() => {
                    navigator.clipboard.writeText(serviceId.name);
                    logCtx.success(`${serviceId.name} copied`);
                  }}
                >
                  {`${serviceId.name}`}
                </Button>
              </Stack>
            );
          })}
        </Stack>
      );
    }, [allServices]);

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    const createExtendedInfo = useMemo(() => {
      return allServices.map((rs: ServiceExtendedInfo, index: number) => {
        return (
          <Stack
            key={`info-${rs.name}-${index}`}
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
                  navigator.clipboard.writeText(rs.srvType);
                  logCtx.success(`${rs.srvType} copied`);
                }}
              >
                {rs.srvType}
              </Button>
            </Stack>
            <Typography fontWeight="500" fontStyle="italic" fontSize="small">
              Provider [{rs.nodeProviders?.length || 0}]:
            </Typography>
            {rs.nodeProviders?.map((item: TServiceNodeInfo) => {
              const pubNodeName = removeDDSuid(item.nodeId);
              return (
                <Stack key={item.nodeId} paddingLeft={"1em"} direction="row">
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
                      const id: string = `${rs.providerId}${item.nodeId.replaceAll("/", "#")}`;
                      navCtx.setSelectedNodes([id], true);
                      // inform details panel tab about selected nodes by user
                      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                    }}
                  >
                    {pubNodeName}
                  </Button>
                </Stack>
              );
            })}
            {/* <Typography fontWeight="500" fontStyle="italic" fontSize="small">
              Requester [{rs.nodeRequester.length || 0}]:
            </Typography>
            {rs.nodeRequester.map((item: TServiceNodeInfo) => {
              const pubNodeName = removeDDSuid(item.nodeId);
              return (
                <Stack key={item.nodeId} paddingLeft={"1em"} direction="row">
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
                      const id: string = `${rs.providerId}${item.nodeId.replaceAll("/", "#")}`;
                      navCtx.setSelectedNodes([id], true);
                      // inform details panel tab about selected nodes by user
                      emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                    }}
                  >
                    {pubNodeName}
                  </Button>
                </Stack>
              );
            })} */}
          </Stack>
        );
      });
    }, [allServices]);

    return (
      <Stack direction="column" alignItems="left" spacing={0} ref={ref}>
        {createInfo}
        {showInfo && createExtendedInfo}
      </Stack>
    );
  }
);

export default ServiceDetailsItem;
