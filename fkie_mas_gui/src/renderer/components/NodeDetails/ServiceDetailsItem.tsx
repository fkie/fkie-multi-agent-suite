import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import RosContext from "@/renderer/context/RosContext";
import SettingsContext from "@/renderer/context/SettingsContext";
import { RosService, RosTopicId, ServiceExtendedInfo, TServiceNodeInfo } from "@/renderer/models";
import { LAYOUT_TAB_SETS, LAYOUT_TABS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import ServiceCallerPanel from "@/renderer/pages/NodeManager/panels/ServiceCallerPanel";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { generateUniqueId, removeDDSuid } from "@/renderer/utils";
import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import { Button, IconButton, Stack, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import { colorFromHostname, CopyButton } from "../UI";

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
    const settingsCtx = useContext(SettingsContext);
    const [serviceInfo, setServiceInfo] = useState<ServiceExtendedInfo | undefined>();
    const [showInfo, setShowInfo] = useState<boolean>(false);
    const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    useEffect(() => {
      setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
    }, [settingsCtx, settingsCtx.changed]);

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
          const rosService: RosService | undefined = provider?.getService(serviceId);
          if (!rosService) {
            setServiceInfo(undefined);
            return;
          }
          const newServiceInfo: ServiceExtendedInfo = new ServiceExtendedInfo(rosService);
          // Get topics from the ros node list of each provider.
          for (const provider of rosCtx.providers) {
            for (const rosNode of provider.rosNodes) {
              newServiceInfo.add(rosNode);
            }
          }
          setServiceInfo(newServiceInfo);
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

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    const createInfo = useMemo(() => {
      if (!serviceInfo) return <></>;
      return (
        <Stack direction="row" alignItems="center" spacing={0}>
          <Stack
            key={`service-${serviceInfo.id}`}
            alignItems="center"
            direction="row"
            margin={0}
            spacing={"0.1em"}
            style={{ display: "flex", flexGrow: 1, borderBottom: `1px solid ${alpha(grey[600], 0.4)}` }}
          >
            <IconButton
              style={{ color: "#09770fff" }}
              onClick={(event) => {
                onServiceCallClick(serviceInfo);
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
                logCtx.info(`${serviceId.name} copied`);
              }}
            >
              {`${serviceId.name}`}
            </Button>
            {showInfo && <CopyButton value={serviceId.name} fontSize="0.7em" />}
          </Stack>
        </Stack>
      );
    }, [serviceInfo, showInfo]);

    // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
    const createExtendedInfo = useMemo(() => {
      if (!serviceInfo) return <></>;
      return (
        <Stack
          key={`info-${serviceInfo.id}`}
          style={{ marginLeft: 15, paddingLeft: 5, borderLeft: `1px dashed ${alpha(grey[600], 0.4)}` }}
        >
          <Stack direction="row" alignItems="center" spacing="0.3em">
            <Typography fontWeight="500" fontStyle="italic" fontSize="small">
              Type:
            </Typography>
            <Typography fontSize="small">{serviceInfo.srvType}</Typography>
            <CopyButton value={serviceInfo.srvType} fontSize="0.7em" />
          </Stack>
          <Typography fontWeight="500" fontStyle="italic" fontSize="small">
            Provider [{serviceInfo.nodeProviders?.length || 0}]:
          </Typography>
          {serviceInfo.nodeProviders?.map((item: TServiceNodeInfo) => {
            const provNodeName = removeDDSuid(item.nodeId);
            return (
              <Stack key={item.nodeId} paddingLeft={"0.5em"} alignItems="center" direction="row" spacing="0.5em" style={getHostStyle(item.providerName)}>
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
                    const id: string = `${item.providerId}${item.nodeId.replaceAll("/", "#")}`;
                    navCtx.setSelectedNodes([id], true);
                    // inform details panel tab about selected nodes by user
                    emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default"));
                  }}
                >
                  {provNodeName}
                </Button>
                <CopyButton value={provNodeName} fontSize="0.7em" />
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
    }, [serviceInfo]);

    return (
      <Stack direction="column" alignItems="left" spacing={0} ref={ref}>
        {createInfo}
        {showInfo && createExtendedInfo}
      </Stack>
    );
  }
);

export default ServiceDetailsItem;
