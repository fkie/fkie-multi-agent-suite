import SyncAltOutlinedIcon from "@mui/icons-material/SyncAltOutlined";
import { Button, IconButton, Stack, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { LAYOUT_TABS } from "@/renderer/components/layout";
import { emitOpenComponent } from "@/renderer/components/layout/events";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { RosService, RosTopicId, ServiceExtendedInfo, TServiceNodeInfo } from "@/renderer/models";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { removeDDSuid } from "@/renderer/utils";
import { CopyButton } from "../UI";

type ServiceDetailsItemsProps = {
  providerId: string | undefined;
  serviceId: RosTopicId;
  nodeName: string;
};

export default function ServiceDetailsItem(props: ServiceDetailsItemsProps): JSX.Element {
  const { providerId, serviceId, nodeName = "" } = props;

  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();
  const [serviceInfo, setServiceInfo] = useState<ServiceExtendedInfo | undefined>();
  const [showInfo, setShowInfo] = useState<boolean>(false);
  const [colorizeHosts] = useSetting<boolean>("colorizeHosts");

  function onServiceCallClick(service: ServiceExtendedInfo, event: React.MouseEvent<HTMLButtonElement>): void {
    navCtx.openServiceCaller({
      providerId: providerId || "",
      serviceName: service.name,
      serviceType: service.srvType,
      externalKeyModifier: event.shiftKey || event.metaKey,
      forceOpenTerminal: false,
    });
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
    (providerId: string): object => {
      if (providerId && colorizeHosts) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: rosCtx.providerColor(providerId),
          borderLeftWidth: "0.5em",
        };
      }
      return { flexGrow: 1, alignItems: "center" };
    },
    [colorizeHosts, rosCtx.providerColor]
  );

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  const createInfo = useMemo(() => {
    if (!serviceInfo) return <></>;
    let serviceName = serviceId.name;
    const fullNodeName = nodeName ? `${nodeName}/` : "";
    if (fullNodeName && serviceName.startsWith(fullNodeName)) {
      serviceName = serviceName.replace(fullNodeName, "");
    }

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
              onServiceCallClick(serviceInfo, event);
              event?.stopPropagation();
            }}
            size="small"
          >
            <SyncAltOutlinedIcon sx={{ fontSize: (theme) => theme.typography.fontSize }} />
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
              logCtx.info(`${serviceId.name} copied`, "", `${serviceId.name} copied`);
            }}
          >
            {`${serviceName}`}
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
            <Stack
              key={item.nodeId}
              paddingLeft={"0.5em"}
              alignItems="center"
              direction="row"
              spacing="0.5em"
              style={getHostStyle(item.providerId)}
            >
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
                  navCtx.setSelected("service-panel", [id], true);
                  // inform details panel tab about selected nodes by user
                  emitOpenComponent({
                    id: LAYOUT_TABS.DETAILS,
                    title: "Details",
                    component: LAYOUT_TABS.DETAILS,
                    closable: false,
                    toNodeId: "details-set",
                  });
                }}
              >
                {provNodeName}
              </Button>
              <CopyButton value={provNodeName} fontSize="0.7em" />
            </Stack>
          );
        })}
      </Stack>
    );
  }, [serviceInfo]);

  return (
    <Stack direction="column" alignItems="left" spacing={0}>
      {createInfo}
      {showInfo && createExtendedInfo}
    </Stack>
  );
}
