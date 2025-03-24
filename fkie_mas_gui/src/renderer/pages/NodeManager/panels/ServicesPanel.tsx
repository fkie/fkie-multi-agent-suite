import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RefreshIcon from "@mui/icons-material/Refresh";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { grey } from "@mui/material/colors";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import { ServiceGroupTreeItem, ServiceTreeItem } from "@/renderer/components/ServiceTreeView";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { RosService, ServiceExtendedInfo } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";
import { LAYOUT_TAB_SETS, LAYOUT_TABS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import ServiceCallerPanel from "./ServiceCallerPanel";

type TTreeItem = {
  groupKey: string;
  groupName: string;
  services: TTreeItem[];
  count: number;
  fullPrefix: string;
  srvType: string;
  groupKeys: string[];
  serviceInfo: ServiceExtendedInfo | null;
};

interface ServicesPanelProps {
  initialSearchTerm?: string;
}

const ServicesPanel = forwardRef<HTMLDivElement, ServicesPanelProps>(function ServicesPanel(props, ref) {
  const { initialSearchTerm = "" } = props;

  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [services, setServices] = useState<ServiceExtendedInfo[]>([]);
  const [filteredServices, setFilteredServices] = useState<ServiceExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expanded, setExpanded] = useState<string[]>([]);
  const [expandedFiltered, setExpandedFiltered] = useState<string[]>([]);
  const [serviceForSelected, setServiceForSelected] = useState<ServiceExtendedInfo>();
  const [selectedItem, setSelectedItem] = useState<string>("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [avoidGroupWithOneItem, setAvoidGroupWithOneItem] = useState<boolean>(
    settingsCtx.get("avoidGroupWithOneItem") as boolean
  );

  useEffect(() => {
    setAvoidGroupWithOneItem(settingsCtx.get("avoidGroupWithOneItem") as boolean);
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx, settingsCtx.changed]);

  function genKey(items: string[]): string {
    return `${items.join("#")}`;
  }

  async function updateServiceList(): Promise<void> {
    // Get services from the ros node list of each provider.
    const newServicesMap = new Map();
    rosCtx.providers.forEach((provider: Provider) => {
      provider.rosServices.forEach((service: RosService) => {
        const key = genKey([provider.id, service.name, service.srv_type]);
        const serviceInfo = newServicesMap.get(key);
        if (serviceInfo) {
          service.provider?.map((item) => {
            serviceInfo.addProvider(item.split("-")[0], item);
          });
          service.requester?.map((item) => {
            serviceInfo.addRequester(item.split("-")[0], item);
          });
        } else {
          const serviceInfo = new ServiceExtendedInfo(service, provider.id, provider.name());
          service.provider?.map((item) => {
            serviceInfo.addProvider(item.split("-")[0], item);
          });
          service.requester?.map((item) => {
            serviceInfo.addRequester(item.split("-")[0], item);
          });
          newServicesMap.set(key, serviceInfo);
        }
      });
    });

    const newServices: ServiceExtendedInfo[] = Array.from(newServicesMap.values());
    newServices.sort(function (a, b) {
      // sort groups first
      const aCountSep = (a.name.match(/\//g) || []).length;
      const bCountSep = (b.name.match(/\//g) || []).length;
      if (aCountSep === bCountSep) {
        return a.name.localeCompare(b.name);
      }
      return aCountSep < bCountSep ? 1 : -1;
    });
    setServices(newServices);
  }

  function getServiceList(): void {
    rosCtx.providers.forEach((provider) => {
      provider.updateRosServices();
    });
  }

  useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, () => {
    updateServiceList();
  });

  // debounced search callback
  const onSearch = useDebounceCallback((searchTerm: string) => {
    setSearchTerm(searchTerm);
    if (!searchTerm) {
      setFilteredServices(services);
      return;
    }

    const newFilteredServices = services.filter((service) => {
      const isMatch = findIn(searchTerm, [
        service.name,
        service.srvType,
        ...service.nodeProviders.map((item) => item.nodeName),
      ]);
      if (isMatch) {
        return isMatch;
      }
      // providers[service.name].forEach((providerName) => {
      //   if (findIn(searchTerm, [providerName])) {
      //     isMatch = true;
      //   }
      // });
      // if (isMatch) {
      //   return true;
      // }
      return false;
    });

    setFilteredServices(newFilteredServices);
  }, 300);

  function onCallService(service: ServiceExtendedInfo, external: boolean, openInTerminal: boolean): void {
    // TODO: open in external window like subscriber
    console.debug(`not implemented service parameter: ${external} ${openInTerminal}`);
    // navCtx.openSubscriber(topic.providerId, topic.name, true, false, external, openInTerminal);
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `call-service-${service.name}-${service.providerId}}`,
        `Call Service - ${service.name}`,
        <ServiceCallerPanel serviceName={service.name} serviceType={service.srvType} providerId={service.providerId} />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(false, LAYOUT_TABS.SERVICES)
      )
    );
  }

  // Get service list when mounting the component
  useEffect(() => {
    updateServiceList();
  }, [rosCtx.initialized]);

  // Initial filter when setting the services
  useEffect(() => {
    onSearch(searchTerm);
  }, [services, searchTerm]);

  // create tree based on service namespace
  // services are grouped only if more then one is in the group
  function fillTree(fullPrefix: string, serviceGroup: ServiceExtendedInfo[], itemId: string): TTreeItem {
    const byPrefixP1 = new Map<
      string,
      { restNameSuffix: string; serviceInfo: ServiceExtendedInfo; isGroup: boolean }[]
    >();
    // create a map with simulated tree for the namespaces of the service list
    serviceGroup.forEach((serviceInfo) => {
      const nameSuffix = serviceInfo.id.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 1) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName)?.push({ restNameSuffix, serviceInfo, isGroup: true });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, serviceInfo, isGroup: true }]);
        }
      } else {
        byPrefixP1.set(`${groupName}#${serviceInfo.srvType}`, [{ restNameSuffix: "", serviceInfo, isGroup: false }]);
      }
    });

    let count = 0;
    // create result
    const groupKeys: string[] = [];
    const newFilteredServices: TTreeItem[] = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const newFullPrefix = `${fullPrefix}/${groupName}`;
      const serviceValues = value.filter((item) => !item.isGroup);
      const groupValues = value.filter((item) => !serviceValues.includes(item));

      if (groupValues.length > 0) {
        const groupKey: string = itemId ? `${itemId}-${groupName}` : groupName;
        groupKeys.push(groupKey);
        const groupServices = value.map((item) => {
          return item.serviceInfo;
        });
        const subResult = fillTree(newFullPrefix, groupServices, groupKey);
        // the result count is 0 -> we added multiple provider for service with same name.
        if (subResult.count === 0) {
          count += 1;
        } else {
          count += subResult.count;
        }
        if (subResult.groupKeys.length > 0) {
          groupKeys.push(...subResult.groupKeys);
        }
        // if all services of the group have same message id, show it in the group info
        let srvType: string | undefined = undefined;
        groupValues.map((item) => {
          if (srvType === undefined) {
            srvType = item.serviceInfo.srvType;
          } else if (srvType !== item.serviceInfo.srvType) {
            if (srvType !== "") {
              srvType = "";
            }
          }
        });
        newFilteredServices.push({
          groupKey: groupKey,
          groupName: groupName,
          services: subResult.services,
          count: subResult.count,
          fullPrefix: fullPrefix,
          srvType: srvType ? srvType : "",
          groupKeys: groupKeys,
          serviceInfo: null,
        } as TTreeItem);
      }
      serviceValues.forEach((item) => {
        newFilteredServices.push({
          groupKey: "",
          groupName: "",
          services: [],
          count: 0,
          fullPrefix: fullPrefix,
          srvType: item.serviceInfo.srvType,
          groupKeys: groupKeys,
          serviceInfo: item.serviceInfo,
        } as TTreeItem);
        count += 1;
      });
      // if (value[0].serviceInfo.providerName !== groupName) {
      //   // since the same service can be on multiple provider
      //   // we count only services
      //   count += 1;
      // }
    });
    return { services: newFilteredServices, count, groupKeys } as TTreeItem;
  }

  // create services tree from filtered service list
  useEffect(() => {
    const tree = fillTree("", filteredServices, "");
    setRootDataList(tree.services);
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      setExpandedFiltered(tree.groupKeys);
    }
  }, [filteredServices]);

  const serviceTreeToStyledItems = useCallback(
    (rootPath: string, treeItem: TTreeItem, selectedItem: string) => {
      if (treeItem.serviceInfo) {
        return (
          <ServiceTreeItem
            key={genKey([treeItem.serviceInfo.name, treeItem.srvType, treeItem.serviceInfo.providerId])}
            itemId={genKey([treeItem.serviceInfo.name, treeItem.srvType, treeItem.serviceInfo.providerId])}
            rootPath={rootPath}
            serviceInfo={treeItem.serviceInfo}
            selectedItem={selectedItem}
          />
        );
      } else {
        if (avoidGroupWithOneItem && treeItem.services.length === 1) {
          // avoid groups with one item
          return serviceTreeToStyledItems(
            rootPath.length > 0 ? `${rootPath}/${treeItem.groupName}` : treeItem.groupName,
            treeItem.services[0],
            selectedItem
          );
        } else {
          return (
            <ServiceGroupTreeItem
              key={treeItem.groupKey}
              itemId={treeItem.groupKey}
              rootPath={rootPath}
              groupName={treeItem.groupName}
              countChildren={treeItem.count}
            >
              {treeItem.services.map((subItem) => {
                return serviceTreeToStyledItems("", subItem, selectedItem);
              })}
            </ServiceGroupTreeItem>
          );
        }
      }
    },
    [avoidGroupWithOneItem]
  );

  useEffect(() => {
    const selectedServices = filteredServices.filter((item) => {
      return genKey([item.name, item.srvType, item.providerId]) === selectedItem;
    });
    if (selectedServices?.length >= 0) {
      setServiceForSelected(selectedServices[0]);
    } else {
      setServiceForSelected(undefined);
    }
  }, [filteredServices, selectedItem]);

  const handleToggle = useCallback(
    (itemIds: string[]) => {
      if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
        setExpanded(itemIds);
      } else {
        setExpandedFiltered(itemIds);
      }
    },
    [searchTerm]
  );

  const createButtonBox = useMemo(() => {
    return (
      <ButtonGroup orientation="vertical" aria-label="service control group">
        <Tooltip
          title="Call service"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!serviceForSelected}
              size="medium"
              aria-label="call service"
              onClick={(event) => {
                if (serviceForSelected) {
                  onCallService(serviceForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey);
                }
              }}
            >
              <PlayArrowIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        {/* <Tooltip
          title="Call service in Terminal"
          placement="left"
          enterDelay={tooltipDelay}
          // enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              disabled={!serviceForSelected}
              size="medium"
              aria-label="call service in a terminal"
              onClick={(event) => {
                onCallService(serviceForSelected, event.nativeEvent.shiftKey, true);
              }}
            >
              <DvrIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip> */}
      </ButtonGroup>
    );
  }, [tooltipDelay, serviceForSelected]);

  const createTreeView = useMemo(() => {
    return (
      <SimpleTreeView
        aria-label="services"
        expandedItems={searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        expansionTrigger={"iconContainer"}
        onExpandedItemsChange={(_event, itemIds: string[]) => handleToggle(itemIds)}
        onSelectedItemsChange={(_event, itemId: string | null) => {
          setSelectedItem(itemId || "");
          const copyExpanded = [...(searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
          if (itemId) {
            const index =
              searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS
                ? expanded.indexOf(itemId)
                : expandedFiltered.indexOf(itemId);
            if (index === -1) {
              if (itemId) {
                copyExpanded.push(itemId);
              }
            } else {
              copyExpanded.splice(index, 1);
            }
          }
          if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
            setExpanded(copyExpanded);
          } else {
            setExpandedFiltered(copyExpanded);
          }
        }}
      >
        {rootDataList.map((item) => {
          return serviceTreeToStyledItems("", item, selectedItem);
        })}
      </SimpleTreeView>
    );
  }, [expanded, expandedFiltered, rootDataList, searchTerm, avoidGroupWithOneItem]);

  const createPanel = useMemo(() => {
    return (
      <Box ref={ref} height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
        <Stack
          spacing={1}
          height="100%"
          // sx={{
          //   height: '100%',
          //   display: 'flex',
          // }}
        >
          <Stack direction="row" spacing={0.5} alignItems="center">
            <Tooltip title="Reload service list" placement="left" disableInteractive>
              <IconButton
                size="small"
                onClick={() => {
                  getServiceList();
                }}
              >
                <RefreshIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
            <SearchBar
              onSearch={onSearch}
              placeholder="Filter Services (OR: <space>, AND: +, NOT: !)"
              defaultValue={initialSearchTerm}
              fullWidth
            />
          </Stack>
          <Stack direction="row" height="100%" overflow="auto">
            <Box height="100%" sx={{ boxShadow: `0px 0px 5px ${alpha(grey[600], 0.4)}` }}>
              {/* <Box height="100%" sx={{ borderRight: "solid", borderColor: `${alpha(grey[600], 0.4)}`, borderWidth: 1 }}> */}
              {/* <Paper elevation={0} sx={{ border: 1 }} height="100%"> */}
              {createButtonBox}
              {/* </Paper> */}
            </Box>
            <Box width="100%" height="100%" overflow="auto">
              {createTreeView}
            </Box>
          </Stack>
        </Stack>
      </Box>
    );
  }, [
    rootDataList,
    expanded,
    expandedFiltered,
    searchTerm,
    selectedItem,
    serviceForSelected,
    settingsCtx.changed,
    avoidGroupWithOneItem,
  ]);
  return createPanel;
});

export default ServicesPanel;
