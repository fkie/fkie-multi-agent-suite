import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RefreshIcon from "@mui/icons-material/Refresh";
import { Box, ButtonGroup, IconButton, Stack, Tooltip } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { SearchBar, ServiceTreeItem } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { findIn } from "../../../utils/index";
import { LAYOUT_TAB_SETS, LAYOUT_TABS, LayoutTabConfig } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";
import ServiceCallerPanel from "./ServiceCallerPanel";

class ServiceExtendedInfo {
  id;

  name;

  srvType = "";

  providerId = "";

  providerName = "";

  nodeProviders = [];

  nodeRequester = [];

  // providers = {}; // {providerId: string, providerName: string}
  // nodes = {}; //{(providerId: string, itemId: string): nodeName: string}

  constructor(service, providerId, providerName) {
    this.id = `${service.name}/${providerName}`;
    this.name = service.name;
    this.srvType = service.srvtype;
    this.providerId = providerId;
    this.providerName = providerName;
  }

  addProvider(nodeName, nodeId) {
    this.nodeProviders.push({ nodeName, nodeId });
  }
  addRequester(nodeName, nodeId) {
    this.nodeRequester.push({ nodeName, nodeId });
  }
}

function ServicesPanel({ initialSearchTerm = "" }) {
  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [services, setServices] = useState([]);
  const [filteredServices, setFilteredServices] = useState([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState([]);
  const [expanded, setExpanded] = useState([]);
  const [expandedFiltered, setExpandedFiltered] = useState([]);
  const [serviceForSelected, setServiceForSelected] = useState(null);
  const [selectedItem, setSelectedItem] = useState("");

  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  function genKey(items) {
    return `${items.join("#")}`;
  }

  const getServiceList = useCallback(async () => {
    // Get services from the ros node list of each provider.
    const newServicesMap = new Map();
    rosCtx.mapProviderRosNodes.forEach((nodeList, providerId) => {
      nodeList.forEach((node) => {
        node.services.forEach((service) => {
          const key = genKey([node.providerId, service.name, service.srvtype]);
          const serviceInfo = newServicesMap.get(key);
          if (serviceInfo) {
            service.provider.map((item) => {
              serviceInfo.addProvider(item.split("-")[0], item);
            });
            service.requester.map((item) => {
              serviceInfo.addRequester(item.split("-")[0], item);
            });
          } else {
            const serviceInfo = new ServiceExtendedInfo(service, node.providerId, node.providerName);
            service.provider.map((item) => {
              serviceInfo.addProvider(item.split("-")[0], item);
            });
            service.requester.map((item) => {
              serviceInfo.addRequester(item.split("-")[0], item);
            });
            newServicesMap.set(key, serviceInfo);
          }
        });
      });
    });
    const newServices = Array.from(newServicesMap.values());
    newServices.sort(function (a, b) {
      return a.name.localeCompare(b.name);
    });
    setServices(newServices);
  }, [rosCtx.mapProviderRosNodes, setServices, setFilteredServices]);

  // debounced search callback
  const onSearch = useDebounceCallback((searchTerm) => {
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

  const onCallService = useCallback(
    (service, external, openInTerminal = false) => {
      // TODO: open in external window like subscriber
      // rosCtx.openSubscriber(topic.providerId, topic.name, true, false, external, openInTerminal);
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `call-service-${service.name}-${service.providerId}}`,
          `Call Service - ${service.name}`,
          <ServiceCallerPanel serviceName={service.name} providerId={service.providerId} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, LAYOUT_TABS.SERVICES)
        )
      );
    },
    [rosCtx]
  );

  // Get service list when mounting the component
  useEffect(() => {
    getServiceList();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.mapProviderRosNodes]);

  // Initial filter when setting the services
  useEffect(() => {
    onSearch(searchTerm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [services, searchTerm]);

  // create tree based on service namespace
  // services are grouped only if more then one is in the group
  const fillTree = (fullPrefix, serviceGroup, itemId) => {
    const groupKeys = [];
    const byPrefixP1 = new Map("", []);
    // create a map with simulated tree for the namespaces of the service list
    Object.entries(serviceGroup).forEach(([key, serviceInfo]) => {
      const nameSuffix = serviceInfo.id.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName).push({ restNameSuffix, serviceInfo });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, serviceInfo }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ serviceInfo }]);
      }
    });

    let count = 0;
    // create result
    const newFilteredServices = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const groupKey = itemId ? `${itemId}-${groupName}` : groupName;
      const newFullPrefix = `${fullPrefix}/${groupName}`;
      if (value.length > 1) {
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
        let srvType = undefined;
        value.map((item) => {
          if (srvType === undefined) {
            srvType = item.serviceInfo.srvType;
          } else if (srvType !== item.serviceInfo.srvType) {
            if (srvType !== "") {
              srvType = "";
            }
          }
        });
        newFilteredServices.push([
          {
            groupKey: groupKey,
            groupName: `/${groupName}`,
            services: subResult.services,
            count: subResult.count,
            fullPrefix: newFullPrefix,
            srvType: srvType ? srvType : "",
            groupKeys: groupKeys,
          },
        ]);
      } else {
        newFilteredServices.push(value[0].serviceInfo);
        if (value[0].serviceInfo.providerName !== groupName) {
          // since the same service can be on multiple provider
          // we count only services
          count += 1;
        }
      }
    });
    return { services: newFilteredServices, count, groupKeys };
  };

  // create services tree from filtered service list
  useEffect(() => {
    const tree = fillTree("", filteredServices, "");
    setRootDataList(tree.services);
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      setExpandedFiltered(tree.groupKeys);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [filteredServices]);

  const serviceTreeToStyledItems = useCallback((rootPath, treeItem, selectedItem) => {
    if (Array.isArray(treeItem)) {
      return treeItem.map((item) => {
        return (
          <ServiceTreeItem
            key={item.groupKey}
            itemId={item.groupKey}
            labelRoot={rootPath}
            labelText={item.groupName}
            labelCount={item.count}
            labelInfo={item.srvType}
            selectedItem={selectedItem}
          >
            {item.services.map((subItem) => {
              return serviceTreeToStyledItems(item.fullPrefix, subItem, selectedItem);
            })}
          </ServiceTreeItem>
        );
      });
    }
    return (
      <ServiceTreeItem
        key={genKey([treeItem.name, treeItem.srvType, treeItem.providerId])}
        itemId={genKey([treeItem.name, treeItem.srvType, treeItem.providerId])}
        labelRoot={rootPath}
        labelText={`${treeItem.name}`}
        labelInfo={treeItem.srvType}
        color="#1a73e8"
        bgColor="#e8f0fe"
        colorForDarkMode="#B8E7FB"
        bgColorForDarkMode="#071318"
        serviceInfo={treeItem}
        selectedItem={selectedItem}
      />
    );
  }, []);

  useEffect(() => {
    const selectedServices = filteredServices.filter((item) => {
      return genKey([item.name, item.srvType, item.providerId]) === selectedItem;
    });
    if (selectedServices?.length >= 0) {
      setServiceForSelected(selectedServices[0]);
    } else {
      setServiceForSelected(null);
    }
  }, [filteredServices, selectedItem]);

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
                onCallService(serviceForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey);
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
        onSelectedItemsChange={(event, itemId) => {
          setSelectedItem(itemId);
          const index =
            searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS
              ? expanded.indexOf(itemId)
              : expandedFiltered.indexOf(itemId);
          const copyExpanded = [...(searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
          if (index === -1) {
            copyExpanded.push(itemId);
          } else {
            copyExpanded.splice(index, 1);
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
  }, [expanded, expandedFiltered, rootDataList, searchTerm]);

  const createPanel = useMemo(() => {
    return (
      <Box height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
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
            <Box height="100%" sx={{ borderRight: "solid", borderColor: "#D3D3D3", borderWidth: 1 }}>
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
  }, [rootDataList, expanded, expandedFiltered, searchTerm, selectedItem, serviceForSelected]);
  return createPanel;
}

ServicesPanel.propTypes = {
  initialSearchTerm: PropTypes.string,
};

export default ServicesPanel;
