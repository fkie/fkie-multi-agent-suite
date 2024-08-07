import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import RefreshIcon from "@mui/icons-material/Refresh";
import {
  Alert,
  AlertTitle,
  Box,
  IconButton,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Tooltip,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import SearchBar from "../../../components/UI/SearchBar";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { findIn, generateUniqueId } from "../../../utils";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../../utils/events";
import { LAYOUT_TABS, LAYOUT_TAB_SETS, LayoutTabConfig } from "../layout";
import OverflowMenuProviderSelector from "./OverflowMenuProviderSelector";
import ServiceCallerPanel from "./ServiceCallerPanel";

const headers = [
  {
    key: "actions",
    header: "Actions",
  },
  {
    key: "name",
    header: "Name",
  },
  {
    key: "srvtype",
    header: "Message Type",
  },
];

function ServicesPanel({ initialSearchTerm = "" }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [services, setServices] = useState([]);
  const [filteredServices, setFilteredServices] = useState([]);
  const [providers, setProviders] = useState({}); // {<serviceName: string>: Map< <providerId: string>: <name: string>>}

  const getServiceList = useCallback(async () => {
    // Get services from the ros node list of each provider.
    const serviceList = [];
    const providerList = {};
    rosCtx.mapProviderRosNodes.forEach((nodeList) => {
      nodeList.forEach((node) => {
        node.services.forEach((service) => {
          if (!serviceList.find((t) => t.name === service.name)) {
            serviceList.push(service);
            const newMap = new Map();
            newMap.set(node.providerId, node.providerName);
            providerList[service.name] = newMap;
          } else {
            providerList[service.name].set(node.providerId, node.providerName);
          }
        });
      });
    });
    // sort services by name
    serviceList.sort(function (a, b) {
      return a.name.localeCompare(b.name);
    });

    setServices(serviceList);
    setFilteredServices(serviceList);
    setProviders(providerList);
  }, [rosCtx.mapProviderRosNodes, setProviders, setFilteredServices]);

  // debounced search callback
  const onSearch = useDebounceCallback((searchTerm) => {
    if (!searchTerm) {
      setFilteredServices(services);
      return;
    }

    const newFilteredServices = services.filter((service) => {
      const isMatch = findIn(searchTerm, [service.name, service.srvtype]);
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

  const onCallActionClick = useCallback((actionType, providerId, providerName, service) => {
    if (actionType === "CALL") {
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          `call-service-${generateUniqueId()}`,
          `Call Service - ${service.name}`,
          <ServiceCallerPanel serviceName={service.name} providerId={providerId} />,
          true,
          LAYOUT_TAB_SETS.BORDER_RIGHT,
          new LayoutTabConfig(false, LAYOUT_TABS.SERVICES)
        )
      );
    }
  }, []);

  // Get topic list when mounting the component
  useEffect(() => {
    getServiceList();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.mapProviderRosNodes]);

  // Initial filter when setting the topics
  useEffect(() => {
    onSearch(initialSearchTerm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [services]);

  return (
    <Box height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
      {filteredServices && (
        <Stack
          spacing={1}
          sx={{
            height: "100%",
            display: "flex",
          }}
        >
          <Stack direction="row" spacing={1} alignItems="center">
            <SearchBar
              onSearch={onSearch}
              placeholder="Filter Services (<space> for OR, + for AND)"
              defaultValue={initialSearchTerm}
              fullWidth
            />
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
          </Stack>

          {filteredServices.length > 0 && (
            <TableContainer
              sx={{
                flex: 1,
                minHeight: 0,
              }}
            >
              <Table stickyHeader size="small" aria-label="services table">
                <TableHead>
                  <TableRow key="header-service">
                    {headers.map((header) => {
                      if (header.key === "name") {
                        return (
                          <TableCell key={header.key} sx={{ fontWeight: "bold" }}>
                            {header.header} [{filteredServices.length}]
                          </TableCell>
                        );
                      }
                      return (
                        <TableCell key={header.key} sx={{ fontWeight: "bold" }}>
                          {header.header}
                        </TableCell>
                      );
                    })}
                  </TableRow>
                </TableHead>
                <TableBody>
                  {filteredServices.map((row) => {
                    const rowId = row.name;
                    return (
                      <TableRow key={rowId}>
                        <TableCell key={`${rowId}-actions`}>
                          <Tooltip title="Call service" enterDelay={1000} disableInteractive>
                            <div>
                              <OverflowMenuProviderSelector
                                onClick={onCallActionClick}
                                providerMap={providers[row.name]}
                                icon={PlayArrowIcon}
                                actionType="CALL"
                                args={row}
                              />
                            </div>
                          </Tooltip>
                        </TableCell>
                        <TableCell key={`${rowId}-name`}>
                          <Stack direction="row">
                            {row.name}
                            {/* <CopyButton value={row.name} /> */}
                          </Stack>
                        </TableCell>
                        <TableCell key={`${rowId}-type`}>
                          <Stack direction="row">
                            {row.srvtype}
                            {/* <CopyButton value={row.srvtype} /> */}
                          </Stack>
                        </TableCell>
                      </TableRow>
                    );
                  })}
                </TableBody>
              </Table>
            </TableContainer>
          )}

          {filteredServices.length === 0 && (
            <Alert severity="info" style={{ minWidth: 0 }}>
              <AlertTitle>No Services found</AlertTitle>
            </Alert>
          )}
        </Stack>
      )}
    </Box>
  );
}

ServicesPanel.propTypes = {
  initialSearchTerm: PropTypes.string,
};

export default ServicesPanel;
