import RefreshIcon from "@mui/icons-material/Refresh";
import SyncAltOutlinedIcon from "@mui/icons-material/SyncAltOutlined";
import TroubleshootIcon from "@mui/icons-material/Troubleshoot";
import { alpha, Box, ButtonGroup, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { Virtuoso } from "react-virtuoso";

import ServiceTreeItem from "@/renderer/components/ServiceTreeView/ServiceTreeItem";
import TopicGroupTreeItem from "@/renderer/components/TopicTreeView/TopicGroupTreeItem";
import LongPressIconButton from "@/renderer/components/UI/LongPressIconButton";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { BUTTON_LOCATIONS } from "@/renderer/context/SettingsContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { ServiceExtendedInfo } from "@/renderer/models";
import { EVENT_PROVIDER_ROS_SERVICES } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";
import { TContentId } from "../../../components/layout/LayoutTabConfig";
import { EVENT_FILTER_SERVICES, TFilterText } from "../../../components/layout/events";

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

type TTreeResult = {
  services: TTreeItem[];
  count: number;
  groupKeys: string[];
};

// Selected item is now just the id (no domain tracking needed)
type TSelected = string | null;

type FlatRow = {
  id: string;
  type: "group" | "service";
  depth: number;
  treeItem: TTreeItem;
  rootPath: string;
};

interface ServicesPanelProps {
  contentId?: TContentId;
  initialSearchTerm?: string;
}

const EXPAND_ON_SEARCH_MIN_CHARS = 2;

/** Suffixes that identify action-related services in ROS 2 */
const ACTION_SERVICE_SUFFIXES = ["/_action/send_goal", "/_action/cancel_goal", "/_action/get_result"];

/** Returns true if the service name belongs to a ROS 2 action */
function isActionService(serviceName: string): boolean {
  return ACTION_SERVICE_SUFFIXES.some((suffix) => serviceName.endsWith(suffix));
}

export default function ServicesPanel({ contentId, initialSearchTerm = "" }: ServicesPanelProps): JSX.Element {
  const rosCtx = useRosContext();
  const navCtx = useNavigationContext();

  // Flat list of services for this panel (filtered by contentId)
  const [services, setServices] = useState<ServiceExtendedInfo[]>([]);
  const [searchTerm, setSearchTerm] = useState(initialSearchTerm);
  const [rootDataList, setRootDataList] = useState<TTreeItem[]>([]);
  const [expandedItems, setExpandedItems] = useState<string[]>([]);
  const [selected, setSelected] = useState<TSelected>(null);
  const [serviceForSelected, setServiceForSelected] = useState<ServiceExtendedInfo | undefined>();

  const [avoidGroupWithOneItem] = useSetting<boolean>("avoidGroupWithOneItem");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [buttonLocation] = useSetting<string>("buttonLocation");

  const genKey = useCallback((items: string[]): string => items.join("#"), []);

  /**
   * Build ServiceExtendedInfo list filtered by contentId.
   * - If contentId.providerId is set: only services from that provider.
   * - If contentId.domainId is set: services from all providers in that domain.
   */
  const updateServiceList = useCallback(async () => {
    if (!rosCtx.initialized) {
      return;
    }

    // serviceKey -> ServiceExtendedInfo
    const serviceMap = new Map<string, ServiceExtendedInfo>();

    const selectedDomainId = contentId?.domainId;
    const selectedProviderId = contentId?.providerId;

    for (const provider of rosCtx.providers) {
      // Filter by providerId if set
      if (selectedProviderId !== undefined) {
        if (provider.id !== selectedProviderId) {
          continue;
        }
      } else if (selectedDomainId !== undefined) {
        // Otherwise filter by domainId if set
        const providerDomainId = provider.connection?.domainId;
        if (providerDomainId !== selectedDomainId) {
          continue;
        }
      }

      for (const service of provider.rosServices) {
        // --- FILTER: skip action-related services ---
        if (isActionService(service.name)) {
          continue;
        }

        const key = genKey([service.name, service.srv_type]);
        let serviceInfo = serviceMap.get(key);

        if (!serviceInfo) {
          serviceInfo = new ServiceExtendedInfo(service);
          serviceMap.set(key, serviceInfo);
        }

        // attach all nodes from this provider to the aggregated ServiceExtendedInfo
        for (const rosNode of provider.rosNodes) {
          serviceInfo.add(rosNode);
        }
      }
    }

    const list = Array.from(serviceMap.values()).sort((a, b) => {
      const aSeps = (a.name.match(/\//g) || []).length;
      const bSeps = (b.name.match(/\//g) || []).length;
      if (aSeps === bSeps) {
        return a.name.localeCompare(b.name);
      }
      return bSeps - aSeps;
    });

    setServices(list);
  }, [rosCtx.initialized, rosCtx.providers, genKey, contentId]);

  /**
   * Trigger providers to refresh their ROS node lists, which will in turn update services.
   */
  const getServiceList = useCallback(() => {
    for (const provider of rosCtx.providers) {
      provider.updateRosNodes({}, true);
    }
  }, [rosCtx.providers]);

  /**
   * If a group contains exactly one child and no direct service itself,
   * merge the group with its child to avoid deep, single-branch hierarchies.
   */
  const flattenSingleChildGroups = useCallback((node: TTreeItem): TTreeItem => {
    if (node.services.length === 1 && !node.serviceInfo) {
      const child = node.services[0];

      node.groupName = `${node.groupName}/${child.groupName}`;
      node.groupKey = `${node.groupKey}-${child.groupKey}`;
      node.services = child.services;
      node.count = child.count;
      node.groupKeys = [...node.groupKeys, ...child.groupKeys];
      node.serviceInfo = child.serviceInfo;
      node.srvType = child.srvType || node.srvType;

      if (node.services.length === 1 && !node.serviceInfo) {
        return flattenSingleChildGroups(node);
      }
    } else {
      const nextChildren: TTreeItem[] = [];
      for (const child of node.services) {
        nextChildren.push(flattenSingleChildGroups(child));
      }
      node.services = nextChildren;
    }

    return node;
  }, []);

  /**
   * Tree structure builder for services (similar to topics).
   * Groups services by namespace segments.
   */
  const buildTree = useCallback(
    (servicesList: ServiceExtendedInfo[], avoidSingle: boolean): TTreeResult => {
      // Map of full path ("/foo/bar") to tree node
      const nodes = new Map<string, TTreeItem>();
      const rootNodes: TTreeItem[] = [];

      /**
       * Phase 1: create a node for every path segment and attach ServiceExtendedInfo
       * - For service "/foo/bar/baz", we create nodes for:
       *   "/foo", "/foo/bar", "/foo/bar/baz"
       * - Only the last segment (leaf) holds serviceInfo
       */
      for (const service of servicesList) {
        const parts = service.name.split("/").filter(Boolean);
        let currentPath = "";

        for (let i = 0; i < parts.length; i += 1) {
          const path = parts.slice(0, i + 1).join("/");
          currentPath = `/${path}`;

          if (!nodes.has(currentPath)) {
            nodes.set(currentPath, {
              groupKey: path.replace(/\//g, "-"),
              groupName: parts[i],
              services: [],
              count: 0,
              fullPrefix: i > 0 ? `/${parts.slice(0, i).join("/")}` : "",
              srvType: "",
              groupKeys: [],
              serviceInfo: null,
            });
          }
        }

        // Attach ServiceExtendedInfo to the leaf node
        const leafNode = nodes.get(currentPath);
        if (leafNode) {
          leafNode.serviceInfo = service;
          // Leaf node always represents exactly one service initially
          leafNode.count = 1;
          leafNode.srvType = service.srvType;
        }
      }

      /**
       * Phase 2: connect nodes to a tree based on their parent paths
       * - Determine parent by stripping the last "/segment" from the path
       * - Root nodes have no valid parent in the map and are pushed to rootNodes
       * - We DO NOT propagate counts here; that is done later recursively
       */
      for (const [path, node] of nodes.entries()) {
        const parentPath = path.substring(0, path.lastIndexOf("/"));

        if (parentPath && nodes.has(parentPath)) {
          const parent = nodes.get(parentPath);
          if (parent) {
            parent.services.push(node);
            parent.groupKeys.push(node.groupKey);

            // Propagate type information upwards if this group itself has no service
            if (!parent.serviceInfo) {
              parent.srvType = node.srvType || parent.srvType;
            }
          }
        } else {
          // No parent => this is a root node
          rootNodes.push(node);
        }
      }

      /**
       * Phase 3: sort root nodes
       * - Groups before leaf services
       * - Alphabetical by groupName
       */
      rootNodes.sort((a, b) => {
        const aIsGroup = a.services.length > 0;
        const bIsGroup = b.services.length > 0;
        if (aIsGroup && !bIsGroup) return -1;
        if (!aIsGroup && bIsGroup) return 1;
        return a.groupName.localeCompare(b.groupName);
      });

      /**
       * Phase 4: optionally flatten groups that contain only a single child
       * - This is only a structural change; counts will be recalculated afterwards
       */
      const processedRoots: TTreeItem[] = [];
      if (avoidSingle) {
        for (const node of rootNodes) {
          processedRoots.push(flattenSingleChildGroups(node));
        }
      } else {
        processedRoots.push(...rootNodes);
      }

      /**
       * Phase 5: recursively compute counts for all nodes
       * - Leaf node (serviceInfo != null) has count = 1
       * - Group node has count = sum of all leaf services in its subtree
       * - This guarantees that count includes all services in subgroups
       */
      const computeCounts = (node: TTreeItem): number => {
        // Leaf service
        if (node.serviceInfo) {
          node.count = 1;
          return 1;
        }

        let sum = 0;
        for (const child of node.services) {
          sum += computeCounts(child);
        }
        node.count = sum;
        return sum;
      };

      for (const root of processedRoots) {
        computeCounts(root);
      }

      return { services: processedRoots, count: processedRoots.length, groupKeys: [] };
    },
    [flattenSingleChildGroups]
  );

  /**
   * Text filter on service name, type and provider / requester node names.
   */
  const filteredServices = useMemo(() => {
    if (!searchTerm.trim()) return services;

    return services.filter((service) =>
      findIn(searchTerm, [
        service.name,
        service.srvType,
        ...service.nodeProviders.map((item) => item.nodeName),
        ...service.nodeRequester.map((item) => item.nodeName),
      ])
    );
  }, [services, searchTerm]);

  /**
   * Tree data for the current filter state (single structure, no domains).
   */
  const treeData = useMemo(() => {
    const treeResult = buildTree(
      filteredServices,
      searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS ? avoidGroupWithOneItem : false
    );
    return treeResult.services;
  }, [filteredServices, avoidGroupWithOneItem, searchTerm.length, buildTree]);

  // initial & event-driven updates
  useEffect(() => {
    updateServiceList();
  }, []);

  useEffect(() => {
    updateServiceList();
  }, [rosCtx.mapProviderRosNodes, updateServiceList]);

  useCustomEventListener(EVENT_PROVIDER_ROS_SERVICES, updateServiceList);
  useCustomEventListener(EVENT_FILTER_SERVICES, (filter: TFilterText) => setSearchTerm(filter.data));

  // keep derived root tree list in state (used by Virtuoso)
  useEffect(() => {
    setRootDataList(treeData);
  }, [treeData]);

  const onSearch = useCallback((term: string) => {
    setSearchTerm(term);
  }, []);

  // expand/collapse for groups
  const toggleExpanded = useCallback((id: string) => {
    setExpandedItems((prev) => (prev.includes(id) ? prev.filter((x) => x !== id) : [...prev, id]));
  }, []);

  // selection handler (no domainId anymore)
  const handleSelect = useCallback((itemId: string) => {
    setSelected(itemId);
  }, []);

  // resolve selected ServiceExtendedInfo from selected id
  useEffect(() => {
    if (!selected) {
      setServiceForSelected(undefined);
      return;
    }

    let found: ServiceExtendedInfo | undefined;

    for (const svc of services) {
      if (genKey([svc.name, svc.srvType]) === selected) {
        found = svc;
        break;
      }
    }

    setServiceForSelected(found);
  }, [selected, services, genKey]);

  /**
   * Flat rows for Virtuoso.
   * Groups and services are flattened into a single list while preserving depth.
   */
  const flatRows = useMemo<FlatRow[]>(() => {
    const expandedSet = new Set(expandedItems);
    const rows: FlatRow[] = [];

    const walk = (node: TTreeItem, depth: number, rootPath: string) => {
      if (node.serviceInfo) {
        rows.push({
          id: genKey([node.serviceInfo.name, node.serviceInfo.srvType]),
          type: "service",
          depth,
          treeItem: node,
          rootPath,
        });
        return;
      }

      if (avoidGroupWithOneItem && node.services.length === 1) {
        const nextRoot = rootPath ? `${rootPath}/${node.groupName}` : node.groupName;
        walk(node.services[0], depth, nextRoot);
        return;
      }

      rows.push({
        id: node.groupKey,
        type: "group",
        depth,
        treeItem: node,
        rootPath,
      });

      if (expandedSet.has(node.groupKey)) {
        const sortedChildren = [...node.services].sort((a, b) => {
          const aIsGroup = !a.serviceInfo;
          const bIsGroup = !b.serviceInfo;
          if (aIsGroup && !bIsGroup) return -1;
          if (!aIsGroup && bIsGroup) return 1;
          return a.groupName.localeCompare(b.groupName);
        });

        for (const child of sortedChildren) {
          walk(child, depth + 1, "");
        }
      }
    };

    const sortedRoots = [...rootDataList].sort((a, b) => {
      const aIsGroup = !a.serviceInfo;
      const bIsGroup = !b.serviceInfo;
      if (aIsGroup && !bIsGroup) return -1;
      if (!aIsGroup && bIsGroup) return 1;
      return a.groupName.localeCompare(b.groupName);
    });

    for (const root of sortedRoots) {
      walk(root, 0, "");
    }

    return rows;
  }, [rootDataList, expandedItems, avoidGroupWithOneItem, genKey]);

  /**
   * Open "call service" panel for the selected service.
   * external / openInTerminal are reserved for future extension (parity with topics).
   */
  const onCallService = useCallback(
    (service: ServiceExtendedInfo | undefined, external: boolean, openInTerminal = false) => {
      if (!service) return;

      // currently we do not distinguish external / terminal for services,
      // but we keep the parameters for future alignment with topic handling
      console.debug(`open service dialog for '${service.name}'  external=${external} terminal=${openInTerminal}`);
      navCtx.openServiceCaller({
        providerId: service.nodeProviders[0]?.providerId || "",
        serviceName: service.name,
        serviceType: service.srvType,
        externalKeyModifier: external,
        forceOpenTerminal: openInTerminal,
      });
    },
    [navCtx.openServiceCaller]
  );

  const onIntrospect = useCallback(
    (service: ServiceExtendedInfo | undefined, external: boolean, openInTerminal = false) => {
      if (!service) return;

      console.debug(`open dialog for introspect service: external=${external} terminal=${openInTerminal}`);
      navCtx.openServiceIntrospection({
        providerId: service.nodeProviders[0]?.providerId || "",
        serviceName: service.name,
        serviceType: service.srvType,
        externalKeyModifier: external,
        forceOpenTerminal: openInTerminal,
      });
    },
    [navCtx.openServiceIntrospection]
  );

  const buttonBox = useMemo(
    () => (
      <ButtonGroup orientation="vertical" aria-label="service control group">
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Call service
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight="bold" fontSize="inherit">
                  Shift or long press:
                </Typography>
                <Typography fontSize="inherit">alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <LongPressIconButton
              disabled={!serviceForSelected}
              size="medium"
              aria-label="call service"
              onClick={(event) =>
                onCallService(serviceForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey)
              }
              onLongPress={() => {
                onCallService(serviceForSelected, true, false);
              }}
            >
              <SyncAltOutlinedIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>
        <Tooltip
          title={
            <div>
              <Typography fontWeight="bold" fontSize="inherit">
                Introspect service
              </Typography>
              <Stack direction="row" spacing={"0.2em"}>
                <Typography fontWeight="bold" fontSize="inherit">
                  Shift or long press:
                </Typography>
                <Typography fontSize="inherit">alternative open location</Typography>
              </Stack>
            </div>
          }
          placement="left"
          disableInteractive
        >
          <span>
            <LongPressIconButton
              disabled={!serviceForSelected}
              size="medium"
              aria-label="introspect"
              onClick={(event) =>
                onIntrospect(serviceForSelected, event.nativeEvent.shiftKey, event.nativeEvent.ctrlKey)
              }
              onLongPress={() => {
                onIntrospect(serviceForSelected, true, false);
              }}
            >
              <TroubleshootIcon fontSize="inherit" />
            </LongPressIconButton>
          </span>
        </Tooltip>
      </ButtonGroup>
    ),
    [serviceForSelected, onCallService, onIntrospect]
  );

  const reloadButton = useMemo(
    () => (
      <Tooltip title="Reload service list" placement="left" disableInteractive>
        <IconButton size="small" onClick={getServiceList}>
          <RefreshIcon sx={{ fontSize: "inherit" }} />
        </IconButton>
      </Tooltip>
    ),
    [getServiceList]
  );

  /**
   * Tree view: single Virtuoso with flattened rows (no multi-domain mode anymore).
   */
  const treeView = useMemo(
    () => (
      <Virtuoso
        style={{ height: "100%" }}
        totalCount={flatRows.length}
        itemContent={(index: number) => {
          const row = flatRows[index];

          if (row.type === "group") {
            const node = row.treeItem;
            const isSelected = selected === row.id;

            return (
              <TopicGroupTreeItem
                key={row.id}
                itemId={row.id}
                rootPath={row.rootPath}
                groupName={node.groupName}
                countChildren={node.count}
                hasIncompatibleQos={false}
                depth={row.depth}
                expanded={expandedItems.includes(row.id)}
                selected={isSelected}
                onToggle={() => toggleExpanded(row.id)}
                onSelect={() => handleSelect(row.id)}
              />
            );
          }

          const serviceInfo = row.treeItem.serviceInfo;
          if (!serviceInfo) return null;

          const id = row.id;
          const isSelected = selected === id;

          return (
            <ServiceTreeItem
              key={id}
              itemId={id}
              rootPath={row.rootPath}
              serviceInfo={serviceInfo}
              selectedItem={selected ?? ""}
              selected={isSelected}
              depth={row.depth}
              onSelect={() => handleSelect(id)}
            />
          );
        }}
      />
    ),
    [flatRows, selected, expandedItems, toggleExpanded, handleSelect]
  );

  return (
    <Box height="100%" overflow="hidden" sx={{ backgroundColor: backgroundColor }}>
      <Stack spacing={1} height="100%">
        <Stack direction="row" spacing={0.5} alignItems="center">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && reloadButton}
          <SearchBar
            onSearch={onSearch}
            placeholder="Filter Services (OR: <space>, AND: +, NOT: !)"
            defaultValue={searchTerm}
            fullWidth
          />
          {buttonLocation === BUTTON_LOCATIONS.RIGHT && reloadButton}
        </Stack>

        <Stack direction="row" height="100%" overflow="hidden">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}

          {/* Click on empty area deselects the current item */}
          <Box
            width="100%"
            height="100%"
            overflow="hidden"
            onClick={() => {
              setSelected(null);
            }}
          >
            {treeView}
          </Box>

          {buttonLocation === BUTTON_LOCATIONS.RIGHT && (
            <Box height="100%" sx={{ boxShadow: `0px 0px 1px ${alpha(grey[600], 0.4)}` }}>
              {buttonBox}
            </Box>
          )}
        </Stack>
      </Stack>
    </Box>
  );
}
