import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ComputerIcon from "@mui/icons-material/Computer";
import DeleteIcon from "@mui/icons-material/Delete";
import HideSourceIcon from "@mui/icons-material/HideSource";
import Label from "@mui/icons-material/Label";
import { Box, IconButton, Stack, Tooltip } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
// import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import RefreshIcon from "@mui/icons-material/Refresh";
import { ParameterTreeItem } from "../../../components";
import SearchBar from "../../../components/UI/SearchBar";
import { DEFAULT_BUG_TEXT, LoggingContext } from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { findIn } from "../../../utils/index";

export default function ParameterPanel({ nodes = null, providers = null }) {
  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const [roots, setRoots] = useState(new Map("", {}));
  const [rootData, setRootData] = useState(new Map("", {}));
  const [rootDataFiltered, setRootDataFiltered] = useState(new Map("", []));
  const [rootDataTree, setRootDataTree] = useState(new Map("", []));
  const [expanded, setExpanded] = useState([]);
  const [expandedFiltered, setExpandedFiltered] = useState([]);
  const [nodeFilter, setNodeFilter] = useState(false);
  const [searched, setSearched] = useState("");
  const [selectedItems, setSelectedItems] = useState(null);

  // debounced search callback
  // search in the origin parameter list and create a new one
  const onSearch = useDebounceCallback((searchTerm) => {
    const filteredData = new Map("", {});
    if (!searchTerm) {
      rootData.forEach((paramList, rootName) => {
        filteredData.set(rootName, paramList);
      });
    } else {
      rootData.forEach((paramList, rootName) => {
        filteredData.set(
          rootName,
          paramList.filter((p) => {
            const isMatch = findIn(searchTerm, [p.name, JSON.stringify(p.value), p.type]);
            return isMatch;
          })
        );
      });
    }
    setRootDataFiltered(filteredData);
  }, 300);

  useEffect(() => {
    onSearch(searched);
  }, [searched]);

  const getParameterList = useCallback(async () => {
    if (!rosCtx.initialized) return;
    setRootData(new Map("", {}));
    setRootDataFiltered(new Map("", []));

    Array.from(roots).forEach(async ([rootId, rootObj]) => {
      if (Object.hasOwn(rootObj, "system_node")) {
        // dirty check to test if object is RosNode
        // TODO: optimize request if all nodes are from the same provider
        const provider = rosCtx.getProviderById(rootObj.providerId);
        if (!provider || !provider.isAvailable()) return;
        // check if provider supports [getNodeParameters]
        if (!provider.getNodeParameters) {
          logCtx.error(`Provider ${provider.name()} does not support [getNodeParameters] method`, DEFAULT_BUG_TEXT);
          return;
        }
        const paramList = await provider.getNodeParameters([rootObj.name]);
        // sort parameter by name
        paramList.sort(function (a, b) {
          return a.name.localeCompare(b.name);
        });
        rootData.set(rootId, paramList);
        setRootData(new Map(rootData));
        onSearch(searched);
      } else {
        // exclude not available provider
        if (rootObj && !rootObj.isAvailable()) return;

        // check if provider supports [getParameterList]
        if (!rootObj.getParameterList) {
          logCtx.error(`Provider ${rootObj.name} does not support [getParameterList] method`, DEFAULT_BUG_TEXT);
          return;
        }
        const paramList = await rootObj.getParameterList();
        // sort parameter by name
        paramList.sort(function (a, b) {
          return a.name.localeCompare(b.name);
        });
        rootData.set(rootId, paramList);
        setRootData(new Map(rootData));
        onSearch(searched);
      }
    });

    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.initialized, rosCtx.providers, nodes, roots, rootData, searched]);

  // debounced callback when updating a parameter
  const updateParameter = useDebounceCallback(async (parameter, newValue, newType) => {
    const provider = rosCtx.getProviderById(parameter.providerId);
    if (!provider || !provider.isAvailable()) return;

    if (!provider.setParameter) {
      logCtx.error(
        `Provider ${rosCtx.getProviderName(parameter.providerId)} does not support [setParameter] method`,
        DEFAULT_BUG_TEXT
      );
      return;
    }

    parameter.value = newValue;
    if (newType) {
      parameter.type = newType;
    }
    const result = await provider.setParameter(parameter);

    if (result) {
      logCtx.success("Parameter updated successfully", `Parameter: ${parameter.name}, value: ${parameter.value}`);
    } else {
      logCtx.error(`Could not update parameter [${parameter.name}]`, DEFAULT_BUG_TEXT);
    }
  }, 300);

  const deleteParameters = useCallback(
    (paramsMap) => {
      Array.from(paramsMap).map(async ([providerId, params]) => {
        const provider = rosCtx.getProviderById(providerId);
        if (!provider || !provider.isAvailable()) return;

        if (!provider.deleteParameters) {
          logCtx.error(
            `Provider ${rosCtx.getProviderName(providerId)} does not support [deleteParameters] method`,
            DEFAULT_BUG_TEXT
          );
          return;
        }

        const result = await provider.deleteParameters(params);

        if (result) {
          logCtx.success(`Parameter deleted successfully from ${provider.name()}`, `${JSON.stringify(params)}`);
        } else {
          logCtx.error(`Could not delete parameters from ${provider.name()}`, DEFAULT_BUG_TEXT);
        }
        // TODO: update only involved provider / nodes
        getParameterList();
      });
    },
    [getParameterList, logCtx, rosCtx]
  );

  // callback when deleting parameters
  const onDeleteParameters = useCallback(async () => {
    if (!rootDataFiltered || rootDataFiltered.size === 0) {
      logCtx.warn("No parameters to be deleted", "");
      return;
    }

    // Sort parameter by provider
    const paramsToBeDeleted = new Map("", []);

    rootDataFiltered.forEach(([p, key]) => {
      if (!paramsToBeDeleted.has(p.providerId)) {
        paramsToBeDeleted.set(p.providerId, []);
      }
      paramsToBeDeleted.get(p.providerId).push(p.name);
    });
    deleteParameters(paramsToBeDeleted);

    // Do we need to get the whole list of parameters again?
    // getParameterList();
  }, [rootDataFiltered, deleteParameters, logCtx]);

  useEffect(() => {
    if (!rosCtx.initialized) return;
    const newRoots = new Map();
    if (nodes?.length > 0) {
      nodes.forEach((node) => {
        newRoots.set(node.name, node);
      });
      setNodeFilter(true);
    } else if (providers?.length > 0) {
      providers.forEach((providerId) => {
        newRoots.set(providerId, rosCtx.getProviderById(providerId));
      });
    } else {
      rosCtx.providers.forEach((provider) => {
        if (provider.isAvailable()) {
          if (provider.rosVersion === "1") {
            newRoots.set(provider.id, provider);
          } else {
            provider.rosNodes.map((item) => newRoots.set(item.name, item));
          }
        }
      });
    }
    setRoots(newRoots);
    setExpanded([...newRoots.keys()]);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.initialized, rosCtx.providers]);

  useEffect(() => {
    getParameterList();
  }, [roots]);

  // create tree based on parameter namespace
  // parameters are grouped only if more then one is in the group
  const fillTree = (name, fullPrefix, params, itemId) => {
    if (!params) return { params: [], count: 0, groupKeys: [] };
    const groupKeys = [];
    const byPrefixP1 = new Map("", []);
    // count parameter for each group
    params.forEach((param) => {
      const nameSuffix = param.name.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName).push({ restNameSuffix, param });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, param }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ param }]);
      }
    });

    // create result
    let count = 0;
    const filteredParams = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      if (value.length > 1) {
        const groupKey = `${itemId}-${groupName}`;
        const newFullPrefix = `${fullPrefix}/${groupName}`;
        groupKeys.push(groupKey);
        const groupParams = value.map((item) => {
          return item.param;
        });
        const subResult = fillTree(groupName, newFullPrefix, groupParams, groupKey);
        // the result count is 0 -> we added multiple provider for topic with same name.
        if (subResult.count === 0) {
          count += 1;
        } else {
          count += subResult.count;
        }
        if (subResult.groupKeys.length > 0) {
          groupKeys.push(...subResult.groupKeys);
        }
        filteredParams.push([
          {
            groupKey: groupKey,
            groupName: `/${groupName}`,
            params: subResult.params,
            count: subResult.count,
            fullPrefix: newFullPrefix,
            groupKeys: groupKeys,
          },
        ]);
      } else {
        filteredParams.push(value[0].param);
        count += 1;
      }
    });
    return { params: filteredParams, count, groupKeys };
  };

  // create parameter tree from filtered parameter list
  useEffect(() => {
    const tree = new Map("", []);
    const rootKeys = [];
    Array.from(roots).map(([rootName, rootObj]) => {
      const subtree = fillTree(
        rootName,
        nodeFilter ? `${rootName}` : "",
        rootDataFiltered.get(rootName),
        nodeFilter ? `${rootName}` : rootObj.id
      );
      tree.set(rootName, subtree.params);
      rootKeys.push(rootName);
      rootKeys.push(...subtree.groupKeys);
      return null;
    });
    setRootDataTree(tree);
    // if (searched.length < EXPAND_ON_SEARCH_MIN_CHARS) {
    setExpandedFiltered(rootKeys);
    // }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [nodeFilter, rootDataFiltered, roots]);

  const getIcon = (obj) => {
    if (Object.hasOwn(obj, "isLocalHost")) {
      // it is Provider
      return ComputerIcon;
      // return PrecisionManufacturingIcon;
    }
    if (Object.hasOwn(obj, "system_node")) {
      // it is RosNode
      return Label;
    }
    return HideSourceIcon;
  };

  const paramTreeToStyledItems = (rootPath, params) => {
    return params?.map((param) => {
      if (param.providerId) {
        return (
          <ParameterTreeItem
            key={param.id}
            itemId={param.id}
            labelRoot={rootPath}
            labelText={`${param.name}`}
            labelInfo={param.type}
            color="#1a73e8"
            bgColor="#e8f0fe"
            colorForDarkMode="#B8E7FB"
            bgColorForDarkMode="#071318"
            param={param}
            updateParameter={updateParameter}
          />
        );
      }
      return param.map((group) => {
        return (
          <ParameterTreeItem
            key={group.groupKey}
            itemId={group.groupKey}
            labelRoot={rootPath}
            labelText={group.groupName}
            labelCount={group.count}
          >
            {paramTreeToStyledItems(group.fullPrefix, group.params)}
          </ParameterTreeItem>
        );
      });
    });
  };

  const deleteParamsFromIds = (treeItemId) => {
    // Sort parameter by provider
    const paramsToBeDeleted = new Map("", []);
    rootData.forEach((paramList, rootName) => {
      paramList.forEach((p) => {
        let addParam = false;
        if (rootName === treeItemId) {
          addParam = true;
        } else if (p.id === treeItemId) {
          addParam = true;
        } else if (treeItemId.endsWith("#")) {
          const trimmed = treeItemId.slice(0, treeItemId.length - 2);
          addParam = p.name.startsWith(trimmed);
        }
        if (addParam) {
          if (!paramsToBeDeleted.has(p.providerId)) {
            paramsToBeDeleted.set(p.providerId, []);
          }
          paramsToBeDeleted.get(p.providerId).push(p.name);
        }
      });
    });
    deleteParameters(paramsToBeDeleted);
  };

  const createParameterItems = useMemo(() => {
    return (
      <>
        {Array.from(roots).map(([rootId, rootObj]) => {
          const filteredItems = rootDataFiltered.get(rootId);
          return (
            <ParameterTreeItem
              key={`${rootId}`}
              itemId={`${rootId}`}
              labelText={`${Object.hasOwn(rootObj, "system_node") ? rootId : rosCtx.getProviderName(rootId)}`}
              labelIcon={getIcon(rootObj)}
              labelCount={filteredItems ? filteredItems.length : null}
              requestData={!rootData.has(rootId)}
              providerName={rosCtx.getProviderName(rootId)}
            >
              {paramTreeToStyledItems(nodeFilter ? `${rootId}` : "", rootDataTree.get(rootId))}
            </ParameterTreeItem>
          );
        })}
      </>
    );
  }, [roots, rootDataTree, nodeFilter, rootDataFiltered]);

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
          <Tooltip title="Delete selected parameter" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
            <span>
              <IconButton
                disabled={!selectedItems}
                size="small"
                aria-label="Delete selected parameter"
                onClick={() => {
                  deleteParamsFromIds(selectedItems);
                }}
              >
                <DeleteIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
          <Tooltip title="Reload parameter list" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
            <IconButton
              size="small"
              onClick={() => {
                getParameterList();
              }}
            >
              <RefreshIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>
          <SearchBar
            onSearch={setSearched}
            placeholder="Filter parameters (OR: <space>, AND: +, NOT: !)"
            // defaultValue={initialSearchTerm}
            fullWidth
          />
          {searched && (
            <Tooltip title="Delete filtered parameters" placement="bottom" disableInteractive>
              <IconButton size="small" onClick={onDeleteParameters} color="warning">
                <DeleteIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )}
        </Stack>
        <Stack direction="row" height="100%" overflow="auto">
          <Box width="100%" height="100%" overflow="auto">
            <SimpleTreeView
              aria-label="parameters"
              expandedItems={searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
              slots={{
                collapseIcon: ArrowDropDownIcon,
                expandIcon: ArrowRightIcon,
              }}
              // defaultEndIcon={<div style={{ width: 24 }} />}
              onSelectedItemsChange={(event, itemId) => {
                setSelectedItems(itemId);
                const index =
                  searched.length < EXPAND_ON_SEARCH_MIN_CHARS
                    ? expanded.indexOf(itemId)
                    : expandedFiltered.indexOf(itemId);
                const copyExpanded = [...(searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
                if (index === -1) {
                  copyExpanded.push(itemId);
                } else {
                  copyExpanded.splice(index, 1);
                }
                if (searched.length < EXPAND_ON_SEARCH_MIN_CHARS) {
                  setExpanded(copyExpanded);
                } else {
                  setExpandedFiltered(copyExpanded);
                }
              }}
              // workaround for https://github.com/mui/mui-x/issues/12622
              onItemFocus={(e, itemId) => {
                const input = document.getElementById(`input-${itemId}`);
                if (input) {
                  input.focus();
                }
              }}
            >
              {createParameterItems}
            </SimpleTreeView>
          </Box>
        </Stack>
      </Stack>
    </Box>
  );
}

ParameterPanel.propTypes = {
  nodes: PropTypes.arrayOf(PropTypes.any),
  providers: PropTypes.arrayOf(PropTypes.string),
};
