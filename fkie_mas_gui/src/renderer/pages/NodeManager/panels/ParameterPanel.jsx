import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";

import { Box, IconButton, Stack, Tooltip } from "@mui/material";

import { SimpleTreeView } from "@mui/x-tree-view";

import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ComputerIcon from "@mui/icons-material/Computer";
import DeleteIcon from "@mui/icons-material/Delete";
import HideSourceIcon from "@mui/icons-material/HideSource";
import Label from "@mui/icons-material/Label";
// import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import RefreshIcon from "@mui/icons-material/Refresh";

import { ParameterTreeItem } from "../../../components";
import SearchBar from "../../../components/UI/SearchBar";
import {
  DEFAULT_BUG_TEXT,
  LoggingContext,
} from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { findIn } from "../../../utils/index";

export default function ParameterPanel({ nodes = null, providers = null }) {
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const [roots, setRoots] = useState(new Map("", {}));
  const [rootData, setRootData] = useState(new Map("", {}));
  const [rootDataFiltered, setRootDataFiltered] = useState(new Map("", []));
  const [rootDataTree, setRootDataTree] = useState(new Map("", []));
  const [expanded, setExpanded] = useState([]);
  const [nodeFilter, setNodeFilter] = useState(false);
  const [searched, setSearched] = useState("");
  const [selectedItems, setSelectedItems] = useState(null);

  // debounced search callback
  // search in the origin parameter list and create a new one
  const onSearch = useDebounceCallback((searchTerm) => {
    setSearched(searchTerm);
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
            const isMatch = findIn(searchTerm, [
              p.name,
              JSON.stringify(p.value),
              p.type,
            ]);
            return isMatch;
          }),
        );
      });
    }
    setRootDataFiltered(filteredData);
  }, 300);

  const getParameterList = useCallback(async () => {
    if (!rosCtx.initialized) return;
    setRootData(new Map("", {}));
    setRootDataFiltered(new Map("", []));

    const parameterMapLocal = new Map("", "");

    Array.from(roots).forEach(async ([rootId, rootObj]) => {
      if (Object.hasOwn(rootObj, "system_node")) {
        // dirty check to test if object is RosNode
        // TODO: optimize request if all nodes are from the same provider
        const provider = rosCtx.getProviderById(rootObj.providerId);
        if (!provider || !provider.isAvailable()) return;
        // check if provider supports [getNodeParameters]
        if (!provider.getNodeParameters) {
          logCtx.error(
            `Provider ${provider.name()} does not support [getNodeParameters] method`,
            DEFAULT_BUG_TEXT,
          );
          return;
        }
        const paramList = await provider.getNodeParameters([rootObj.name]);
        paramList.forEach((p) => {
          parameterMapLocal.set(p.name, p);
        });
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
          logCtx.error(
            `Provider ${rootObj.name} does not support [getParameterList] method`,
            DEFAULT_BUG_TEXT,
          );
          return;
        }
        const paramList = await rootObj.getParameterList();
        paramList.forEach((p) => parameterMapLocal.set(p.name, p));
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
  }, [rosCtx.initialized, rosCtx.providers, nodes, roots]);

  // debounced callback when updating a parameter
  const updateParameter = useDebounceCallback(
    async (parameter, newValue, newType) => {
      const provider = rosCtx.getProviderById(parameter.providerId);
      if (!provider || !provider.isAvailable()) return;

      if (!provider.setParameter) {
        logCtx.error(
          `Provider ${rosCtx.getProviderName(
            parameter.providerId,
          )} does not support [setParameter] method`,
          DEFAULT_BUG_TEXT,
        );
        return;
      }

      parameter.value = newValue;
      if (newType) {
        parameter.type = newType;
      }
      const result = await provider.setParameter(parameter);

      if (result) {
        logCtx.success(
          "Parameter updated successfully",
          `Parameter: ${parameter.name}, value: ${parameter.value}`,
        );
      } else {
        logCtx.error(
          `Could not update parameter [${parameter.name}]`,
          DEFAULT_BUG_TEXT,
        );
      }
    },
    300,
  );

  const deleteParameters = useCallback(
    (paramsMap) => {
      Array.from(paramsMap).map(async ([providerId, params]) => {
        const provider = rosCtx.getProviderById(providerId);
        if (!provider || !provider.isAvailable()) return;

        if (!provider.deleteParameters) {
          logCtx.error(
            `Provider ${rosCtx.getProviderName(
              providerId,
            )} does not support [deleteParameters] method`,
            DEFAULT_BUG_TEXT,
          );
          return;
        }

        const result = await provider.deleteParameters(params);

        if (result) {
          logCtx.success(
            `Parameter deleted successfully from ${provider.name()}`,
            `${JSON.stringify(params)}`,
          );
        } else {
          logCtx.error(
            `Could not delete parameters from ${provider.name()}`,
            DEFAULT_BUG_TEXT,
          );
        }
        // TODO: update only involved provider / nodes
        getParameterList();
      });
    },
    [getParameterList, logCtx, rosCtx],
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
          newRoots.set(provider.id, provider);
        }
      });
    }
    setRoots(newRoots);
    setExpanded([...newRoots.keys()]);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.initialized, rosCtx.providers]);

  useEffect(() => {
    getParameterList();
  }, [getParameterList, roots]);

  // get group name from id of group tree item
  const fromGroupId = (id) => {
    if (id.endsWith("#")) {
      const trimmed = id.slice(0, -1);
      return {
        groupName: trimmed.substr(
          trimmed.lastIndexOf("/") + 1,
          trimmed.length - 1,
        ),
        fullPrefix: trimmed.substr(0, id.lastIndexOf("/")),
      };
    }
    return { groupName: id, fullPrefix: id };
  };

  // create id for group tree item
  const toGroupId = (groupName, fullPrefix) => {
    return `${fullPrefix}/${groupName}#`;
  };

  // create tree based on parameter namespace
  // parameters are grouped only if more then one is in the group
  const fillTree = (name, fullPrefix, params) => {
    if (!params) return [];
    const byPrefixP1 = new Map("", []);
    // count parameter for each group
    params.forEach((param) => {
      const nameSuffix = param.name.slice(fullPrefix.length + 1);
      const [firstName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const groupName = firstName;
        const restNameSuffix = restName.join("/");
        const groupId = toGroupId(groupName, fullPrefix);
        if (byPrefixP1.has(groupId)) {
          byPrefixP1.get(groupId).push({ restNameSuffix, param });
        } else {
          byPrefixP1.set(groupId, [{ restNameSuffix, param }]);
        }
      } else {
        byPrefixP1.set(firstName, [{ param }]);
      }
    });

    // create result
    const filteredParams = [];
    byPrefixP1.forEach((value, key) => {
      // don't create group with one parameter
      if (value.length > 1) {
        const { groupName } = fromGroupId(key);
        const groupParams = value.map((item) => {
          return item.param;
        });
        filteredParams.push([
          {
            groupKey: key,
            params: fillTree(key, `${fullPrefix}/${groupName}`, groupParams),
            count: value.length,
            fullPrefix: fullPrefix ? fullPrefix : name,
          },
        ]);
      } else {
        filteredParams.push(value[0].param);
      }
    });
    return filteredParams;
  };

  // create parameter tree from filtered parameter list
  useEffect(() => {
    const tree = new Map("", []);
    Array.from(roots).map(([rootName, rootObj]) => {
      const subtree = fillTree(
        rootName,
        nodeFilter ? `${rootName}` : "",
        rootDataFiltered.get(rootName),
      );
      tree.set(rootName, subtree);
      return null;
    });
    setRootDataTree(tree);
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
        const { groupName, fullPrefix } = fromGroupId(group.groupKey);
        const newRootName = `${fullPrefix}/${groupName}`;
        return (
          <ParameterTreeItem
            key={`${group.fullPrefix}-${newRootName}`}
            itemId={`${group.fullPrefix}-${newRootName}`}
            labelRoot={rootPath}
            labelText={`${groupName}`}
            labelCount={group.count}
          >
            {paramTreeToStyledItems(newRootName, group.params)}
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
              labelText={`${
                Object.hasOwn(rootObj, "system_node")
                  ? rootId
                  : rosCtx.getProviderName(rootId)
              }`}
              labelIcon={getIcon(rootObj)}
              labelCount={filteredItems ? filteredItems.length : null}
              requestData={!rootData.has(rootId)}
              providerName={rosCtx.getProviderName(rootId)}
            >
              {paramTreeToStyledItems(
                nodeFilter ? `${rootId}` : "",
                rootDataTree.get(rootId),
              )}
            </ParameterTreeItem>
          );
        })}
      </>
    );
  }, [roots, rootDataTree, nodeFilter, rootDataFiltered]);

  return (
    <Box
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get("backgroundColor")}
    >
      <Stack
        spacing={1}
        height="100%"
        // sx={{
        //   height: '100%',
        //   display: 'flex',
        // }}
      >
        <Stack direction="row" spacing={1} alignItems="center">
          <Tooltip
            title="Delete selected parameter"
            placement="bottom"
            enterDelay={tooltipDelay}
            enterNextDelay={tooltipDelay}
          >
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
          <SearchBar
            onSearch={onSearch}
            placeholder="Filter parameters (<space> for OR, + for AND)"
            // defaultValue={initialSearchTerm}
            fullWidth
          />
          <Tooltip title="Reload parameter list" placement="bottom">
            <IconButton size="small" onClick={getParameterList}>
              <RefreshIcon sx={{ fontSize: "inherit" }} />
            </IconButton>
          </Tooltip>
          {searched && (
            <Tooltip title="Delete filtered parameters" placement="bottom">
              <IconButton
                size="small"
                onClick={onDeleteParameters}
                color="warning"
              >
                <DeleteIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )}
        </Stack>
        <Stack direction="row" height="100%" overflow="auto">
          <Box width="100%" height="100%" overflow="auto">
            <SimpleTreeView
              aria-label="parameters"
              expandedItems={expanded}
              slots={{
                collapseIcon: ArrowDropDownIcon,
                expandIcon: ArrowRightIcon,
              }}
              // defaultEndIcon={<div style={{ width: 24 }} />}
              onSelectedItemsChange={(event, itemId) => {
                setSelectedItems(itemId);
                const index = expanded.indexOf(itemId);
                const copyExpanded = [...expanded];
                if (index === -1) {
                  copyExpanded.push(itemId);
                } else {
                  copyExpanded.splice(index, 1);
                }
                setExpanded(copyExpanded);
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
