import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ComputerIcon from "@mui/icons-material/Computer";
import DeleteIcon from "@mui/icons-material/Delete";
import Label from "@mui/icons-material/Label";
import { Box, IconButton, Stack, Tooltip } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import RefreshIcon from "@mui/icons-material/Refresh";
import { RosNode, RosParameter } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import RosContext from "@/renderer/context/RosContext";
import SettingsContext from "@/renderer/context/SettingsContext";
import LoggingContext, { DEFAULT_BUG_TEXT } from "@/renderer/context/LoggingContext";
import { findIn } from "@/renderer/utils";
import ParameterTreeItem from "./ParameterTreeItem";
import ParameterGroupTreeItem from "./ParameterGroupTreeItem";
import { SearchBar } from "../UI";

type TTreeItem = {
  groupKey: string;
  groupName: string;
  params: TTreeItem[];
  count: number;
  fullPrefix: string;
  groupKeys: string[];
  paramInfo: RosParameter | null;
};

interface ParameterRootTreeProps {
  provider: Provider;
  rosNode?: RosNode;
  updateOnCreate?: boolean;
  filterText: string;
}

const ParameterRootTree = forwardRef<HTMLDivElement, ParameterRootTreeProps>(function ParameterRootTree(props, ref) {
  const { provider, rosNode = undefined, updateOnCreate = false, filterText = "" } = props;

  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const [itemId] = useState<string>(rosNode ? rosNode.idGlobal : provider.id);
  const [rosParameters, setRosParameters] = useState<RosParameter[]>();
  const [rosParametersFiltered, setRosParametersFiltered] = useState<RosParameter[]>();
  const [tree, setTree] = useState<TTreeItem[]>();
  const [expanded, setExpanded] = useState<string[]>([itemId]);
  const [expandedFiltered, setExpandedFiltered] = useState<string[]>([itemId]);
  const [searched, setSearched] = useState<string>(filterText);
  const [selectedItem, setSelectedItem] = useState<string>("");

  // const [roots, setRoots] = useState<TParamRootItem[]>([]);
  // const [rootData, setRootData] = useState<TRootData[]>([]);
  // const [receivedData, setReceivedData] = useState<TRootData>();
  // const [rootDataFiltered, setRootDataFiltered] = useState<TRootData[]>([]);
  // const [rootDataTree, setRootDataTree] = useState<TRootTree[]>([]);
  // const [nodeFilter, setNodeFilter] = useState<boolean>(false);
  // const [searched, setSearched] = useState<string>("");
  // const [selectedItem, setSelectedItem] = useState<string>("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  useEffect(() => {
    setSearched(filterText);
  }, [filterText]);

  function filterParameters(searchTerm: string, parameters: RosParameter[] | undefined) {
    if (searchTerm.length < EXPAND_ON_SEARCH_MIN_CHARS) {
      return parameters;
    }
    return parameters?.filter((p) => {
      const isMatch = findIn(searched, [p.name, JSON.stringify(p.value), p.type]);
      return isMatch;
    });
  }

  // debounced search callback
  // search in the origin parameter list and create a new one
  const onSearch = useDebounceCallback((searchTerm: string) => {
    setRosParametersFiltered(filterParameters(searchTerm, rosParameters));
  }, 300);

  useEffect(() => {
    onSearch(searched);
  }, [searched]);

  const getParameterList = useCallback(async () => {
    if (!provider.isAvailable()) return;
    // check if provider supports [getNodeParameters]
    if (!provider.getNodeParameters) {
      logCtx.error(`Provider ${provider.name()} does not support [getNodeParameters] method`, DEFAULT_BUG_TEXT);
      return;
    }
    setRosParameters(undefined);
    console.log(`GET rootId: ${rosNode ? rosNode.idGlobal : provider.id}`);
    const paramList: RosParameter[] = await (rosNode
      ? provider.getNodeParameters([rosNode.name])
      : provider.getParameterList());
    // sort parameter by name
    paramList.sort(function (a, b) {
      return a.name.localeCompare(b.name);
    });
    paramList.map((p) => {
      console.log(`pp: ${JSON.stringify(p)}`);
      return p;
    });
    // setRootData((prev) => [...prev, { rootId: root.id, params: paramList }]);
    // newData.push({ rootId: root.id, params: paramList });
    setRosParameters(paramList);
    console.log(`GET done rootId: ${rosNode ? rosNode.idGlobal : provider.id}`);
    setRosParametersFiltered(filterParameters(searched, paramList));
  }, [provider, rosNode, searched, setRosParameters, setRosParametersFiltered, filterParameters]);

  useEffect(() => {
    // update the parameter on create this component
    if (updateOnCreate) {
      getParameterList();
    }
  }, []);

  // debounced callback when updating a parameter
  const updateParameter = useCallback(
    async (parameter: RosParameter, newValue: string | boolean | number | string[], newType?: string) => {
      if (!provider.isAvailable()) return;

      if (!provider.setParameter) {
        logCtx.error(
          `Provider ${rosCtx.getProviderName(parameter.providerId)} does not support [setParameter] method`,
          DEFAULT_BUG_TEXT
        );
        return;
      }
      console.log(`update parameter: ${JSON.stringify(parameter)}`);
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
    },
    [provider]
  );

  const deleteParameters = useCallback(
    async (params: RosParameter[]) => {
      if (params.length > 0) {
        if (!provider.deleteParameters) {
          logCtx.error(`Provider ${provider.name()} does not support [deleteParameters] method`, DEFAULT_BUG_TEXT);
          return;
        }

        const result = await provider.deleteParameters(params.map((p) => p.name));

        if (result) {
          logCtx.success(`Parameter deleted successfully from ${provider.name()}`, `${JSON.stringify(params)}`);
        } else {
          logCtx.error(`Could not delete parameters from ${provider.name()}`, DEFAULT_BUG_TEXT);
        }
      }
      // TODO: update only involved provider / nodes
      getParameterList();
    },
    [getParameterList, logCtx]
  );

  // callback when deleting parameters
  const onDeleteParameters = useCallback(async () => {
    if (!rosParametersFiltered || rosParametersFiltered.length === 0) {
      logCtx.warn("No parameters to be deleted", "");
      return;
    }

    deleteParameters(rosParametersFiltered);

    // Do we need to get the whole list of parameters again?
    // getParameterList();
  }, [rosParametersFiltered, deleteParameters, logCtx]);

  // create tree based on parameter namespace
  // parameters are grouped only if more then one is in the group
  const fillTree = (fullPrefix: string, params: RosParameter[], itemId: string) => {
    if (!params) return { params: [], count: 0, groupKeys: [] };
    const byPrefixP1: Map<string, { restNameSuffix: string; paramInfo: RosParameter }[]> = new Map();
    // count parameter for each group
    params.forEach((param) => {
      const nameSuffix = param.name.slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName)?.push({ restNameSuffix, paramInfo: param });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, paramInfo: param }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ restNameSuffix: "", paramInfo: param }]);
      }
    });

    // create result
    let count = 0;
    const groupKeys: string[] = [];
    const filteredParams: TTreeItem[] = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const newFullPrefix = `${fullPrefix}/${groupName}`;
      if (value.length > 1) {
        const groupKey = `${itemId}-${groupName}`;
        groupKeys.push(groupKey);
        const groupParams: RosParameter[] = value.map((item) => {
          return item.paramInfo;
        });
        const subResult = fillTree(newFullPrefix, groupParams, groupKey);
        // the result count is 0 -> we added multiple provider for topic with same name.
        if (subResult.count === 0) {
          count += 1;
        } else {
          count += subResult.count;
        }
        if (subResult.groupKeys.length > 0) {
          groupKeys.push(...subResult.groupKeys);
        }
        filteredParams.push({
          groupKey: groupKey,
          groupName: `/${groupName}`,
          params: subResult.params,
          count: subResult.count,
          fullPrefix: newFullPrefix,
          groupKeys: groupKeys,
          paramInfo: null,
        } as TTreeItem);
      } else {
        filteredParams.push({
          groupKey: "",
          groupName: "",
          params: [],
          count: 0,
          fullPrefix: newFullPrefix,
          groupKeys: groupKeys,
          paramInfo: value[0].paramInfo,
        } as TTreeItem);
        count += 1;
      }
    });
    return { params: filteredParams, count, groupKeys };
  };

  // create parameter tree from filtered parameter list
  useEffect(() => {
    const subtree = fillTree(rosNode ? rosNode.name : "", rosParametersFiltered || [], itemId);
    setTree(subtree.params);
    setExpandedFiltered([itemId, ...subtree.groupKeys]);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosParametersFiltered]);

  const paramTreeToStyledItems = (rootPath: string, treeItems: TTreeItem[]) => {
    return treeItems.map((param) => {
      if (param.paramInfo) {
        console.log(`param.paramInfo.id: ${param.paramInfo.id}`);
        return (
          <ParameterTreeItem
            key={param.paramInfo.id}
            itemId={param.paramInfo.id}
            rootPath={rootPath}
            paramInfo={param.paramInfo}
            updateParameter={(param: RosParameter, value: string | boolean | number | string[], valueType?: string) =>
              updateParameter(param, value, valueType)
            }
            rosVersion={provider.rosVersion}
          />
        );
      } else {
        console.log(`param.groupKey: ${param.groupKey}`);
        return (
          <ParameterGroupTreeItem
            key={param.groupKey}
            itemId={param.groupKey}
            rootPath={rootPath}
            groupName={param.groupName}
            icon={null}
            countChildren={param.count}
            requestData={false}
          >
            {paramTreeToStyledItems(param.fullPrefix, param.params)}
          </ParameterGroupTreeItem>
        );
      }
    });
  };

  const deleteParamsFromId = (treeItemId: string) => {
    // Sort parameter by provider
    deleteParameters(
      rosParameters?.filter((p) => {
        let addParam = false;
        if (itemId === treeItemId) {
          addParam = true;
        } else if (p.id === treeItemId) {
          addParam = true;
        } else if (treeItemId.endsWith("#")) {
          const trimmed = treeItemId.slice(0, treeItemId.length - 2);
          addParam = p.name.startsWith(trimmed);
        }
        return addParam;
      }) || []
    );
  };

  const handleToggle = useCallback(
    (itemIds: string[]) => {
      if (searched.length < EXPAND_ON_SEARCH_MIN_CHARS) {
        setExpanded(itemIds);
      } else {
        setExpandedFiltered(itemIds);
      }
    },
    [searched]
  );

  const createParameterItems = useMemo(() => {
    console.log(`CREATE PARAM TREE: ${tree?.length}`);
    return (
      <ParameterGroupTreeItem
        key={itemId}
        itemId={itemId}
        rootPath={rosNode ? rosNode.name : ""}
        groupName={rosNode ? rosNode.name : provider.name()}
        icon={rosNode ? Label : ComputerIcon}
        providerName={provider.name()}
        countChildren={rosParameters ? rosParameters.length : 0}
        requestData={rosParameters === undefined}
      >
        {paramTreeToStyledItems(rosNode ? rosNode.name : "", tree || [])}
      </ParameterGroupTreeItem>
    );
  }, [tree]);

  return (
    <Box>
      <Stack
        spacing={1}
        // sx={{
        //   height: '100%',
        //   display: 'flex',
        // }}
      >
        {/* <Stack direction="row" spacing={0.5} alignItems="center">
          <Tooltip title="Delete selected parameter" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
            <span>
              <IconButton
                disabled={!selectedItem}
                size="small"
                aria-label="Delete selected parameter"
                onClick={() => {
                  if (selectedItem) {
                    deleteParamsFromId(selectedItem);
                  }
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
        </Stack> */}
        <Stack direction="row" sx={{ flexGrow: 1 }}>
          {/* <Box width="100%" height="100%" overflow="auto"> */}
          {(searched.length < EXPAND_ON_SEARCH_MIN_CHARS || (tree && tree.length > 0)) && (
            <SimpleTreeView
              aria-label="parameters"
              expandedItems={searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
              slots={{
                collapseIcon: ArrowDropDownIcon,
                expandIcon: ArrowRightIcon,
              }}
              sx={{ flexGrow: 1 }}
              // defaultEndIcon={<div style={{ width: 24 }} />}
              onExpandedItemsChange={(_event, itemIds: string[]) => handleToggle(itemIds)}
              onSelectedItemsChange={(_event, itemId) => {
                setSelectedItem(itemId || "");
                const copyExpanded = [...(searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered)];
                if (itemId) {
                  const index =
                    searched.length < EXPAND_ON_SEARCH_MIN_CHARS
                      ? expanded.indexOf(itemId)
                      : expandedFiltered.indexOf(itemId);
                  if (index === -1) {
                    copyExpanded.push(itemId);
                  } else {
                    copyExpanded.splice(index, 1);
                  }
                }
                if (searched.length < EXPAND_ON_SEARCH_MIN_CHARS) {
                  setExpanded(copyExpanded);
                } else {
                  setExpandedFiltered(copyExpanded);
                }
              }}
              // workaround for https://github.com/mui/mui-x/issues/12622
              onItemFocus={(_event, itemId) => {
                const input = document.getElementById(`input-${itemId}`);
                if (input) {
                  input.focus();
                }
              }}
            >
              {createParameterItems}
            </SimpleTreeView>
          )}
          {/* </Box> */}
        </Stack>
      </Stack>
    </Box>
  );
});

export default ParameterRootTree;
