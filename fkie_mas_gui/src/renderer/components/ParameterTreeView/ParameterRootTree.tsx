import LoggingContext, { DEFAULT_BUG_TEXT } from "@/renderer/context/LoggingContext";
import RosContext from "@/renderer/context/RosContext";
import SettingsContext from "@/renderer/context/SettingsContext";
import { RosNode, RosParameter } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { findIn } from "@/renderer/utils";
import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ComputerIcon from "@mui/icons-material/Computer";
import Label from "@mui/icons-material/Label";
import { Box, Stack } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import ParameterGroupTreeItem from "./ParameterGroupTreeItem";
import ParameterTreeItem from "./ParameterTreeItem";

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
  forceReload: number;
  onSelectParams: (provider: Provider, params: RosParameter[]) => void;
}

const ParameterRootTree = forwardRef<HTMLDivElement, ParameterRootTreeProps>(function ParameterRootTree(props, ref) {
  const {
    provider,
    rosNode = undefined,
    updateOnCreate = false,
    filterText = "",
    forceReload,
    onSelectParams = (): void => {},
  } = props;

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
  const [avoidGroupWithOneItem, setAvoidGroupWithOneItem] = useState<string>(
    settingsCtx.get("avoidGroupWithOneItem") as string
  );

  useEffect(() => {
    setAvoidGroupWithOneItem(settingsCtx.get("avoidGroupWithOneItem") as string);
  }, [settingsCtx.changed]);

  useEffect(() => {
    setSearched(filterText);
  }, [filterText]);

  function filterParameters(searchTerm: string, parameters: RosParameter[] | undefined): RosParameter[] | undefined {
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
    const paramList: RosParameter[] = await (rosNode
      ? provider.getNodeParameters([rosNode.name])
      : provider.getParameterList());
    // sort parameter by name
    paramList.sort(function (a, b) {
      return a.name.localeCompare(b.name);
    });
    paramList.map((p) => {
      return p;
    });
    setRosParameters(paramList);
    setRosParametersFiltered(filterParameters(searched, paramList));
  }, [provider, rosNode, searched, setRosParameters, setRosParametersFiltered, filterParameters]);

  useEffect(() => {
    setTree(undefined);
    if (updateOnCreate) {
      getParameterList();
    }
  }, [forceReload]);

  // useEffect(() => {
  //   // update the parameter on create this component
  //   if (updateOnCreate) {
  //     getParameterList();
  //   }
  // }, []);

  // callback when updating a parameter
  async function updateParameter(
    parameter: RosParameter,
    newValue: string | boolean | number | string[],
    newType?: string
  ): Promise<void> {
    if (!provider.isAvailable()) return;

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
  }

  // create tree based on parameter namespace
  // parameters are grouped only if more then one is in the group
  function fillTree(fullPrefix: string, params: RosParameter[], itemId: string): TTreeItem {
    if (!params)
      return { params: [], count: 0, groupKeys: [], groupKey: "", groupName: "", fullPrefix: "", paramInfo: null };
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
        const groupKey = `${itemId}/${groupName}`;
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
          groupName: groupName,
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
    return { params: filteredParams, count, groupKeys, groupKey: "", groupName: "", fullPrefix: "", paramInfo: null };
  }

  // create parameter tree from filtered parameter list
  useEffect(() => {
    const subtree = fillTree(rosNode ? rosNode.name : "", rosParametersFiltered || [], rosNode ? rosNode.name : "");
    setTree(subtree.params);
    setExpandedFiltered([itemId, ...subtree.groupKeys]);
  }, [rosParametersFiltered]);

  function paramTreeToStyledItems(treeItems: TTreeItem[]): JSX.Element[] {
    return treeItems.map((param) => {
      let namespacePart = "";
      while (avoidGroupWithOneItem && param.params.length === 1) {
        const child = param.params[0];
        param.params = child.params;
        namespacePart = `${namespacePart}${param.groupName}/`;
      }
      if (param.paramInfo) {
        return (
          <ParameterTreeItem
            key={param.paramInfo.id}
            itemId={param.paramInfo.id}
            namespacePart={namespacePart}
            paramInfo={param.paramInfo}
            updateParameter={(param: RosParameter, value: string | boolean | number | string[], valueType?: string) =>
              updateParameter(param, value, valueType)
            }
            rosVersion={provider.rosVersion}
          />
        );
      } else {
        return (
          <ParameterGroupTreeItem
            key={param.groupKey}
            itemId={param.groupKey}
            namespacePart={namespacePart}
            groupName={param.groupName}
            icon={null}
            countChildren={param.count}
            requestData={false}
          >
            {paramTreeToStyledItems(param.params)}
          </ParameterGroupTreeItem>
        );
      }
    });
  }

  useEffect(() => {
    const params: RosParameter[] =
      rosParameters?.filter((p) => {
        let addParam = false;
        if (itemId === selectedItem) {
          addParam = true;
        } else if (p.id === selectedItem) {
          addParam = true;
        } else if (p.name.startsWith(selectedItem)) {
          addParam = true;
        }
        return addParam;
      }) || [];
    onSelectParams(provider, params);
  }, [selectedItem, rosParameters]);

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
    return (
      <ParameterGroupTreeItem
        key={itemId}
        itemId={itemId}
        namespacePart={""}
        groupName={rosNode ? rosNode.name : provider.name()}
        icon={rosNode ? Label : ComputerIcon}
        providerName={provider.name()}
        countChildren={rosParameters ? rosParameters.length : 0}
        requestData={rosParameters === undefined}
      >
        {paramTreeToStyledItems(tree || [])}
      </ParameterGroupTreeItem>
    );
  }, [tree]);

  return (
    <Box ref={ref}>
      <Stack direction="row" sx={{ flexGrow: 1 }}>
        {(searched.length < EXPAND_ON_SEARCH_MIN_CHARS || (tree && tree.length > 0)) && (
          <SimpleTreeView
            aria-label="parameters"
            expandedItems={searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
            slots={{
              collapseIcon: ArrowDropDownIcon,
              expandIcon: ArrowRightIcon,
            }}
            sx={{ flexGrow: 1 }}
            expansionTrigger={"iconContainer"}
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
      </Stack>
    </Box>
  );
});

export default ParameterRootTree;
