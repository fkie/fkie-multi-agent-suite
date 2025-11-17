import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ComputerIcon from "@mui/icons-material/Computer";
import Label from "@mui/icons-material/Label";
import { Box, Stack } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";

import LoggingContext, { DEFAULT_BUG_TEXT } from "@/renderer/context/LoggingContext";
import SettingsContext from "@/renderer/context/SettingsContext";
import { RosNode, RosParameter } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { TParamListResult } from "@/renderer/providers/Provider";
import { findIn } from "@/renderer/utils";
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

export default function ParameterRootTree(props: ParameterRootTreeProps): JSX.Element {
  const {
    provider,
    rosNode = undefined,
    updateOnCreate = false,
    filterText = "",
    forceReload,
    onSelectParams = (): void => {},
  } = props;

  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
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
  const [requestError, setRequestError] = useState<string>("");
  const [requesting, setRequesting] = useState<boolean>(false);
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
    setRequestError("");
    setRosParameters(undefined);
    setRequesting(true);
    const paramResult: TParamListResult = await (rosNode
      ? provider.getNodeParameters([rosNode.name])
      : provider.getParameterList());
    // sort parameter by name
    paramResult.params.sort((a, b) => {
      const an = `${a.node}/${a.name}`;
      const bn = `${b.node}/${b.name}`;
      // sort groups first
      const aCountSep = (an.match(/\/|\./g) || []).length;
      const bCountSep = (bn.match(/\/|\./g) || []).length;
      if (aCountSep === bCountSep) {
        return an.localeCompare(bn);
      }
      return aCountSep < bCountSep ? 1 : -1;
    });
    setRosParameters(paramResult.params);
    setRosParametersFiltered(filterParameters(searched, paramResult.params));
    if (paramResult.errors.length > 0) {
      setRequestError(paramResult.errors.join(";\n"));
    }
    setRequesting(false);
  }, [provider, rosNode, searched, setRosParameters, setRosParametersFiltered, filterParameters]);

  useEffect(() => {
    if (
      updateOnCreate ||
      (forceReload && (searched.length < EXPAND_ON_SEARCH_MIN_CHARS || findIn(searched, [itemId])))
    ) {
      setTree(undefined);
      // if forceReload, check if this root is in filtered list
      getParameterList();
    }
  }, [forceReload]);

  // create tree based on parameter namespace
  // parameters are grouped only if more then one is in the group
  function fillTree(fullPrefix: string, params: RosParameter[], itemId: string): TTreeItem {
    if (!params)
      return { params: [], count: 0, groupKeys: [], groupKey: "", groupName: "", fullPrefix: "", paramInfo: null };
    const byPrefixP1: Map<string, { restNameSuffix: string; paramInfo: RosParameter; isGroup: boolean }[]> = new Map();
    // count parameter for each group
    for (const param of params) {
      const nodeName = param.node && param.node.length > 0 ? `${param.node}/` : "";
      const nameSuffix = `${nodeName}${param.name}`
        .replace(/\./g, "/")
        .replace("//", "/")
        .slice(fullPrefix.length + 1);
      const [groupName, ...restName] = nameSuffix.split("/");
      if (restName.length > 0) {
        const restNameSuffix = restName.join("/");
        if (byPrefixP1.has(groupName)) {
          byPrefixP1.get(groupName)?.push({ restNameSuffix, paramInfo: param, isGroup: true });
        } else {
          byPrefixP1.set(groupName, [{ restNameSuffix, paramInfo: param, isGroup: true }]);
        }
      } else {
        byPrefixP1.set(groupName, [{ restNameSuffix: "", paramInfo: param, isGroup: false }]);
      }
    }

    // create result
    let count = 0;
    const groupKeys: string[] = [];
    const filteredParams: TTreeItem[] = [];
    byPrefixP1.forEach((value, groupName) => {
      // don't create group with one parameter
      const newFullPrefix = `${fullPrefix}/${groupName}`;
      const paramValues = value.filter((item) => !item.isGroup);
      const groupValues = value.filter((item) => !paramValues.includes(item));
      if (groupValues.length > 0) {
        const groupKey: string = itemId ? `${itemId}-${groupName}` : groupName;
        // const groupKey = `${itemId}/${groupName}`;
        groupKeys.push(groupKey);
        const groupParams: RosParameter[] = value.map((item) => {
          return item.paramInfo;
        });
        const subResult = fillTree(newFullPrefix, groupParams, groupKey);
        count += subResult.count;
        if (subResult.groupKeys.length > 0) {
          groupKeys.push(...subResult.groupKeys);
        }
        filteredParams.push({
          groupKey: groupKey,
          groupName: groupName,
          params: subResult.params,
          count: subResult.count,
          fullPrefix: fullPrefix,
          groupKeys: groupKeys,
          paramInfo: null,
        } as TTreeItem);
      }
      for (const item of paramValues) {
        filteredParams.push({
          groupKey: "",
          groupName: "",
          params: [],
          count: 0,
          fullPrefix: fullPrefix,
          groupKeys: groupKeys,
          paramInfo: item.paramInfo,
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

  function paramTreeToStyledItems(rootPath: string, treeItem: TTreeItem): JSX.Element {
    if (treeItem.paramInfo) {
      return (
        <ParameterTreeItem
          key={treeItem.paramInfo.id}
          itemId={treeItem.paramInfo.id}
          namespacePart={rootPath}
          paramInfo={treeItem.paramInfo}
          provider={provider}
        />
      );
    }
    if (avoidGroupWithOneItem && treeItem.params.length === 1) {
      // avoid groups with one item
      return paramTreeToStyledItems(
        rootPath.length > 0 ? `${rootPath}.${treeItem.groupName}` : treeItem.groupName,
        treeItem.params[0]
      );
    }
    return (
      <ParameterGroupTreeItem
        key={treeItem.groupKey}
        itemId={treeItem.groupKey}
        namespacePart={rootPath}
        groupName={treeItem.groupName}
        icon={null}
        countChildren={treeItem.count}
        requestData={false}
      >
        {treeItem.params.map((subItem) => {
          return paramTreeToStyledItems("", subItem);
        })}
      </ParameterGroupTreeItem>
    );
  }

  useEffect(() => {
    const params: RosParameter[] =
      rosParameters?.filter((p) => {
        let addParam = false;
        if (itemId === selectedItem) {
          addParam = true;
        } else if (p.id === selectedItem) {
          addParam = true;
        } else if (p.node?.startsWith(selectedItem)) {
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
        requestData={requesting}
        requestError={requestError}
      >
        {(tree || []).map((item) => {
          return paramTreeToStyledItems("", item);
        })}
      </ParameterGroupTreeItem>
    );
  }, [tree, requesting, requestError, avoidGroupWithOneItem]);

  return (
    <Box>
      <Stack direction="row" sx={{ flexGrow: 1 }}>
        {(searched.length < EXPAND_ON_SEARCH_MIN_CHARS || (tree && tree.length > 0) || findIn(searched, [itemId])) && (
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
                  if (itemId) {
                    copyExpanded.push(itemId);
                  }
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
}
