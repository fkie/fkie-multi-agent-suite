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
import { RosNode, RosNodeStatus, RosParameter } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { ParameterGroupTreeItem, ParameterRootTree } from "@/renderer/components/ParameterTreeView";

type TParamRootItem = {
  id: string;
  node?: RosNode;
  provider?: Provider;
};

type TRootData = {
  provider: Provider;
  rosNode: RosNode | undefined;
  updateOnCreate?: boolean;
};

type TRootTree = {
  rootId: string;
  items: TTreeItem[];
};

type TTreeItem = {
  groupKey: string;
  groupName: string;
  params: TTreeItem[];
  count: number;
  fullPrefix: string;
  groupKeys: string[];
  paramInfo: RosParameter | null;
};

interface ParameterPanelProps {
  nodes: RosNode[];
  providers: string[];
}

export default function ParameterPanel(props: ParameterPanelProps) {
  const { nodes, providers } = props;
  const EXPAND_ON_SEARCH_MIN_CHARS = 2;
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  // const [roots, setRoots] = useState<TParamRootItem[]>([]);
  const [rootData, setRootData] = useState<TRootData[]>([]);
  // const [receivedData, setReceivedData] = useState<TRootData>();
  const [rootDataFiltered, setRootDataFiltered] = useState<TRootData[]>([]);
  // const [rootDataTree, setRootDataTree] = useState<TRootTree[]>([]);
  // const [expanded, setExpanded] = useState<string[]>([]);
  // const [expandedFiltered, setExpandedFiltered] = useState<string[]>([]);
  // const [nodeFilter, setNodeFilter] = useState<boolean>(false);
  const [searched, setSearched] = useState<string>("");
  // const [selectedItem, setSelectedItem] = useState<string>("");
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  // // debounced search callback
  // // search in the origin parameter list and create a new one
  // const onSearch = useDebounceCallback((searchTerm: string) => {
  //   // if (!searchTerm) {
  //   //   return setRootDataFiltered(rootData);
  //   // }
  //   // return setRootDataFiltered(
  //   //   rootData?.filter((p) => {
  //   //     const isMatch = findIn(searched, p.rosNode?.name ? [p.rosNode.name, p.provider.name()] : [p.provider.name()]);
  //   //     return isMatch;
  //   //   })
  //   // );
  // }, 300);

  // useEffect(() => {
  //   onSearch(searched);
  // }, [searched]);

  // useEffect(() => {
  //   console.log(`receivedData: ${receivedData?.rootId}`);
  //   if (receivedData) {
  //     setRootData((prev) => {
  //       return [...prev.filter((item) => item.rootId !== receivedData?.rootId), receivedData];
  //     });
  //     onSearch(searched);
  //   }
  // }, [receivedData]);

  // const getParameterList = useCallback(async () => {
  //   if (!rosCtx.initialized) return;
  //   setRootData([]);
  //   setRootDataFiltered([]);

  //   const newData: TRootData[] = [];
  //   roots.map(async (root: TParamRootItem) => {
  //     if (root.node) {
  //       console.log(`GET rootId: ${root.id}`);
  //       // TODO: optimize request if all nodes are from the same provider
  //       const provider = rosCtx.getProviderById(root.node.providerId);
  //       if (!provider || !provider.isAvailable()) return;
  //       // check if provider supports [getNodeParameters]
  //       if (!provider.getNodeParameters) {
  //         logCtx.error(`Provider ${provider.name()} does not support [getNodeParameters] method`, DEFAULT_BUG_TEXT);
  //         return;
  //       }
  //       const paramList: RosParameter[] = await provider.getNodeParameters([root.node.name]);
  //       // sort parameter by name
  //       paramList.sort(function (a, b) {
  //         return a.name.localeCompare(b.name);
  //       });
  //       // setRootData((prev) => [...prev, { rootId: root.id, params: paramList }]);
  //       // newData.push({ rootId: root.id, params: paramList });
  //       setReceivedData({ rootId: root.id, params: paramList });
  //       console.log(`GET done rootId: ${root.id}`);
  //     } else if (root.provider) {
  //       console.log(`GET provider rootId: ${root.id}`);
  //       // exclude not available provider
  //       if (!root.provider.isAvailable()) return;

  //       // check if provider supports [getParameterList]
  //       if (!root.provider.getParameterList) {
  //         logCtx.error(`Provider ${root.provider.name} does not support [getParameterList] method`, DEFAULT_BUG_TEXT);
  //         return;
  //       }
  //       const paramList: RosParameter[] = await root.provider.getParameterList();
  //       // sort parameter by name
  //       paramList.sort(function (a, b) {
  //         return a.name.localeCompare(b.name);
  //       });
  //       // setRootData((prev) => [...prev, { rootId: root.id, params: paramList }]);
  //       setReceivedData({ rootId: root.id, params: paramList });
  //       console.log(`GET done provider rootId: ${root.id}`);
  //     }
  //   });
  //   setRootData(newData);
  //   onSearch(searched);
  //   // eslint-disable-next-line react-hooks/exhaustive-deps
  // }, [rosCtx.initialized, rosCtx.providers, nodes, roots, rootData, searched]);

  // // debounced callback when updating a parameter
  // const updateParameter = useDebounceCallback(
  //   async (parameter: RosParameter, newValue: string | boolean | number | string[], newType?: string) => {
  //     const provider = rosCtx.getProviderById(parameter.providerId);
  //     if (!provider || !provider.isAvailable()) return;

  //     if (!provider.setParameter) {
  //       logCtx.error(
  //         `Provider ${rosCtx.getProviderName(parameter.providerId)} does not support [setParameter] method`,
  //         DEFAULT_BUG_TEXT
  //       );
  //       return;
  //     }
  //     parameter.value = newValue;
  //     if (newType) {
  //       parameter.type = newType;
  //     }
  //     const result = await provider.setParameter(parameter);

  //     if (result) {
  //       logCtx.success("Parameter updated successfully", `Parameter: ${parameter.name}, value: ${parameter.value}`);
  //     } else {
  //       logCtx.error(`Could not update parameter [${parameter.name}]`, DEFAULT_BUG_TEXT);
  //     }
  //   },
  //   300
  // );

  // const deleteParameters = useCallback(
  //   (params: TRootData[]) => {
  //     params.map(async (root: TRootData) => {
  //       if (root.params.length > 0) {
  //         const provider = rosCtx.getProviderById(root.rootId);
  //         if (!provider || !provider.isAvailable()) return;

  //         if (!provider.deleteParameters) {
  //           logCtx.error(
  //             `Provider ${rosCtx.getProviderName(root.rootId)} does not support [deleteParameters] method`,
  //             DEFAULT_BUG_TEXT
  //           );
  //           return;
  //         }

  //         const result = await provider.deleteParameters(root.params.map((p) => p.name));

  //         if (result) {
  //           logCtx.success(`Parameter deleted successfully from ${provider.name()}`, `${JSON.stringify(params)}`);
  //         } else {
  //           logCtx.error(`Could not delete parameters from ${provider.name()}`, DEFAULT_BUG_TEXT);
  //         }
  //       }
  //     });
  //     // TODO: update only involved provider / nodes
  //     getParameterList();
  //   },
  //   [getParameterList, logCtx, rosCtx]
  // );

  // // callback when deleting parameters
  // const onDeleteParameters = useCallback(async () => {
  //   if (!rootDataFiltered || rootDataFiltered.length === 0) {
  //     logCtx.warn("No parameters to be deleted", "");
  //     return;
  //   }

  //   deleteParameters(rootDataFiltered);

  //   // Do we need to get the whole list of parameters again?
  //   // getParameterList();
  // }, [rootDataFiltered, deleteParameters, logCtx]);

  useEffect(() => {
    if (!rosCtx.initialized) return;
    console.log(`PARAM PANEL: ${nodes} prov: ${providers}`);
    if (nodes?.length > 0) {
      const newRootData: TRootData[] = [];
      nodes.forEach((node) => {
        const provider: Provider | undefined = rosCtx.getProviderById(node.providerId);
        if (provider) {
          newRootData.push({ provider: provider, rosNode: node, updateOnCreate: true } as TRootData);
        }
      });
      setRootData(newRootData);
    } else if (providers?.length > 0) {
      const newRootData: TRootData[] = [];
      providers.forEach((providerId) => {
        const provider: Provider | undefined = rosCtx.getProviderById(providerId);
        if (provider) {
          newRootData.push({ provider: provider, rosNode: undefined, updateOnCreate: true } as TRootData);
        }
      });
      setRootData(newRootData);
    } else {
      const newRootData: TRootData[] = [];
      rosCtx.providers.forEach((provider) => {
        console.log(`PANEL get nodes: ${rosCtx.providers.length}`);
        if (provider.rosVersion === "1") {
          newRootData.push({ provider: provider, rosNode: undefined, updateOnCreate: true } as TRootData);
        } else {
          provider.rosNodes.map((item) => {
            console.log(`PANEL item.status: ${item.status}`);
            if (item.status === RosNodeStatus.RUNNING) {
              console.log(`PANEL add: ${item.name}`);
              newRootData.push({ provider: provider, rosNode: item, updateOnCreate: true } as TRootData);
            }
          });
        }
      });
      console.log(`newRootData ${newRootData.length}`);
      setRootData(newRootData);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosCtx.initialized, rosCtx.providers]);

  // useEffect(() => {
  //   getParameterList();
  // }, [roots]);

  // // create tree based on parameter namespace
  // // parameters are grouped only if more then one is in the group
  // const fillTree = (fullPrefix: string, params: RosParameter[], itemId: string) => {
  //   if (!params) return { params: [], count: 0, groupKeys: [] };
  //   const byPrefixP1: Map<string, { restNameSuffix: string; paramInfo: RosParameter }[]> = new Map();
  //   // count parameter for each group
  //   params.forEach((param) => {
  //     const nameSuffix = param.name.slice(fullPrefix.length + 1);
  //     const [groupName, ...restName] = nameSuffix.split("/");
  //     if (restName.length > 0) {
  //       const restNameSuffix = restName.join("/");
  //       if (byPrefixP1.has(groupName)) {
  //         byPrefixP1.get(groupName)?.push({ restNameSuffix, paramInfo: param });
  //       } else {
  //         byPrefixP1.set(groupName, [{ restNameSuffix, paramInfo: param }]);
  //       }
  //     } else {
  //       byPrefixP1.set(groupName, [{ restNameSuffix: "", paramInfo: param }]);
  //     }
  //   });

  //   // create result
  //   let count = 0;
  //   const groupKeys: string[] = [];
  //   const filteredParams: TTreeItem[] = [];
  //   byPrefixP1.forEach((value, groupName) => {
  //     // don't create group with one parameter
  //     const newFullPrefix = `${fullPrefix}/${groupName}`;
  //     if (value.length > 1) {
  //       const groupKey = `${itemId}-${groupName}`;
  //       groupKeys.push(groupKey);
  //       const groupParams: RosParameter[] = value.map((item) => {
  //         return item.paramInfo;
  //       });
  //       const subResult = fillTree(newFullPrefix, groupParams, groupKey);
  //       // the result count is 0 -> we added multiple provider for topic with same name.
  //       if (subResult.count === 0) {
  //         count += 1;
  //       } else {
  //         count += subResult.count;
  //       }
  //       if (subResult.groupKeys.length > 0) {
  //         groupKeys.push(...subResult.groupKeys);
  //       }
  //       filteredParams.push({
  //         groupKey: groupKey,
  //         groupName: `/${groupName}`,
  //         params: subResult.params,
  //         count: subResult.count,
  //         fullPrefix: newFullPrefix,
  //         groupKeys: groupKeys,
  //         paramInfo: null,
  //       } as TTreeItem);
  //     } else {
  //       filteredParams.push({
  //         groupKey: "",
  //         groupName: "",
  //         params: [],
  //         count: 0,
  //         fullPrefix: newFullPrefix,
  //         groupKeys: groupKeys,
  //         paramInfo: value[0].paramInfo,
  //       } as TTreeItem);
  //       count += 1;
  //     }
  //   });
  //   return { params: filteredParams, count, groupKeys };
  // };

  // create parameter tree from filtered parameter list
  // useEffect(() => {
  //   const rootKeys: string[] = [];
  //   setRootDataTree(
  //     rootDataFiltered.map((root: TRootData) => {
  //       const subtree = fillTree(nodeFilter ? `${root.rootId}` : "", root.params, root.rootId);
  //       rootKeys.push(root.rootId);
  //       rootKeys.push(...subtree.groupKeys);
  //       return { rootId: root.rootId, items: subtree.params } as TRootTree;
  //     })
  //   );
  //   setExpandedFiltered(rootKeys);
  //   // eslint-disable-next-line react-hooks/exhaustive-deps
  // }, [nodeFilter, rootDataFiltered, roots]);

  // const getIcon = (obj: TParamRootItem) => {
  //   if (obj.provider) {
  //     // it is Provider
  //     return ComputerIcon;
  //     // return PrecisionManufacturingIcon;
  //   }
  //   if (obj.node) {
  //     // it is RosNode
  //     return Label;
  //   }
  //   return HideSourceIcon;
  // };

  // const paramTreeToStyledItems = (rootPath: string, treeItems: TTreeItem[]) => {
  //   return treeItems.map((param) => {
  //     if (param.paramInfo) {
  //       console.log(`param.paramInfo.id: ${param.paramInfo.id}`);
  //       return (
  //         <ParameterTreeItem
  //           key={param.paramInfo.id}
  //           itemId={param.paramInfo.id}
  //           rootPath={rootPath}
  //           paramInfo={param.paramInfo}
  //           updateParameter={(param: RosParameter, value: string | boolean | number | string[], valueType?: string) =>
  //             updateParameter(param, value, valueType)
  //           }
  //         />
  //       );
  //     } else {
  //       console.log(`param.groupKey: ${param.groupKey}`);
  //       return (
  //         <ParameterGroupTreeItem
  //           key={param.groupKey}
  //           itemId={param.groupKey}
  //           rootPath={rootPath}
  //           groupName={param.groupName}
  //           icon={null}
  //           countChildren={param.count}
  //           requestData={false}
  //         >
  //           {paramTreeToStyledItems(param.fullPrefix, param.params)}
  //         </ParameterGroupTreeItem>
  //       );
  //     }
  //   });
  // };

  // const deleteParamsFromId = (treeItemId: string) => {
  //   // Sort parameter by provider
  //   deleteParameters(
  //     rootData.map((root: TRootData) => {
  //       return {
  //         rootId: root.rootId,
  //         params: root.params.filter((p) => {
  //           let addParam = false;
  //           if (root.rootId === treeItemId) {
  //             addParam = true;
  //           } else if (p.id === treeItemId) {
  //             addParam = true;
  //           } else if (treeItemId.endsWith("#")) {
  //             const trimmed = treeItemId.slice(0, treeItemId.length - 2);
  //             addParam = p.name.startsWith(trimmed);
  //           }
  //           return addParam;
  //           // if (addParam) {
  //           //   if (paramsToBeDeleted.has(p.providerId)) {
  //           //     paramsToBeDeleted.get(p.providerId)?.push(p.name);
  //           //   } else {
  //           //     paramsToBeDeleted.set(p.providerId, [p.name]);
  //           //   }
  //           // }
  //         }),
  //       };
  //     })
  //   );
  // };

  // const handleToggle = useCallback(
  //   (itemIds: string[]) => {
  //     if (searched.length < EXPAND_ON_SEARCH_MIN_CHARS) {
  //       setExpanded(itemIds);
  //     } else {
  //       setExpandedFiltered(itemIds);
  //     }
  //   },
  //   [searched]
  // );

  // const createParameterItems = useMemo(() => {
  //   return (
  //     <>
  //       {roots.map((root: TParamRootItem) => {
  //         const filteredItems = rootDataFiltered.filter((item) => item.rootId === root.id);
  //         const dataExists = rootData.filter((item) => item.rootId === root.id).length > 0;
  //         const rootItems: TRootTree[] = rootDataTree.filter((item) => item.rootId === root.id);
  //         console.log(`root.id: ${root.id}`);
  //         return (
  //           <ParameterGroupTreeItem
  //             key={`${root.id}`}
  //             itemId={`${root.id}`}
  //             rootPath={""}
  //             groupName={root.node ? root.id : root.provider ? rosCtx.getProviderName(root.id) : "unknown"}
  //             icon={getIcon(root)}
  //             providerName={rosCtx.getProviderName(root.id)}
  //             countChildren={filteredItems ? filteredItems.length : 0}
  //             requestData={!dataExists}
  //           >
  //             {paramTreeToStyledItems(nodeFilter ? `${root.id}` : "", rootItems[0]?.items || [])}
  //           </ParameterGroupTreeItem>
  //         );
  //       })}
  //     </>
  //   );
  // }, [rootDataFiltered]);

  const createParameterItems = useMemo(() => {
    return (
      <Stack key="parameter-panel-param-items" direction="column" sx={{ flexGrow: 1 }}>
        {rootData.map((root) => {
          console.log(`ADD ${root.rosNode?.idGlobal}`);
          return (
            <ParameterRootTree
              key={root.rosNode ? root.rosNode.idGlobal : root.provider.id}
              provider={root.provider}
              rosNode={root.rosNode}
              updateOnCreate={root.updateOnCreate}
              filterText={searched}
            />
          );
        })}
      </Stack>
    );
  }, [rootData]);

  return (
    <Box height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
      <Stack
        spacing={1}
        height="100%"
        // sx={{
        //   height: '100%',
        //   display: 'flex',
        // }}
      >
        <Stack direction="row" spacing={0.5} alignItems="center">
          {/* <Tooltip title="Delete selected parameter" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
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
          </Tooltip> */}
          <SearchBar
            onSearch={setSearched}
            placeholder="Filter parameters (OR: <space>, AND: +, NOT: !)"
            // defaultValue={initialSearchTerm}
            fullWidth
          />
          {/* {searched && (
            <Tooltip title="Delete filtered parameters" placement="bottom" disableInteractive>
              <IconButton size="small" onClick={onDeleteParameters} color="warning">
                <DeleteIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )} */}
        </Stack>
        {createParameterItems}
        {/* <Stack direction="column" sx={{ flexGrow: 1 }}>
          {rootData.map((root) => {
            console.log(`ADD ${root.rosNode?.idGlobal}`);
            return (
              <ParameterRootTree
                key={root.rosNode ? root.rosNode.idGlobal : root.provider.id}
                provider={root.provider}
                rosNode={root.rosNode}
                updateOnCreate={root.updateOnCreate}
                filterText={searched}
              />
            );
          })} */}
        {/* <Box width="100%" height="100%" overflow="auto">
            <SimpleTreeView
              aria-label="parameters"
              expandedItems={searched.length < EXPAND_ON_SEARCH_MIN_CHARS ? expanded : expandedFiltered}
              slots={{
                collapseIcon: ArrowDropDownIcon,
                expandIcon: ArrowRightIcon,
              }}
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
          </Box> */}
        {/* </Stack> */}
      </Stack>
    </Box>
  );
}

ParameterPanel.propTypes = {
  nodes: PropTypes.arrayOf(PropTypes.any),
  providers: PropTypes.arrayOf(PropTypes.string),
};
