import DeleteIcon from "@mui/icons-material/Delete";
import RefreshIcon from "@mui/icons-material/Refresh";
import { Alert, AlertTitle, Box, IconButton, Stack, Tooltip } from "@mui/material";
import { useContext, useEffect, useMemo, useReducer, useState } from "react";

import { ParameterRootTree } from "@/renderer/components/ParameterTreeView";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { DEFAULT_BUG_TEXT, LoggingContext } from "@/renderer/context/LoggingContext";
import { RosContext } from "@/renderer/context/RosContext";
import { BUTTON_LOCATIONS, SettingsContext } from "@/renderer/context/SettingsContext";
import { RosNode, RosNodeStatus, RosParameter } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { EventProviderRosNodes } from "@/renderer/providers/events";
import { EVENT_PROVIDER_ROS_NODES } from "@/renderer/providers/eventTypes";
import { useCustomEventListener } from "react-custom-events";

type TRootData = {
  provider: Provider;
  rosNode: RosNode | undefined;
  updateOnCreate?: boolean;
};

interface ParameterPanelProps {
  nodes: RosNode[];
  providers: string[];
}

export default function ParameterPanel(props: ParameterPanelProps): JSX.Element {
  const { nodes, providers } = props;
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const [rootData, setRootData] = useState<TRootData[]>([]);
  const [forceReload, setForceReload] = useReducer((x) => x + 1, 0);
  const [searched, setSearched] = useState<string>("");
  const [selectedParameter, setSelectedParameter] = useState<{ provider: Provider; params: RosParameter[] }>();
  const [showWarning, setShowWarning] = useState<boolean>(false);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [buttonLocation, setButtonLocation] = useState<string>(settingsCtx.get("buttonLocation") as string);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setButtonLocation(settingsCtx.get("buttonLocation") as string);
  }, [settingsCtx.changed]);

  async function deleteSelectedParameters(): Promise<void> {
    if (selectedParameter) {
      if (!selectedParameter.provider.deleteParameters) {
        logCtx.error(
          `Provider ${selectedParameter.provider.name()} does not support [deleteParameters] method`,
          DEFAULT_BUG_TEXT
        );
        return;
      }

      const nodesParameter = {};
      for (const p of selectedParameter.params) {
        if (!nodesParameter[p.node]) {
          nodesParameter[p.node] = [];
        }
        nodesParameter[p.node].push(p.name);
      }
      await Promise.all(
        Object.keys(nodesParameter).map(async (node) => {
          const result = await selectedParameter.provider.deleteParameters(nodesParameter[node], node);

          if (result.result) {
            logCtx.success(
              `Parameter deleted successfully from ${selectedParameter.provider.name()}`,
              `${JSON.stringify(selectedParameter.params)}`
            );
          } else {
            logCtx.error(`Could not delete parameters from ${selectedParameter.provider.name()}`, `${result.message}`);
          }
        })
      );
    }
    // TODO: update only involved provider / nodes
    setForceReload();
  }

  useEffect(() => {
    if (!rosCtx.initialized) return;
    if (nodes?.length > 0) {
      const newRootData: TRootData[] = [];
      for (const node of nodes) {
        const provider: Provider | undefined = rosCtx.getProviderById(node.providerId);
        if (provider) {
          newRootData.push({ provider: provider, rosNode: node, updateOnCreate: true } as TRootData);
        }
      }
      setRootData(newRootData);
    } else if (providers?.length > 0) {
      const newRootData: TRootData[] = [];
      for (const providerId of providers) {
        const provider: Provider | undefined = rosCtx.getProviderById(providerId);
        if (provider) {
          if (provider.rosVersion === "1") {
            newRootData.push({ provider: provider, rosNode: undefined, updateOnCreate: true } as TRootData);
          } else {
            provider.rosNodes.map((item) => {
              if (item.status === RosNodeStatus.RUNNING) {
                newRootData.push({ provider: provider, rosNode: item, updateOnCreate: false } as TRootData);
              }
            });
          }
        }
      }
      setRootData(newRootData);
    } else {
      const newRootData: TRootData[] = [];
      for (const provider of rosCtx.providers) {
        if (provider.rosVersion === "1") {
          newRootData.push({ provider: provider, rosNode: undefined, updateOnCreate: true } as TRootData);
        } else {
          provider.rosNodes.map((item) => {
            if (item.status === RosNodeStatus.RUNNING) {
              newRootData.push({ provider: provider, rosNode: item, updateOnCreate: false } as TRootData);
            }
          });
        }
      }
      setShowWarning(newRootData.length > 5);
      setRootData(newRootData);
    }
  }, [rosCtx.initialized, rosCtx.providers]);

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  useCustomEventListener(EVENT_PROVIDER_ROS_NODES, (data: EventProviderRosNodes) => {
    if (nodes.length === 1 && nodes[0].providerId === data.provider.id) {
      for (const rosNode of data.nodes) {
        if (rosNode.name === nodes[0].name) {
          setRootData([{ provider: data.provider, rosNode: rosNode, updateOnCreate: true } as TRootData]);
          break;
        }
      }
    }
  });

  const createParameterItems = useMemo(() => {
    return (
      <Stack key="parameter-panel-param-items" direction="column" sx={{ flexGrow: 1 }}>
        {rootData.map((root) => {
          return (
            <ParameterRootTree
              key={root.rosNode ? root.rosNode.idGlobal : root.provider.id}
              provider={root.provider}
              rosNode={root.rosNode}
              updateOnCreate={root.updateOnCreate || rootData.length <= 5}
              filterText={searched}
              forceReload={forceReload}
              onSelectParams={(provider: Provider, params: RosParameter[]) => {
                if (params.length === 0) {
                  setSelectedParameter(undefined);
                } else {
                  if (provider.rosVersion === "1") {
                    setSelectedParameter({ provider: provider, params: params });
                  } else {
                    setSelectedParameter(undefined);
                  }
                }
              }}
            />
          );
        })}
      </Stack>
    );
  }, [rootData, forceReload, searched]);

  const createButtons = useMemo(() => {
    return (
      <Stack direction="row" spacing={0.5} alignItems="center">
        <Tooltip title="Reload parameter list" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
          <IconButton
            size="small"
            onClick={() => {
              setShowWarning(false);
              setForceReload();
            }}
          >
            <RefreshIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>

        {selectedParameter && (
          <Tooltip
            title="Delete selected ROS1 parameter"
            placement="bottom"
            enterDelay={tooltipDelay}
            disableInteractive
          >
            <span>
              <IconButton
                disabled={!selectedParameter}
                size="small"
                aria-label="Delete selected parameter"
                onClick={() => {
                  deleteSelectedParameters();
                }}
              >
                <DeleteIcon fontSize="inherit" />
              </IconButton>
            </span>
          </Tooltip>
        )}
      </Stack>
    );
  }, [tooltipDelay, selectedParameter]);

  return (
    <Box height="100%" overflow="auto" sx={{ backgroundColor: backgroundColor }}>
      <Stack spacing={1} height="100%">
        <Stack direction="row" spacing={0.5} alignItems="center">
          {buttonLocation === BUTTON_LOCATIONS.LEFT && createButtons}

          <SearchBar
            onSearch={setSearched}
            placeholder="Filter parameters (OR: <space>, AND: +, NOT: !)"
            // defaultValue={initialSearchTerm}
            fullWidth
          />
          {buttonLocation === BUTTON_LOCATIONS.RIGHT && createButtons}
        </Stack>
        {showWarning && (
          <Alert
            severity="warning"
            style={{ minWidth: 0 }}
            onClose={() => {
              setShowWarning(false);
            }}
          >
            <AlertTitle>
              {
                "Many ROS nodes must be queried. This can take some time and affect other functions of the GUI during this time."
              }
            </AlertTitle>
            {"Use the filter and then click on refresh. And then delete filter!"}
          </Alert>
        )}
        {createParameterItems}
      </Stack>
    </Box>
  );
}
