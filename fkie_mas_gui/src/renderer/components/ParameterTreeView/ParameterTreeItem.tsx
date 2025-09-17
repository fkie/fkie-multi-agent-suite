import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import {
  Box,
  Grid,
  IconButton,
  Input,
  Menu,
  MenuItem,
  Slider,
  Stack,
  Switch,
  Tooltip,
  Typography,
} from "@mui/material";
import React, { forwardRef, LegacyRef, useContext, useEffect, useMemo, useState } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import RosContext from "@/renderer/context/RosContext";
import SettingsContext from "@/renderer/context/SettingsContext";
import { RosParameter, RosParameterRange, RosParameterValue } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { treeItemClasses } from "@mui/x-tree-view";
import OverflowMenu from "../UI/OverflowMenu";
import StyledTreeItem from "./StyledTreeItem";

interface ParameterTreeItemProps {
  itemId: string;
  namespacePart: string;
  paramInfo: RosParameter;
  provider: Provider;
}

const ParameterTreeItem = forwardRef<HTMLDivElement, ParameterTreeItemProps>(function ParameterTreeItem(props, ref) {
  const { itemId, namespacePart, paramInfo, provider } = props;

  function fixStringArray(val: RosParameterValue): RosParameterValue {
    if (Array.isArray(val)) {
      return val.map((v) => {
        if (typeof v === "string") {
          return v.includes(",") || v.includes("'") ? `"${v}"` : v;
        }
        return v;
      });
    }
    return val;
  }

  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [parameterType, setParameterType] = useState<string>(paramInfo.type);
  const [changed, setChanged] = useState<boolean>(false);
  const [value, setValue] = useState(fixStringArray(paramInfo.value));
  const [typedValue, setTypedValue] = useState<string | number | boolean | string[]>(toTypedValue(paramInfo.value));
  const [name, setName] = useState<string>("");
  const [namespace, setNamespace] = useState<string>("");
  const [showDescription, setShowDescription] = useState<boolean>(false);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);
  const [showParameterType, setShowParameterType] = useState<boolean>(settingsCtx.get("showParameterType") as boolean);

  useEffect(() => {
    setShowParameterType(settingsCtx.get("showParameterType") as boolean);
  }, [settingsCtx.changed]);

  function updateValue(val: RosParameterValue): void {
    setValue(fixStringArray(val));
  }

  // biome-ignore lint/correctness/useExhaustiveDependencies: <explanation>
  useEffect(() => {
    if (!paramInfo.name) return;
    const nameParts = paramInfo.name.replaceAll("/", ".").replaceAll("..", ".").split(".");
    setName(`${nameParts.pop()}`);
    setNamespace(namespacePart ? `${namespacePart}.` : "");
    setTypedValue(toTypedValue(value));
  }, []);

  useEffect(() => {
    setTypedValue(toTypedValue(value));
  }, [value]);

  function toTypedValue(value): string | number | boolean | string[] {
    if (paramInfo.type === "float") {
      return Number.parseFloat(value) || 0.0;
    }
    if (paramInfo.type === "int") {
      return Number.parseInt(value) | 0;
    }
    return value;
  }

  // callback when updating a parameter
  async function updateParameter(
    parameter: RosParameter,
    newValue: RosParameterValue,
    newType?: string
  ): Promise<void> {
    if (!provider.isAvailable()) return;

    if (!provider.setParameter) {
      logCtx.error(
        `Provider ${rosCtx.getProviderName(parameter.providerId)} does not support [setParameter] method`,
        ""
      );
      return;
    }
    if (newType !== undefined) {
      parameter.type = newType;
    }
    const oldValue = parameter.value;
    if (newValue === undefined || (["int", "float", "bool"].includes(parameter.type) && `${newValue}`.length === 0)) {
      // do not update parameter if new value is undefined or
      // not valid integer, float or boolean
      if (parameter.type === "float") {
        parameter.value = 0.0;
      } else if (parameter.type === "int") {
        parameter.value = 0;
      } else if (parameter.type === "bool") {
        parameter.value = false;
      }
    } else {
      parameter.value = newValue;
    }

    const result = await provider.setParameter(parameter.name, parameter.type, `${parameter.value}`, parameter.node);

    if (result.result) {
      logCtx.success("Parameter updated successfully", `Parameter: ${parameter.name}, value: ${parameter.value}`);
      if (result.value) {
        updateValue(result.value);
        parameter.value = result.value;
      }
      if (result.value_type) {
        paramInfo.type = result.value_type;
      }
    } else {
      logCtx.error(`Could not update parameter [${parameter.name}]`, `${result.message}`);
      parameter.value = oldValue;
      updateValue(oldValue);
    }
  }

  function handleKey(event: React.KeyboardEvent): void {
    if (event.key === "Enter") {
      updateParameter(paramInfo, typedValue, paramInfo.type);
      setChanged(false);
    } else if (event.key === "Escape") {
      updateValue(paramInfo.value);
      setChanged(false);
    }
  }

  function handleChange(event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>): void {
    setChanged(true);
    updateValue(event.target.value);
  }

  function handleSliderChange(_event: Event, newValue: number | number[]): void {
    setChanged(true);
    updateValue(newValue as number);
  }

  function onLeave(): void {
    if (changed) {
      updateParameter(paramInfo, typedValue, paramInfo.type);
      setChanged(false);
    }
  }

  const typeOptions = [
    {
      name: "int",
      key: "int",
      onClick: (): void => {
        setParameterType("int");
        updateParameter(paramInfo, paramInfo.value, "int");
      },
    },
    {
      name: "float",
      key: "float",
      onClick: (): void => {
        setParameterType("float");
        updateParameter(paramInfo, paramInfo.value, "float");
      },
    },
    {
      name: "str",
      key: "str",
      onClick: (): void => {
        setParameterType("str");
        updateParameter(paramInfo, paramInfo.value, "str");
      },
    },
    {
      name: "bool",
      key: "bool",
      onClick: (): void => {
        setParameterType("bool");
        updateParameter(paramInfo, paramInfo.value, "bool");
      },
    },
    {
      name: "str[]",
      key: "str[]",
      onClick: (): void => {
        setParameterType("str[]");
        updateParameter(paramInfo, paramInfo.value, "str[]");
      },
    },
    {
      name: "int[]",
      key: "int[]",
      onClick: (): void => {
        setParameterType("int[]");
        updateParameter(paramInfo, paramInfo.value, "int[]");
      },
    },
    {
      name: "float[]",
      key: "float[]",
      onClick: (): void => {
        setParameterType("float[]");
        updateParameter(paramInfo, paramInfo.value, "float[]");
      },
    },
    {
      name: "bool[]",
      key: "bool[]",
      onClick: (): void => {
        setParameterType("bool[]");
        updateParameter(paramInfo, paramInfo.value, "bool[]");
      },
    },
  ];

  const renderInput = useMemo(() => {
    let inputElement: JSX.Element | null = null;
    if (["int", "float"].includes(parameterType)) {
      let range: RosParameterRange | null = null;
      if (paramInfo.floating_point_range && paramInfo.floating_point_range.length === 1) {
        range = paramInfo.floating_point_range[0];
      } else if (paramInfo.integer_range && paramInfo.integer_range.length === 1) {
        range = paramInfo.integer_range[0];
      }
      // TODO: handle multiple parameter ranges
      if (range) {
        return (
          <Stack direction="row" sx={{ flexGrow: 1 }}>
            <Input
              id={`input-${paramInfo.id}`}
              value={value}
              size="small"
              onChange={(event) => handleChange(event)}
              onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
              onBlur={() => onLeave()}
              sx={{ minWidth: "80px" }}
              inputProps={{
                step: range.step || 1.0,
                min: range.from_value,
                max: range.to_value,
                type: "number",
                "aria-labelledby": `slider-${paramInfo.id}`,
              }}
            />
            <Slider
              id={`slider-${paramInfo.id}`}
              sx={{ minWidth: "80px", marginLeft: "1em" }}
              value={value as number}
              step={range.step || 1.0}
              min={range.from_value}
              max={range.to_value}
              onChange={(event, newValue) => handleSliderChange(event, newValue)}
              onChangeCommitted={() => onLeave()}
              aria-labelledby="input-slider"
            />
          </Stack>
        );
      }
      inputElement = (
        <Input
          type="number"
          id={`input-${paramInfo.id}`}
          size="small"
          value={value}
          placeholder={JSON.stringify(value)}
          // InputProps={{ inputProps: { min: 0, max: 99 } }}
          fullWidth
          inputProps={{
            type: "number",
          }}
          disabled={paramInfo.readonly}
          onChange={(event) => handleChange(event)}
          onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
          onBlur={() => onLeave()}
        />
      );
    } else if (["list", "str[]", "int[]", "float[]", "bool[]"].includes(parameterType)) {
      // TODO: show proper list/arrays
      inputElement = (
        <Input
          id={`input-${paramInfo.id}`}
          value={value}
          placeholder={`${JSON.stringify(paramInfo.value)}`}
          size="small"
          fullWidth
          disabled={paramInfo.readonly}
          onChange={(event) => handleChange(event)}
          onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
          onBlur={() => onLeave()}
        />
      );
    } else if (["bool"].includes(parameterType)) {
      inputElement = (
        <Switch
          id={`input-${paramInfo.id}`}
          checked={!!typedValue}
          size="small"
          disabled={paramInfo.readonly}
          onChange={(event) => {
            updateParameter(paramInfo, event.target.checked, "bool");
            updateValue(event.target.checked);
          }}
        />
      );
    } else {
      // default render (usually string)
      inputElement = (
        <Input
          key={paramInfo.id}
          id={`input-${paramInfo.id}`}
          value={value}
          placeholder={`${paramInfo.value}`}
          size="small"
          fullWidth
          disabled={paramInfo.readonly}
          onChange={(event) => handleChange(event)}
          onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
          onBlur={() => onLeave()}
        />
      );
    }

    return inputElement;
  }, [paramInfo, value, typedValue, parameterType]);

  return (
    <StyledTreeItem
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
      sx={{
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: 0,
        },
      }}
      label={
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            // p: 0.3,
            padding: 0,
            pr: 0,
          }}
          onKeyDown={(e) => {
            // avoid lost focus inn input elements
            if (!["ArrowDown", "ArrowUp", "ArrowRight", "ArrowLeft", "Tab"].includes(e.key)) {
              e.stopPropagation();
            }
          }}
          onContextMenu={(event) => {
            event.preventDefault();
            setContextMenu(
              contextMenu === null
                ? {
                    mouseX: event.clientX + 2,
                    mouseY: event.clientY - 6,
                  }
                : null
            );
          }}
        >
          <Stack sx={{ flexGrow: 1 }} direction="column">
            <Grid container spacing={"1em"} sx={{alignItems: "center"}}>
              <Grid sx={{ flexGrow: 1 }} size={1}>
                <Stack direction="column" sx={{ minHeight: "2em" }}>
                  <Stack direction="row" sx={{ alignItems: "center", minHeight: "2em" }}>
                    <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                      {namespace}
                    </Typography>{" "}
                    <Typography
                      variant="body2"
                      sx={{ fontSize: "inherit", fontWeight: "bold", overflow: "hidden", textOverflow: "ellipsis" }}
                      onClick={(e) => {
                        if (e.detail === 2) {
                          navigator.clipboard.writeText(paramInfo.name);
                          logCtx.success(`${paramInfo.name} copied!`);
                        }
                      }}
                    >
                      {name}
                    </Typography>
                    {(paramInfo.description || paramInfo.additional_constraints) && (
                      <Tooltip
                        title={`${paramInfo.description}${paramInfo.additional_constraints ? `; Constraints: ${paramInfo.additional_constraints}` : ""}`}
                        placement="right"
                        disableInteractive
                      >
                        <span>
                          <IconButton
                            color="default"
                            onClick={(event) => {
                              setShowDescription((prev) => !prev);
                              event.stopPropagation();
                            }}
                            size="small"
                          >
                            <InfoOutlinedIcon fontSize="inherit" />
                          </IconButton>
                        </span>
                      </Tooltip>
                    )}
                  </Stack>
                </Stack>
              </Grid>
              <Grid sx={{ flexGrow: 1 }} size={showParameterType ? 2 : 1}>
                <Stack direction="row" sx={{ alignItems: "center" }}>
                  {showParameterType && provider.rosVersion === "1" && parameterType ? (
                    <OverflowMenu
                      icon={
                        <Typography variant="caption" color="inherit" padding={0.5} minWidth="5em">
                          [{parameterType}]
                        </Typography>
                      }
                      options={typeOptions}
                      id="provider-options"
                    />
                  ) : (
                    showParameterType && (
                      <Typography variant="caption" color="inherit" padding={0.5} minWidth="5em">
                        [{parameterType}]
                      </Typography>
                    )
                  )}
                  {paramInfo && renderInput}
                </Stack>
              </Grid>
            </Grid>
            {showDescription && (
              <Stack sx={{ flexGrow: 1 }} direction="column">
                <Typography
                  variant="body2"
                  sx={{ fontSize: "-0.5em", fontStyle: "italic" }}
                  onClick={(e) => {
                    setShowDescription((prev) => !prev);
                    e.stopPropagation();
                  }}
                >
                  {paramInfo.description}
                </Typography>
                {paramInfo.additional_constraints && (
                  <Typography
                    variant="body2"
                    sx={{ fontSize: "-0.5em", fontStyle: "italic" }}
                    onClick={(e) => {
                      setShowDescription((prev) => !prev);
                      e.stopPropagation();
                    }}
                  >
                    Constraints: {paramInfo.additional_constraints}
                  </Typography>
                )}
              </Stack>
            )}
          </Stack>
          <Menu
            open={contextMenu != null}
            onClose={() => setContextMenu(null)}
            anchorReference="anchorPosition"
            anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
          >
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(paramInfo.name);
                setContextMenu(null);
              }}
            >
              Copy parameter name
            </MenuItem>
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(JSON.stringify(paramInfo.value));
                setContextMenu(null);
              }}
            >
              Copy parameter value
            </MenuItem>
          </Menu>
        </Box>
      }
    />
  );
});

export default ParameterTreeItem;
