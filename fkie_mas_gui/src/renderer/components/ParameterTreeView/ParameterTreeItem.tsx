import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import { Box, Grid2, IconButton, Input, Slider, Stack, Switch, Tooltip, Typography } from "@mui/material";
import React, { forwardRef, LegacyRef, useContext, useEffect, useMemo, useState } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import { RosParameter } from "@/renderer/models";
import { RosParameterRange } from "@/renderer/models/RosParameter";
import OverflowMenu from "../UI/OverflowMenu";
import StyledTreeItem from "./StyledTreeItem";

interface ParameterTreeItemProps {
  itemId: string;
  namespacePart: string;
  paramInfo: RosParameter;
  updateParameter: (param: RosParameter, value: string | boolean | number | string[], valueType?: string) => void;
  rosVersion?: string;
}

const ParameterTreeItem = forwardRef<HTMLDivElement, ParameterTreeItemProps>(function ParameterTreeItem(props, ref) {
  const { itemId, namespacePart, paramInfo, updateParameter = (): void => {}, rosVersion = "" } = props;

  const logCtx = useContext(LoggingContext);
  const [parameterType, setParameterType] = useState<string>(paramInfo.type);
  const [changed, setChanged] = useState<boolean>(false);
  const [value, setValue] = useState(paramInfo.value);
  const [typedValue, setTypedValue] = useState<string | number | boolean | string[]>(toTypedValue(paramInfo.value));
  const [name, setName] = useState<string>("");
  const [namespace, setNamespace] = useState<string>("");
  const [showDescription, setShowDescription] = useState<boolean>(false);

  useEffect(() => {
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
      return parseFloat(value) || 0.0;
    } else if (paramInfo.type === "int") {
      return parseInt(value) | 0;
    }
    return value;
  }

  function handleKey(event: React.KeyboardEvent): void {
    if (event.key === "Enter") {
      updateParameter(paramInfo, typedValue, paramInfo.type);
      setValue(typedValue);
      setChanged(false);
    } else if (event.key === "Escape") {
      setValue(paramInfo.value);
      setChanged(false);
    }
  }

  function handleChange(event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>): void {
    setChanged(true);
    setValue(event.target.value);
  }

  function handleSliderChange(_event: Event, newValue: number | number[]): void {
    setChanged(true);
    setValue(newValue as number);
  }

  function onLeave(): void {
    if (changed) {
      updateParameter(paramInfo, typedValue, paramInfo.type);
      setValue(typedValue);
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
      } else {
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
      }
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
          checked={paramInfo.value ? true : false}
          size="small"
          disabled={paramInfo.readonly}
          onChange={(event) => {
            updateParameter(paramInfo, event.target.checked, "bool");
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
  }, [paramInfo, value, typedValue]);

  return (
    <StyledTreeItem
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
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
        >
          <Grid2 container spacing={2} sx={{ flexGrow: 1 }}>
            <Grid2 sx={{ flexGrow: 1 }} size={4}>
              <Stack direction="row" sx={{ alignItems: "center", minHeight: "2em" }}>
                <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                  {namespace}
                </Typography>{" "}
                <Typography
                  variant="body2"
                  sx={{ fontSize: "inherit", fontWeight: "bold" }}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(paramInfo.name);
                      logCtx.success(`${paramInfo.name} copied!`);
                    }
                  }}
                >
                  {name}
                </Typography>
                {!showDescription && (paramInfo.description || paramInfo.additional_constraints) && (
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
                {showDescription && (
                  <Stack sx={{ paddingLeft: "0.5em", flexGrow: 1 }} direction="column">
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
            </Grid2>
            <Grid2 size={6}>
              <Stack direction="row" sx={{ alignItems: "center" }}>
                {rosVersion === "1" && parameterType ? (
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
                  <Typography variant="caption" color="inherit" padding={0.5} minWidth="5em">
                    [{parameterType}]
                  </Typography>
                )}
                {paramInfo && renderInput}
              </Stack>
            </Grid2>
          </Grid2>
        </Box>
      }
    />
  );
});

export default ParameterTreeItem;
