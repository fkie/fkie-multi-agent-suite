import { RosParameter } from "@/renderer/models";
import { RosParameterRange } from "@/renderer/models/RosParameter";
import { basename, normalizeNameWithPrefix } from "@/renderer/utils";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import { Box, Grid2, IconButton, Input, Slider, Stack, Switch, TextField, Typography } from "@mui/material";
import React, { forwardRef, LegacyRef, useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
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
  const [label, setLabel] = useState<string>(normalizeNameWithPrefix(basename(paramInfo.name), namespacePart));
  const [showDescription, setShowDescription] = useState<boolean>(false);

  useEffect(() => {
    setLabel(normalizeNameWithPrefix(basename(paramInfo.name), namespacePart));
  }, [namespacePart, paramInfo.name]);

  function handleKey(event: React.KeyboardEvent): void {
    if (event.key === "Enter") {
      updateParameter(paramInfo, value, paramInfo.type);
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
      updateParameter(paramInfo, value, paramInfo.type);
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

  function renderInput(): JSX.Element {
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
              value={value}
              size="small"
              onChange={(event) => handleChange(event)}
              onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
              onBlur={() => onLeave()}
              inputProps={{
                step: range.step || 1.0,
                min: range.from_value,
                max: range.to_value,
                type: "number",
                "aria-labelledby": "input-slider",
              }}
            />
            <Slider
              sx={{ minWidth: "80px", marginLeft: "1em" }}
              value={typeof value === "number" ? value : 0}
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
        <TextField
          id={`input-${paramInfo.id}`}
          value={value}
          placeholder={`${JSON.stringify(paramInfo.value)}`}
          variant="standard"
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
        <TextField
          key={paramInfo.id}
          id={`input-${paramInfo.id}`}
          value={value}
          placeholder={`${paramInfo.value}`}
          variant="standard"
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
  }

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
        >
          <Grid2 container spacing={2} sx={{ flexGrow: 1 }}>
            <Grid2 size={3}>
              <Stack direction="row" sx={{ alignItems: "center" }}>
                <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                  {namespacePart}
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
                  {label}
                </Typography>
                {!showDescription && (paramInfo.description || paramInfo.additional_constraints) && (
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
            <Grid2 size={9}>
              <Stack direction="row" sx={{ alignItems: "center" }}>
                {rosVersion === "1" && parameterType ? (
                  <OverflowMenu
                    icon={
                      <Typography variant="caption" color="inherit" padding={0.5} minWidth="4em">
                        [{parameterType}]
                      </Typography>
                    }
                    options={typeOptions}
                    id="provider-options"
                  />
                ) : (
                  <Typography variant="caption" color="inherit" padding={0.5} minWidth="4em">
                    [{parameterType}]
                  </Typography>
                )}
                {paramInfo && renderInput()}
              </Stack>
            </Grid2>
          </Grid2>
        </Box>
      }
    />
  );
});

export default ParameterTreeItem;
