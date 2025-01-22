import { RosParameter } from "@/renderer/models";
import { basename, normalizeNameWithPrefix } from "@/renderer/utils";
import { Box, Stack, Switch, TextField, Typography } from "@mui/material";
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

  useEffect(() => {
    setLabel(normalizeNameWithPrefix(basename(paramInfo.name), namespacePart));
  }, [namespacePart, paramInfo.name]);

  function handleKey(event: React.KeyboardEvent): void {
    if (event.key === "Enter") {
      updateParameter(paramInfo, value);
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

  function onLeave(): void {
    if (changed) {
      updateParameter(paramInfo, value);
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
  ];

  function renderInput(): JSX.Element {
    let inputElement: JSX.Element | null = null;
    if (["int", "float"].includes(parameterType)) {
      inputElement = (
        <TextField
          type="number"
          id={`input-${paramInfo.id}`}
          size="small"
          variant="standard"
          value={value}
          placeholder={JSON.stringify(value)}
          // InputProps={{ inputProps: { min: 0, max: 99 } }}
          fullWidth
          onChange={(event) => handleChange(event)}
          onKeyUp={(event: React.KeyboardEvent) => handleKey(event)}
          onBlur={() => onLeave()}
        />
      );
    } else if (["list"].includes(parameterType)) {
      // TODO: show proper list/arrays
      inputElement = (
        <TextField
          id={`input-${paramInfo.id}`}
          value={value}
          placeholder={`${JSON.stringify(paramInfo.value)}`}
          variant="standard"
          size="small"
          fullWidth
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
          <Stack
            direction="row"
            sx={{
              flexGrow: 1,
              alignItems: "center",
            }}
          >
            <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
              {namespacePart}
            </Typography>{" "}
            <Typography
              variant="body2"
              sx={{ fontWeight: "inherit" }}
              onClick={(e) => {
                if (e.detail === 2) {
                  navigator.clipboard.writeText(paramInfo.name);
                  logCtx.success(`${paramInfo.name} copied!`);
                }
              }}
            >
              {label}
            </Typography>
          </Stack>
          <Stack
            direction="row"
            spacing={1}
            sx={{
              alignItems: "end",
            }}
          >
            {rosVersion === "1" && parameterType ? (
              <OverflowMenu
                icon={
                  <Typography variant="caption" color="inherit" padding={0.5}>
                    [{parameterType}]
                  </Typography>
                }
                options={typeOptions}
                id="provider-options"
              />
            ) : (
              <Typography variant="caption" color="inherit" padding={0.5}>
                [{parameterType}]
              </Typography>
            )}
            {paramInfo && renderInput()}
          </Stack>
        </Box>
      }
    />
  );
});

export default ParameterTreeItem;
