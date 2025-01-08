import { Box, Stack, Switch, TextField, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import OverflowMenu from "../UI/OverflowMenu";
import { RosParameter } from "@/renderer/models";
import StyledTreeItem from "./StyledTreeItem";
import { basename, normalizeNameWithPrefix } from "@/renderer/utils";

interface ParameterTreeItemProps {
  itemId: string;
  namespacePart: string;
  paramInfo: RosParameter;
  updateParameter: (param: RosParameter, value: string | boolean | number | string[], valueType?: string) => void;
  rosVersion?: string;
}

const ParameterTreeItem = forwardRef<HTMLDivElement, ParameterTreeItemProps>(function ParameterTreeItem(props, ref) {
  const { itemId, namespacePart, paramInfo, updateParameter = () => {}, rosVersion = "" } = props;

  const logCtx = useContext(LoggingContext);
  const [parameterType, setParameterType] = useState<string>(paramInfo.type);
  const [label, setLabel] = useState<string>(normalizeNameWithPrefix(basename(paramInfo.name), namespacePart));

  useEffect(() => {
    setLabel(normalizeNameWithPrefix(basename(paramInfo.name), namespacePart));
  }, [namespacePart, paramInfo.name]);

  const typeOptions = [
    {
      name: "int",
      key: "int",
      onClick: () => {
        setParameterType("int");
        updateParameter(paramInfo, paramInfo.value, "int");
      },
    },
    {
      name: "float",
      key: "float",
      onClick: () => {
        setParameterType("float");
        updateParameter(paramInfo, paramInfo.value, "float");
      },
    },
    {
      name: "str",
      key: "str",
      onClick: () => {
        setParameterType("str");
        updateParameter(paramInfo, paramInfo.value, "str");
      },
    },
    {
      name: "bool",
      key: "bool",
      onClick: () => {
        setParameterType("bool");
        updateParameter(paramInfo, paramInfo.value, "bool");
      },
    },
  ];

  const renderInput = () => {
    if (["int", "float"].includes(parameterType)) {
      return (
        <TextField
          type="number"
          id={`input-${paramInfo.id}`}
          size="small"
          variant="standard"
          defaultValue={paramInfo.value}
          placeholder={JSON.stringify(paramInfo.value)}
          // InputProps={{ inputProps: { min: 0, max: 99 } }}
          fullWidth
          onChange={(event) => {
            updateParameter(paramInfo, event.target.value);
          }}
        />
      );
    }
    if (["list"].includes(parameterType)) {
      // TODO: show proper list/arrays
      return (
        <TextField
          id={`input-${paramInfo.id}`}
          defaultValue={`${JSON.stringify(paramInfo.value)}`}
          placeholder={`${JSON.stringify(paramInfo.value)}`}
          variant="standard"
          size="small"
          fullWidth
          onChange={(event) => {
            updateParameter(paramInfo, event.target.value);
          }}
        />
      );
    }
    if (["bool"].includes(parameterType)) {
      return (
        <Switch
          id={`input-${paramInfo.id}`}
          checked={paramInfo.value ? true : false}
          size="small"
          onChange={(event) => {
            updateParameter(paramInfo, event.target.checked, "bool");
          }}
        />
      );
    }

    // default render (usually string)
    return (
      <TextField
        key={paramInfo.id}
        id={`input-${paramInfo.id}`}
        defaultValue={`${paramInfo.value}`}
        placeholder={`${paramInfo.value}`}
        variant="standard"
        size="small"
        fullWidth
        onChange={(event) => {
          updateParameter(paramInfo, event.target.value, "string");
        }}
      />
    );
  };

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
