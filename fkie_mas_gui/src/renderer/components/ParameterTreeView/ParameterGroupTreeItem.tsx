import { Box, CircularProgress, Stack, SvgIconTypeMap, Typography } from "@mui/material";
import {
  TreeItem2SlotProps,
  UseTreeItem2ContentSlotOwnProps,
  UseTreeItem2IconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, LegacyRef, useContext } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import StyledTreeItem from "./StyledTreeItem";
import { OverridableComponent } from "@mui/material/OverridableComponent";
import SettingsContext from "@/renderer/context/SettingsContext";
import { colorFromHostname } from "../UI";

interface ParameterGroupTreeItemProps {
  itemId: string;
  rootPath: string;
  groupName: string;
  icon?: OverridableComponent<SvgIconTypeMap> | null;
  countChildren: number;
  requestData: boolean;
  providerName?: string;
  children: React.ReactNode;
}

const ParameterGroupTreeItem = forwardRef<HTMLDivElement, ParameterGroupTreeItemProps>(
  function ParameterGroupTreeItem(props, ref) {
    const { itemId, rootPath, groupName, countChildren, icon = null, requestData, providerName, ...children } = props;

    const logCtx = useContext(LoggingContext);
    const settingsCtx = useContext(SettingsContext);

    const getHostStyle = () => {
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(providerName),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    };

    // avoid selection if collapse icon was clicked
    let toggled = false;
    const handleContentClick: UseTreeItem2ContentSlotOwnProps["onClick"] = (event) => {
      event.defaultMuiPrevented = toggled;
      toggled = false;
    };

    const handleLabelClick: UseTreeItem2ContentSlotOwnProps["onClick"] = () => {};

    const handleIconContainerClick: UseTreeItem2IconContainerSlotOwnProps["onClick"] = () => {
      toggled = true;
    };
    return (
      <StyledTreeItem
        itemId={itemId}
        ref={ref as LegacyRef<HTMLLIElement>}
        slotProps={
          {
            label: { onClick: handleLabelClick },
            content: { onClick: handleContentClick },
            iconContainer: { onClick: handleIconContainerClick },
          } as TreeItem2SlotProps
        }
        sx={getHostStyle()}
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
              spacing={1}
              direction="row"
              sx={{
                flexGrow: 1,
                alignItems: "center",
              }}
            >
              {icon && <Box component={icon} color="inherit" sx={{ mr: 1 }} />}
              {/* <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                {rootPath}
              </Typography> */}
              <Typography
                variant="body2"
                sx={{ fontWeight: "inherit" }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(groupName);
                    logCtx.success(`${groupName} copied!`);
                    e.stopPropagation();
                  }
                }}
              >
                {groupName.startsWith("/") ? groupName.slice(1) : groupName}
              </Typography>
              {requestData && <CircularProgress size="1em" />}
            </Stack>
            <Stack
              direction="row"
              spacing={1}
              sx={{
                alignItems: "center",
              }}
            >
              {countChildren > 0 && (
                // <Tag text={countChildren} color="default" copyButton={false}></Tag>
                <Typography variant="caption" color="inherit" padding={0.2}>
                  [{countChildren}]
                </Typography>
              )}
            </Stack>
          </Box>
        }
        {...children}
      />
    );
  }
);

export default ParameterGroupTreeItem;
