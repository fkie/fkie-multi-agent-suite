import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import { Box, CircularProgress, Stack, SvgIconTypeMap, Tooltip, Typography } from "@mui/material";
import { OverridableComponent } from "@mui/material/OverridableComponent";
import {
    TreeItemSlotProps,
    UseTreeItemContentSlotOwnProps,
    UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, LegacyRef, useCallback, useContext } from "react";

import SettingsContext from "@/renderer/context/SettingsContext";
import { colorFromHostname } from "../UI";
import StyledTreeItem from "./StyledTreeItem";

interface ParameterGroupTreeItemProps {
  itemId: string;
  namespacePart: string;
  groupName: string;
  icon?: OverridableComponent<SvgIconTypeMap> | null;
  countChildren: number;
  requestData: boolean;
  requestError?: string;
  providerName?: string;
  children: React.ReactNode;
}

const ParameterGroupTreeItem = forwardRef<HTMLDivElement, ParameterGroupTreeItemProps>(
  function ParameterGroupTreeItem(props, ref) {
    const {
      itemId,
      namespacePart,
      groupName,
      countChildren,
      icon = null,
      requestData,
      requestError = "",
      providerName,
      ...children
    } = props;

    const settingsCtx = useContext(SettingsContext);

    const getHostStyle = useCallback(
      function getHostStyle(): object {
        if (providerName && settingsCtx.get("colorizeHosts")) {
          return {
            borderLeftStyle: "solid",
            borderLeftColor: colorFromHostname(providerName),
            borderLeftWidth: "0.6em",
          };
        }
        return {};
      },
      [providerName, settingsCtx.changed]
    );

    // avoid selection if collapse icon was clicked
    let toggled = false;
    const handleContentClick: UseTreeItemContentSlotOwnProps["onClick"] = (event) => {
      event.defaultMuiPrevented = toggled;
      toggled = false;
    };

    const handleLabelClick: UseTreeItemContentSlotOwnProps["onClick"] = () => {};

    const handleIconContainerClick: UseTreeItemIconContainerSlotOwnProps["onClick"] = () => {
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
          } as TreeItemSlotProps
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
              direction="row"
              sx={{
                flexGrow: 1,
                alignItems: "center",
              }}
            >
              {icon && <Box component={icon} color="inherit" sx={{ mr: 1 }} />}
              <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                {namespacePart.length > 0 ? `${namespacePart}.` : ""}
              </Typography>
              <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                {groupName}
              </Typography>
              {requestData && <CircularProgress size="1em" />}
              {requestError && (
                <Tooltip title={requestError} placement="bottom" disableInteractive>
                  <WarningAmberIcon color="warning" fontSize="inherit" />
                </Tooltip>
              )}
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
