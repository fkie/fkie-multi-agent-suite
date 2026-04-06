import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import { Box, CircularProgress, Stack, SvgIconTypeMap, Tooltip, Typography } from "@mui/material";
import { OverridableComponent } from "@mui/material/OverridableComponent";
import {
  treeItemClasses,
  TreeItemSlotProps,
  UseTreeItemContentSlotOwnProps,
  UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { useCallback, useEffect, useState } from "react";

import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import StyledTreeItem from "./StyledTreeItem";

interface ParameterGroupTreeItemProps {
  itemId: string;
  namespacePart: string;
  groupName: string;
  icon?: OverridableComponent<SvgIconTypeMap> | null;
  countChildren: number;
  requestData: boolean;
  requestError?: string;
  providerId?: string;
  children: React.ReactNode;
}

export default function ParameterGroupTreeItem(props: ParameterGroupTreeItemProps): JSX.Element {
  const {
    itemId,
    namespacePart,
    groupName,
    countChildren,
    icon = null,
    requestData,
    requestError = "",
    providerId,
    ...children
  } = props;

  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  const getHostStyle = useCallback(
    function getHostStyle(): object {
      if (providerId && colorizeHosts) {
        return {
          borderLeftStyle: "solid",
          borderLeftColor: rosCtx.providerColor(providerId),
          borderLeftWidth: "0.6em",
          [`& .${treeItemClasses.content}`]: {
            paddingLeft: "8px",
          },
        };
      }
      return {
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: "8px",
        },
      };
    },
    [rosCtx.providerColor, providerId, colorizeHosts]
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
