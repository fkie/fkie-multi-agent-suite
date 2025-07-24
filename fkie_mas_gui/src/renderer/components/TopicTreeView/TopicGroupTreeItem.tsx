import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import {
  treeItemClasses,
  TreeItemSlotProps,
  UseTreeItemContentSlotOwnProps,
  UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, LegacyRef, useContext } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import StyledTreeItem from "./StyledTreeItem";

interface TopicGroupTreeItemProps {
  itemId: string;
  rootPath: string;
  groupName: string;
  countChildren: number;
  hasIncompatibleQos: boolean;
  children: React.ReactNode;
}

const TopicGroupTreeItem = forwardRef<HTMLDivElement, TopicGroupTreeItemProps>(function TopicGroupTreeItem(props, ref) {
  const { itemId, rootPath, groupName, countChildren, hasIncompatibleQos, ...children } = props;

  const logCtx = useContext(LoggingContext);

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
      sx={{
        [`& .${treeItemClasses.content}`]: {
          paddingLeft: "8px",
        },
      }}
      label={
        <Stack direction="column">
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
              // p: 0.3,
              pr: 0,
            }}
          >
            <Stack direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
              <Typography variant="body2" sx={{ fontWeight: "inherit", userSelect: "none" }}>
                {rootPath.length > 0 ? `${rootPath}/` : ""}
              </Typography>
              <Typography
                variant="body2"
                sx={{ fontWeight: "inherit", userSelect: "none" }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(`${rootPath}${groupName}`);
                    logCtx.success(`${rootPath}${groupName} copied!`);
                    e.stopPropagation();
                  }
                }}
              >
                {groupName}
              </Typography>
              {hasIncompatibleQos && (
                <Tooltip title={"There are subscribers with incompatible QoS"} placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
                </Tooltip>
              )}
              {/* {topicInfo && topicInfo.subscribers.filter((sub) => sub.incompatible_qos?.length > 0).length > 0 && (
                <Tooltip title={`There are subscribers with incompatible QoS`} placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontWeight: "inherit", color: "red" }} />
                </Tooltip>
              )} */}
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
        </Stack>
      }
      {...children}
    />
  );
});

export default TopicGroupTreeItem;
