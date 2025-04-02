import { Box, Stack, Typography } from "@mui/material";
import {
  TreeItem2SlotProps,
  UseTreeItem2ContentSlotOwnProps,
  UseTreeItem2IconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, LegacyRef, useContext } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import StyledTreeItem from "./StyledTreeItem";

interface ServiceGroupTreeItemProps {
  itemId: string;
  rootPath: string;
  groupName: string;
  countChildren: number;
  children: React.ReactNode;
}

const ServiceGroupTreeItem = forwardRef<HTMLDivElement, ServiceGroupTreeItemProps>(
  function ServiceGroupTreeItem(props, ref) {
    const { itemId, rootPath, groupName, countChildren, ...children } = props;

    const logCtx = useContext(LoggingContext);

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
                {/* {requestData && <CircularProgress size="1em" />} */}
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
                  <Typography variant="caption" color="inherit" padding={0}>
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
  }
);

export default ServiceGroupTreeItem;
