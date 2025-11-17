import TurnSlightLeftIcon from "@mui/icons-material/TurnSlightLeft";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { blue } from "@mui/material/colors";
import { alpha, styled } from "@mui/material/styles";
import {
  TreeItem,
  treeItemClasses,
  TreeItemSlotProps,
  UseTreeItemContentSlotOwnProps,
  UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { useContext } from "react";
import { emitCustomEvent } from "react-custom-events";
import { FileIcon } from "react-file-icon";

import LoggingContext from "@/renderer/context/LoggingContext";
import { getFileExtension, getFileName } from "@/renderer/models";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "@/renderer/pages/NodeManager/layout/events";
import { TLaunchArg } from "@/types";
import fileIconStyles from "./FileIconStyles";
import { TLaunchIncludeItem } from "./types";

const FileTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    borderTopRightRadius: theme.spacing(2),
    borderBottomRightRadius: theme.spacing(2),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    "&.Mui-expanded": {
      fontWeight: theme.typography.fontWeightRegular,
    },
    "&:hover": {
      backgroundColor: theme.palette.action.hover,
    },
    "&.Mui-focused, &.Mui-selected, &.Mui-selected.Mui-focused": {
      backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      color: "var(--tree-view-color)",
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: "inherit",
      color: "inherit",
      padding: theme.spacing(0),
    },
    [`& .${treeItemClasses.iconContainer}`]: {
      marginLeft: 0,
      marginRight: 0,
      padding: theme.spacing(0),
      width: 10,
    },
  },
  [`& .${treeItemClasses.groupTransition}`]: {
    marginLeft: 12,
    paddingLeft: 5,
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
  },
  ...theme.applyStyles("light", {
    color: theme.palette.grey[800],
  }),
}));

interface FileTreeItemProps {
  tabId: string;
  itemId: string;
  item: TLaunchIncludeItem;
  selected: boolean;
  modified: boolean;
  children: React.ReactNode;
}

export default function FileTreeItem(props: FileTreeItemProps): JSX.Element {
  const { tabId, itemId, item, selected, modified, ...children } = props;
  const logCtx = useContext(LoggingContext);
  const fileExtension = getFileExtension(item.file.inc_path as string);

  function getLabelSx(): object {
    if (selected && modified) {
      return {
        textDecoration: "underline",
        fontWeight: 600,
        fontSize: "0.9em",
      };
    }
    if (modified) {
      return { fontWeight: 600, fontStyle: "italic" };
    }
    if (selected) {
      return {
        textDecoration: "underline",
        fontWeight: 600,
        fontSize: "0.9em",
      };
    }
    return {};
  }

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
    <FileTreeItemRoot
      itemId={itemId}
      slotProps={
        {
          label: { onClick: handleLabelClick },
          content: { onClick: handleContentClick },
          iconContainer: { onClick: handleIconContainerClick },
        } as TreeItemSlotProps
      }
      label={
        <Stack direction="column">
          <Stack spacing={0.5} direction="row" alignItems="center" marginLeft={1}>
            <Box sx={{ mr: 0.2, width: 15, color: blue[700], flex: "none" }}>
              <FileIcon extension={fileExtension} radius={10} {...fileIconStyles[fileExtension]} sx={{ mr: 1 }} />
            </Box>

            <Tooltip
              title={
                !item.file.exists
                  ? "file not found"
                  : item.file.conditional_excluded
                    ? "file conditional not loaded"
                    : ""
              }
              placement="top"
              disableInteractive
            >
              <Typography
                flexGrow={1}
                noWrap
                variant="body2"
                sx={getLabelSx()}
                onClick={(event) => {
                  emitCustomEvent(
                    EVENT_EDITOR_SELECT_RANGE,
                    eventEditorSelectRange(
                      tabId,
                      item.file.inc_path,
                      null,
                      item.file.args as TLaunchArg[]
                      // ? file.args.reduce((acc, { name, value }) => {
                      //     acc[name] = value;
                      //     return acc;
                      //   }, {})
                      // : {}
                    )
                  );
                  event.stopPropagation();
                }}
                onDoubleClick={(event) => {
                  navigator.clipboard.writeText(item.file.inc_path);
                  logCtx.success(`${item.file.inc_path} copied!`);
                  event.stopPropagation();
                }}
                color={!item.file.exists ? "red" : item.file.conditional_excluded ? "orange" : ""}
              >
                {modified && "* "}
                {`${getFileName(item.file.inc_path)}`}
              </Typography>
            </Tooltip>

            {item.file.line_number >= 0 && (
              <Tooltip title={"Show include definition in parent file"} placement="right" disableInteractive>
                <Typography
                  variant="caption"
                  color="inherit"
                  onClick={(event) => {
                    emitCustomEvent(
                      EVENT_EDITOR_SELECT_RANGE,
                      eventEditorSelectRange(
                        tabId,
                        item.file.path,
                        {
                          startLineNumber: item.file.line_number,
                          endLineNumber: item.file.line_number,
                          startColumn: 0,
                          endColumn: 0,
                        },
                        []
                      )
                    );
                    event.stopPropagation();
                  }}
                >
                  {/* [{labelLine}] */}
                  <TurnSlightLeftIcon fontSize="inherit" />
                </Typography>
              </Tooltip>
            )}
          </Stack>
        </Stack>
      }
      {...children}
    />
  );
}
