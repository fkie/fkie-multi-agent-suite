import { Stack, Typography } from "@mui/material";
import { styled } from "@mui/material/styles";
import {
    TreeItem,
    treeItemClasses,
    TreeItemSlotProps,
    UseTreeItemContentSlotOwnProps,
    UseTreeItemIconContainerSlotOwnProps,
} from "@mui/x-tree-view";
import React, { forwardRef, LegacyRef } from "react";

const SearchTreeItemRoot = styled(TreeItem)(({ theme }) => ({
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
}));

interface SearchFileTreeItemProps {
  itemId: string;
  fileName: string;
  countChildren: number;
  children: React.ReactNode;
}

const SearchFileTreeItem = forwardRef<HTMLDivElement, SearchFileTreeItemProps>(function SearchFileTreeItem(props, ref) {
  const { itemId, fileName, countChildren, ...children } = props;

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
    <SearchTreeItemRoot
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
      slotProps={
        {
          label: { onClick: handleLabelClick },
          content: { onClick: handleContentClick },
          iconContainer: { onClick: handleIconContainerClick },
        } as TreeItemSlotProps
      }
      label={
        <Stack direction="column">
          <Stack spacing={1} direction="row" alignItems="center" marginLeft={1}>
            <Typography flexGrow={1} variant="body2" sx={{ fontSize: "0.8em", fontWeight: "inherit" }}>
              {fileName}
            </Typography>

            <Typography variant="caption" color="inherit">
              [{countChildren}]
            </Typography>
          </Stack>
        </Stack>
      }
      style={{ marginTop: 1 }}
      {...children}
    />
  );
});

interface SearchResultTreeItemProps {
  itemId: string;
  lineNumber: number;
  lineText: string;
  onClick: () => void;
  children?: React.ReactNode;
}

const SearchResultTreeItem = forwardRef<HTMLDivElement, SearchResultTreeItemProps>(
  function SearchResultTreeItem(props, ref) {
    const { itemId, lineNumber, lineText, onClick = (): void => {}, ...children } = props;

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
      <SearchTreeItemRoot
        itemId={itemId}
        ref={ref as LegacyRef<HTMLLIElement>}
        slotProps={
          {
            label: { onClick: handleLabelClick },
            content: { onClick: handleContentClick },
            iconContainer: { onClick: handleIconContainerClick },
          } as TreeItemSlotProps
        }
        label={
          <Stack direction="column">
            <Stack spacing={1} direction="row" alignItems="center">
              <Typography variant="body2" sx={{ fontWeight: "inherit" }}>
                {lineNumber}
              </Typography>

              <Typography noWrap variant="caption" color="inherit">
                {lineText}
              </Typography>
            </Stack>
          </Stack>
        }
        onClick={() => {
          onClick();
        }}
        {...children}
      />
    );
  }
);

export { SearchFileTreeItem, SearchResultTreeItem };
