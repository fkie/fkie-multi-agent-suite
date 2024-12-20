import TurnSlightLeftIcon from "@mui/icons-material/TurnSlightLeft";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { alpha, styled } from "@mui/material/styles";
import { TreeItem, treeItemClasses } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import React from "react";

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
  [`& .${treeItemClasses.group}`]: {
    marginLeft: 12,
    paddingLeft: 5,
    // [`& .${treeItemClasses.content}`]: {
    //   paddingLeft: theme.spacing(0),
    // },
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
    // borderColor: black[50],
  },
}));

const FileTreeItem = React.forwardRef(function FileTreeItem(
  {
    labelIcon = null,
    textColor = "",
    labelText = "",
    labelInfo = "",
    toolTip = "",
    labelLine = -1,
    modified = false,
    selected = false,
    onClick = null,
    onLabelClick = null,
    onLabelDoubleClick = null,
    onLinenumberClick = null,
    ...other
  },
  ref
) {
  const getLabelSx = () => {
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
  };

  return (
    <FileTreeItemRoot
      label={
        <Stack direction="column">
          <Stack spacing={1} direction="row" alignItems="center" marginLeft={1}>
            {labelIcon && <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />}

            <Tooltip title={toolTip} placement="top" disableInteractive>
              <Typography
                flexGrow={1}
                noWrap
                variant="body2"
                sx={getLabelSx()}
                onClick={(event) => {
                  if (onLabelClick) {
                    onLabelClick(event);
                  }
                }}
                onDoubleClick={(event) => {
                  if (onLabelDoubleClick) {
                    onLabelDoubleClick(event);
                  }
                }}
                color={textColor}
              >
                {modified && "* "}
                {labelText}
              </Typography>
            </Tooltip>

            {labelInfo && (
              <Typography noWrap flexGrow={1} variant="caption" color="inherit">
                {labelInfo}
              </Typography>
            )}
            {labelLine >= 0 && (
              <Tooltip title={"Show include definition in parent file"} placement="right" disableInteractive>
                <Typography
                  variant="caption"
                  color="inherit"
                  onClick={(event) => {
                    if (onLinenumberClick) {
                      onLinenumberClick(event);
                    }
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
      onClick={(event) => {
        if (onClick) {
          onClick(event);
        }
      }}
      {...other}
      ref={ref}
    />
  );
});

FileTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  textColor: PropTypes.string,
  labelInfo: PropTypes.string,
  toolTip: PropTypes.string,
  labelLine: PropTypes.number,
  modified: PropTypes.bool,
  selected: PropTypes.bool,
  onClick: PropTypes.func,
  onLabelClick: PropTypes.func,
  onLabelDoubleClick: PropTypes.func,
  onLinenumberClick: PropTypes.func,
};

export { FileTreeItem };
