import { alpha, styled } from '@mui/material/styles';
import PropTypes from 'prop-types';
import React from 'react';

import { Box, Stack, Typography } from '@mui/material';

import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

const FileTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    borderTopRightRadius: theme.spacing(2),
    borderBottomRightRadius: theme.spacing(2),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    '&.Mui-expanded': {
      fontWeight: theme.typography.fontWeightRegular,
    },
    '&:hover': {
      backgroundColor: theme.palette.action.hover,
    },
    '&.Mui-focused, &.Mui-selected, &.Mui-selected.Mui-focused': {
      backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      color: 'var(--tree-view-color)',
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: 'inherit',
      color: 'inherit',
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
    labelIcon,
    labelText,
    textColor,
    labelInfo,
    labelLine,
    onClick,
    onLabelClick,
    onLinenumberClick,
    ...other
  },
  ref,
) {
  return (
    <FileTreeItemRoot
      label={
        <Stack direction="column">
          <Stack
            spacing={1}
            direction="row"
            alignItems={'center'}
            marginLeft={1}
          >
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}

            <Typography
              flexGrow={1}
              noWrap
              variant="body2"
              sx={{ fontWeight: 'inherit' }}
              onClick={(event) => {
                if (onLabelClick) {
                  onLabelClick(event);
                }
              }}
              color={textColor}
            >
              {labelText}
            </Typography>

            {labelInfo && (
              <Typography noWrap flexGrow={1} variant="caption" color="inherit">
                {labelInfo}
              </Typography>
            )}
            {labelLine >= 0 && (
              <Typography
                variant="caption"
                color="inherit"
                onClick={(event) => {
                  if (onLinenumberClick) {
                    onLinenumberClick(event);
                  }
                }}
              >
                [{labelLine}]
              </Typography>
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

FileTreeItem.defaultProps = {
  labelIcon: null,
  textColor: '',
  labelText: '',
  labelInfo: '',
  labelLine: -1,
  onClick: null,
  onLabelClick: null,
  onLinenumberClick: null,
};

FileTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  textColor: PropTypes.string,
  labelInfo: PropTypes.string,
  labelLine: PropTypes.number,
  onClick: PropTypes.func,
  onLabelClick: PropTypes.func,
  onLinenumberClick: PropTypes.func,
};

export { FileTreeItem };
