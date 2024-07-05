import PropTypes from 'prop-types';
import React from 'react';

import { Box, Chip, Stack, Typography } from '@mui/material';
import { alpha, styled } from '@mui/material/styles';
import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

const SearchTreeItemRoot = styled(TreeItem)(({ theme }) => ({
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

const SearchFileTreeItem = React.forwardRef(function SearchFileTreeItem(
  {
    labelIcon = null,
    labelText = '',
    labelInfo = '',
    labelCount = 0,
    ...other
  },
  ref,
) {
  return (
    <SearchTreeItemRoot
      label={
        <Stack direction="column">
          <Stack spacing={1} direction="row" alignItems="center" marginLeft={1}>
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}

            <Typography
              flexGrow={1}
              variant="body2"
              sx={{ fontWeight: 'inherit' }}
            >
              {labelText}
            </Typography>

            {labelInfo && (
              <Typography variant="caption" color="inherit">
                [{labelInfo}]
              </Typography>
            )}
            {labelCount > 0 && (
              <Chip
                size="small"
                color="default"
                label={labelCount}
                sx={{ fontSize: '0.6em', height: 'auto', width: 'auto' }}
              />
            )}
          </Stack>
        </Stack>
      }
      style={{ marginTop: 1 }}
      {...other}
      ref={ref}
    />
  );
});

SearchFileTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  labelInfo: PropTypes.string,
  labelCount: PropTypes.number,
};

const SearchResultTreeItem = React.forwardRef(function SearchResultTreeItem(
  {
    labelIcon = null,
    labelText = '',
    labelInfo = '',
    onClick = null,
    ...other
  },
  ref,
) {
  return (
    <SearchTreeItemRoot
      label={
        <Stack direction="column">
          <Stack spacing={1} direction="row" alignItems="center">
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}

            <Typography variant="body2" sx={{ fontWeight: 'inherit' }}>
              {labelText}
            </Typography>

            {labelInfo && (
              <Typography noWrap variant="caption" color="inherit">
                {labelInfo}
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

SearchResultTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  labelInfo: PropTypes.string,
  onClick: PropTypes.func,
};

export { SearchFileTreeItem, SearchResultTreeItem };
