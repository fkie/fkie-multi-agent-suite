import { Box, Tooltip, Typography } from '@mui/material';
import { grey } from '@mui/material/colors';
import { alpha, styled } from '@mui/material/styles';
import { TreeItem, treeItemClasses } from '@mui/x-tree-view';
import PropTypes from 'prop-types';

import ContentComponentItemTree from '../ContentComponentItemTree/ContentComponentItemTree';
import CopyButton from '../UI/CopyButton';
import OverflowMenu from '../UI/OverflowMenu';

const StyledTreeItemRoot = styled((props) => <TreeItem {...props} />)(
  ({ theme }) => ({
    color: theme.palette.text.secondary,
    [`& .${treeItemClasses.content}`]: {
      color: theme.palette.text.secondary,
      minHeight: 25,
      borderRadius: theme.spacing(0.9),
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
      borderColor: grey[500],
    },
  }),
);

function PackageItemTree({
  bgColor,
  color,
  labelText,
  paddingLeft,
  itemId = '',
  iconColor = '',
  tooltip = '',
  enableCopy = true,
  onClick = () => {},
  onDoubleClick = () => {},
  menuItems = [],
  labelIcon: LabelIcon = null,
  labelIconComponent = null,
  ...other
}) {
  return (
    <StyledTreeItemRoot
      ContentComponent={ContentComponentItemTree}
      itemId={itemId}
      label={
        <Box
          sx={{
            display: 'flex',
            alignItems: 'center',
            paddingLeft,
          }}
          onClick={() => {
            if (onClick) onClick(labelText, itemId);
          }}
          onDoubleClick={(event, ok) => {
            if (onDoubleClick)
              onDoubleClick(
                labelText,
                itemId,
                event.ctrlKey,
                event.shiftKey,
                event.altKey,
              );
          }}
        >
          {menuItems && menuItems.length > 0 && (
            <OverflowMenu options={menuItems} id={`${itemId}-options"`} />
          )}
          {LabelIcon && (
            <LabelIcon
              sx={{
                mr: 0.2,
                width: 20,
                color: iconColor,
              }}
              style={{ fontSize: 'inherit' }}
            />
          )}

          {labelIconComponent && (
            <Box sx={{ mr: 0.2, width: 15, color: iconColor, flex: 'none' }}>
              {labelIconComponent}
            </Box>
          )}

          <Tooltip title={tooltip} enterDelay={1000} enterNextDelay={1000}>
            <Typography
              // noWrap
              variant="body2"
              sx={{ fontWeight: 'inherit', flexGrow: 1, ml: 0.5 }}
            >
              {labelText}
            </Typography>
          </Tooltip>
          {tooltip && enableCopy && <CopyButton value={tooltip} />}
        </Box>
      }
      style={{
        '--tree-view-color': color,
        '--tree-view-bg-color': bgColor,
      }}
      {...other}
    />
  );
}

PackageItemTree.propTypes = {
  itemId: PropTypes.string,
  iconColor: PropTypes.string,
  bgColor: PropTypes.string.isRequired,
  color: PropTypes.string.isRequired,
  labelIcon: PropTypes.elementType,
  labelIconComponent: PropTypes.any,
  labelText: PropTypes.string.isRequired,
  tooltip: PropTypes.string,
  enableCopy: PropTypes.bool,
  paddingLeft: PropTypes.number.isRequired,
  onClick: PropTypes.func,
  onDoubleClick: PropTypes.func,
  menuItems: PropTypes.arrayOf(PropTypes.any),
};

export default PackageItemTree;
