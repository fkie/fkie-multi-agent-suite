import PropTypes from 'prop-types';
import { useContext, useState } from 'react';

import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import { Badge, Fade, IconButton, Menu, MenuItem } from '@mui/material';
import { styled } from '@mui/material/styles';
import { SettingsContext } from '../../context/SettingsContext';
import { colorFromHostname } from './Colors';

const StyledBadge = styled(Badge)(({ theme }) => ({
  '& .MuiBadge-badge': {
    right: 0,
    top: 22,
    // border: `2px solid ${theme.palette.background.paper}`,
    // padding: '2px 4px',
    fontSize: 'inherit',
    height: 12,
    minWidth: 10,
  },
}));

function OverflowMenu({ icon, options, id, showBadge, colorizeItems }) {
  const settingsCtx = useContext(SettingsContext);

  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  /** create style to colorize the menu item depends on the provider name */
  const getSxPropByName = (name) => {
    if (colorizeItems && settingsCtx.get('colorizeHosts')) {
      return {
        borderLeftStyle: 'outset',
        borderLeftColor: colorFromHostname(name),
        borderLeftWidth: '0.6em',
      };
    }
    return { };
  };

  return (
    <StyledBadge
      color="default"
      badgeContent={`${options.length}`}
      invisible={!showBadge || options.length === 0}
      // variant="dot"
      // anchorOrigin={{
      //   vertical: 'bottom',
      //   horizontal: 'right',
      // }}
    >
      <IconButton
        aria-label={`${id}-icon`}
        id={`${id}-icon`}
        size="small"
        aria-controls={open ? 'long-menu' : undefined}
        aria-expanded={open ? 'true' : undefined}
        aria-haspopup="true"
        onClick={handleClick}
        onMouseDown={handleClick}
      >
        {icon}
      </IconButton>
      <Menu
        id={`${id}-menu`}
        size="small"
        MenuListProps={{
          'aria-labelledby': 'long-button',
        }}
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        TransitionComponent={Fade}
        // PaperProps={{
        //   style: {
        //     maxHeight: 10 * 4.5,
        //     width: '20ch',
        //   },
        // }}
      >
        {options.map((option) => {
          return (
            <MenuItem
              size="small"
              key={option.key}
              onClick={() => {
                option.onClick();
                handleClose();
              }}
              sx={getSxPropByName(option.name)}
            >
              {option.name}
            </MenuItem>
          );
        })}
      </Menu>
    </StyledBadge>
  );
}

OverflowMenu.defaultProps = {
  icon: <MoreVertSharpIcon sx={{ fontSize: 'inherit' }} />,
  options: [],
  id: 'overflow-menu',
  showBadge: false,
  colorizeItems: false,
};

OverflowMenu.propTypes = {
  icon: PropTypes.object,
  options: PropTypes.array,
  id: PropTypes.string,
  showBadge: PropTypes.bool,
  colorizeItems: PropTypes.bool,
};

export default OverflowMenu;
