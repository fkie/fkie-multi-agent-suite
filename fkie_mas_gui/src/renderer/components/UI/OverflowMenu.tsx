import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import { Badge, Fade, IconButton, ListItemText, Menu, MenuItem } from "@mui/material";
import { styled } from "@mui/material/styles";
import React, { forwardRef, useContext, useState } from "react";
import { SettingsContext } from "../../context/SettingsContext";
import { colorFromHostname } from "./Colors";

const StyledBadge = styled(Badge)((/*{ theme }*/) => ({
  "& .MuiBadge-badge": {
    right: 0,
    top: 22,
    // border: `2px solid ${theme.palette.background.paper}`,
    // padding: '2px 4px',
    fontSize: "inherit",
    height: 12,
    minWidth: 10,
  },
}));

export type OverflowMenuItem = {
  key: string;
  name: string | React.ReactNode;
  onClick: () => void;
};

interface OverflowMenuProps {
  id: string;
  icon: React.ReactNode;
  options: OverflowMenuItem[];
  showBadge?: boolean;
  colorizeItems?: boolean;
}

const OverflowMenu = forwardRef<HTMLDivElement, OverflowMenuProps>(function OverflowMenu(props, ref) {
  const {
    id = "overflow-menu",
    icon = <MoreVertSharpIcon sx={{ fontSize: "inherit" }} />,
    options = [],
    showBadge = false,
    colorizeItems = false,
  } = props;

  const settingsCtx = useContext(SettingsContext);

  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
    event.stopPropagation();
  };
  const handleClose = (event) => {
    setAnchorEl(null);
    event.stopPropagation();
  };

  /** create style to colorize the menu item depends on the provider name */
  const getSxPropByName = (name: string | React.ReactNode) => {
    if (colorizeItems && typeof name === "string" && (settingsCtx.get("colorizeHosts") as boolean)) {
      return {
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(name),
        borderLeftWidth: "0.6em",
      };
    }
    return {};
  };

  return (
    <StyledBadge
      ref={ref}
      color="default"
      badgeContent={`${options.length}`}
      invisible={!showBadge || options.length === 0}
      sx={{ fontSize: "inherit" }}
      onClick={(event) => event.stopPropagation()}
    >
      <IconButton
        aria-label={`${id}-icon`}
        id={`${id}-icon`}
        size="small"
        sx={{ padding: 0, margin: 0 }}
        aria-controls={open ? "long-menu" : undefined}
        aria-expanded={open ? "true" : undefined}
        aria-haspopup="true"
        onClick={handleClick}
        onMouseDown={handleClick}
      >
        {icon}
      </IconButton>
      <Menu
        id={`${id}-menu`}
        // size="small"
        MenuListProps={{
          "aria-labelledby": "long-button",
        }}
        anchorEl={anchorEl}
        open={open}
        onClose={(event) => handleClose(event)}
        TransitionComponent={Fade}
      >
        {options.map((option: OverflowMenuItem) => {
          return (
            <MenuItem
              key={option.key}
              sx={getSxPropByName(option.name)}
              onClick={(event) => {
                option.onClick();
                handleClose(event);
              }}
            >
              <ListItemText>{option.name}</ListItemText>
            </MenuItem>
          );
        })}
      </Menu>
    </StyledBadge>
  );
});

export default OverflowMenu;
