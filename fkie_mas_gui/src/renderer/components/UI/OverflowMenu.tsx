import MoreVertSharpIcon from "@mui/icons-material/MoreVert";
import {
  Badge,
  Fade,
  IconButton,
  IconButtonPropsSizeOverrides,
  ListItemText,
  Menu,
  MenuItem,
  SxProps,
  Theme,
  Tooltip,
} from "@mui/material";
import { styled } from "@mui/material/styles";
import { OverridableStringUnion } from "@mui/types";
import React, { useContext, useState } from "react";

import { SettingsContext } from "@/renderer/context/SettingsContext";
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
  tooltip?: string;
  name: string | React.ReactNode;
  onClick: (event?: React.MouseEvent) => void;
};

interface OverflowMenuProps {
  id: string;
  icon: React.ReactNode;
  options: OverflowMenuItem[];
  showBadge?: boolean;
  colorizeItems?: boolean;
  disabled?: boolean;
  size?: OverridableStringUnion<"small" | "medium" | "large", IconButtonPropsSizeOverrides>;
  autoClick?: boolean;
  sx?: SxProps<Theme>;
  tooltip?: string;
}

export default function OverflowMenu(props: OverflowMenuProps): JSX.Element {
  const {
    id = "overflow-menu",
    icon = <MoreVertSharpIcon sx={{ fontSize: "inherit" }} />,
    options = [],
    showBadge = false,
    colorizeItems = false,
    disabled = false,
    size = "small",
    autoClick = false,
    tooltip = "",
    sx = { padding: 0, margin: 0 },
  } = props;

  const settingsCtx = useContext(SettingsContext);

  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  function handleClick(event): void {
    if (options.length > 1) {
      setAnchorEl(event.currentTarget);
      event.stopPropagation();
    } else if (autoClick || options.length === 1) {
      options[0]?.onClick(event);
      handleClose(event);
    }
  }
  function handleClose(event): void {
    setAnchorEl(null);
    event.stopPropagation();
  }

  /** create style to colorize the menu item depends on the provider name */
  function getSxPropByName(name: string | React.ReactNode): object {
    if (colorizeItems && typeof name === "string" && (settingsCtx.get("colorizeHosts") as boolean)) {
      return {
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(name),
        borderLeftWidth: "0.6em",
      };
    }
    return {};
  }

  return (
    <StyledBadge
      color="default"
      badgeContent={`${options.length}`}
      invisible={!showBadge || options.length === 0}
      sx={{ fontSize: "inherit" }}
      onClick={(event) => event.stopPropagation()}
    >
      <Tooltip title={tooltip || ""} disableInteractive>
        <IconButton
          aria-label={`${id}-icon`}
          id={`${id}-icon`}
          size={size}
          sx={sx}
          aria-controls={open ? "long-menu" : undefined}
          aria-expanded={open ? "true" : undefined}
          aria-haspopup="true"
          disabled={disabled}
          onClick={handleClick}
          // onMouseDown={handleClick}
        >
          {icon}
        </IconButton>
      </Tooltip>
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
            <Tooltip key={option.key} title={option.tooltip || ""} disableInteractive>
              <MenuItem
                key={option.key}
                sx={getSxPropByName(option.name)}
                onClick={(event) => {
                  option.onClick(event);
                  handleClose(event);
                }}
              >
                <ListItemText>{option.name}</ListItemText>
              </MenuItem>
            </Tooltip>
          );
        })}
      </Menu>
    </StyledBadge>
  );
}
