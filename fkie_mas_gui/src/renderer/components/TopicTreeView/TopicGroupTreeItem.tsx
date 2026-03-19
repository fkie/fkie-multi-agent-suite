import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Stack, Tooltip, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import React from "react";

import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";

interface TopicGroupTreeItemProps {
  itemId: string;
  rootPath: string;
  groupName: string;
  countChildren: number;
  hasIncompatibleQos: boolean;
  expanded: boolean;
  selected: boolean;
  depth: number;
  onToggle: () => void;
  onSelect: () => void;
}

export default function TopicGroupTreeItem({
  itemId,
  rootPath,
  groupName,
  countChildren,
  hasIncompatibleQos,
  expanded,
  selected,
  depth,
  onToggle,
  onSelect,
}: TopicGroupTreeItemProps): JSX.Element {
  const logCtx = useLoggingContext();

  const handleRowClick = (event: React.MouseEvent) => {
    event.stopPropagation();
    onToggle();
    onSelect();
  };

  const handleIconClick = (event: React.MouseEvent) => {
    event.stopPropagation();
    onToggle();
  };

  const lineKeys = Array.from({ length: depth }, (_, i) => `${itemId}-line-${i}`);

  return (
    <Box
      onClick={handleRowClick}
      sx={{
        display: "flex",
        alignItems: "stretch",
        cursor: "pointer",
        borderRadius: 0,
        bgcolor: selected ? "var(--color-select-bg)" : "transparent",
        color: "text.secondary",
      }}
    >
      {/* eine schmale Spalte mit gestrichelter Linie pro depth */}
      {lineKeys.map((key) => (
        <Box
          key={key}
          sx={{
            ml: 0.9,
            width: "0.9em",
            borderLeft: `1px dashed ${alpha(grey[600], 0.4)}`,
          }}
        />
      ))}

      {/* eigentlicher Inhalt der Row */}
      <Box
        sx={{
          display: "flex",
          alignItems: "center",
          flexGrow: 1,
          py: 0.3,
          pr: 1,
        }}
      >
        {/* Icon-Container (Expand/Collapse) */}
        <Box
          onClick={handleIconClick}
          sx={{
            width: "1em",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            mr: 0,
            ml: 0,
          }}
        >
          {expanded ? <ArrowDropDownIcon fontSize="inherit" /> : <ArrowRightIcon fontSize="inherit" />}
        </Box>

        <Stack direction="column" sx={{ flexGrow: 1 }}>
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
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
                    const value = `${rootPath}${groupName}`;
                    navigator.clipboard.writeText(value);
                    logCtx.success(`${value} copied!`, "", "topic group copied");
                    e.stopPropagation();
                  }
                }}
              >
                {groupName}
              </Typography>
              {hasIncompatibleQos && (
                <Tooltip title={"There are subscribers with incompatible QoS"} placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} sx={{ paddingLeft: "0.1em" }} />
                </Tooltip>
              )}
            </Stack>
            <Stack direction="row" spacing={1} sx={{ alignItems: "center" }}>
              {countChildren > 0 && (
                <Typography variant="caption" color="inherit" padding={0.2}>
                  [{countChildren}]
                </Typography>
              )}
            </Stack>
          </Box>
        </Stack>
      </Box>
    </Box>
  );
}
