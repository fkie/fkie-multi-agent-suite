import { Box, Menu, MenuItem, Stack, Typography } from "@mui/material";
import { grey } from "@mui/material/colors";
import { alpha } from "@mui/material/styles";
import { useCallback, useEffect, useState } from "react";

import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useNavigationContext } from "@/renderer/hooks/useNavigationContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { ServiceExtendedInfo, TServiceNodeInfo } from "@/renderer/models";

interface ServiceTreeItemProps {
  itemId: string;
  rootPath: string;
  serviceInfo: ServiceExtendedInfo;
  selectedItem: string; // ID of last selected service (for soft highlight)
  selected: boolean; // true only in the domain where this item is actively selected
  depth: number;
  onSelect: () => void;
}

/**
 * Virtualized row for a single service.
 * Mirrors TopicTreeItem behavior:
 * - single click selects
 * - second click toggles extended info for the selected row in this domain
 * - soft highlight if selected in another domain

 */
export default function ServiceTreeItem({
  itemId,
  rootPath,
  serviceInfo,
  selectedItem,
  selected,
  depth,
  onSelect,
}: ServiceTreeItemProps): JSX.Element {
  const logCtx = useLoggingContext();
  const navCtx = useNavigationContext();
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();

  const [name, setName] = useState<string>("");
  const [namespace, setNamespace] = useState<string>("");
  const [showExtendedInfo, setShowExtendedInfo] = useState<boolean>(false);
  const [ignoreNextClick, setIgnoreNextClick] = useState<boolean>(true);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  // update colorize setting when context value changes
  useEffect(() => {
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed, settingsCtx]);

  /**
   * Reset local click/extended-info state when this item is no longer selected
   * in its domain (selection moved to another item or another domain).
   */
  useEffect(() => {
    if (!selected) {
      setIgnoreNextClick(true);
    }
  }, [selected]);

  // parse service name and namespace prefix
  useEffect(() => {
    const nameParts = serviceInfo.name.split("/");
    setName(`${nameParts.pop()}`);
    setNamespace(rootPath ? `${rootPath}/` : "");
  }, [serviceInfo.name, rootPath]);

  const getHostStyle = useCallback(
    (providerId: string): object => {
      if (providerId && colorizeHosts) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: rosCtx.providerColor(providerId),
          borderLeftWidth: "0.6em",
        };
      }
      return { flexGrow: 1, alignItems: "center", paddingLeft: 0 };
    },
    [colorizeHosts, rosCtx]
  );

  const handleContextMenu = useCallback(
    (event: React.MouseEvent<HTMLDivElement>) => {
      event.preventDefault();
      event.stopPropagation();
      setContextMenu(
        contextMenu === null
          ? {
              mouseX: event.clientX + 2,
              mouseY: event.clientY - 6,
            }
          : null
      );
    },
    [contextMenu]
  );

  const handleCloseMenu = useCallback((event: React.SyntheticEvent) => {
    setContextMenu(null);
    event.stopPropagation();
  }, []);

  const handleDoubleClickCopy = useCallback(
    (e: React.MouseEvent<HTMLSpanElement>, value: string, label: string) => {
      if (e.detail === 2) {
        navigator.clipboard.writeText(value);
        logCtx.info(`${value} copied!`, "", `${label} copied`);
        e.stopPropagation();
      }
    },
    [logCtx]
  );

  /**
   * Click behavior:
   * - If not selected in this domain: select item and reset extended info.
   * - If already selected:
   *   - first click only arms the next click (ignoreNextClick -> false)
   *   - second click toggles extended info.

   */
  const handleRowClick = (e: React.MouseEvent<HTMLDivElement>) => {
    e.stopPropagation();

    if (ignoreNextClick) {
      setIgnoreNextClick(false);
    } else {
      setShowExtendedInfo((prev) => !prev);
    }

    onSelect();
  };

  const hardSelected = selected;
  const softSelected = !selected && selectedItem === itemId;

  const lineKeys = Array.from({ length: depth }, (_, i) => `${itemId}-line-${i}`);

  return (
    <Box
      sx={(theme) => ({
        display: "flex",
        alignItems: "stretch",
        cursor: "pointer",
        borderRadius: 0,
        bgcolor: hardSelected
          ? alpha(theme.palette.primary.main, 0.18)
          : softSelected
            ? alpha(theme.palette.primary.main, 0.06)
            : "transparent",
        color: "text.secondary",
      })}
      onClick={handleRowClick}
      onContextMenu={handleContextMenu}
    >
      {/* Indentation lines for depth-level visualization */}
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

      {/* Content column (header + extended info + context menu) */}
      <Box sx={{ flexGrow: 1, py: 0.2, pr: 1 }}>
        {/* Header row */}
        <Box
          sx={{
            ml: 0.7,
            display: "flex",
            alignItems: "center",
            py: 0.2,
            pr: 0,
          }}
          onClick={handleRowClick}
        >
          <Stack spacing={1} direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
            <Stack direction="row" sx={getHostStyle(serviceInfo.nodeProviders[0]?.providerId)}>
              <Typography
                variant="body2"
                sx={{ fontSize: "inherit", userSelect: "none" }}
                onClick={(e) => handleDoubleClickCopy(e, serviceInfo.name, "service name")}
              >
                {namespace}
              </Typography>
              <Typography
                variant="body2"
                sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}
                onClick={(e) => handleDoubleClickCopy(e, serviceInfo.name, "service name")}
              >
                {name}
              </Typography>
            </Stack>
          </Stack>

          <Stack direction="row" spacing={1} sx={{ alignItems: "center" }}>
            {serviceInfo.srvType && (
              <Typography
                variant="caption"
                color="inherit"
                padding={0.2}
                onClick={(e) => handleDoubleClickCopy(e, serviceInfo.srvType, "service type")}
              >
                {serviceInfo.srvType}
              </Typography>
            )}
          </Stack>
        </Box>

        {/* Extended info – remains visible until service is clicked again in this domain */}
        {showExtendedInfo && serviceInfo && (
          <Stack paddingLeft={3}>
            <Typography fontWeight="bold" fontSize="small">
              Provider [{serviceInfo.nodeProviders.length}]:
            </Typography>
            {serviceInfo.nodeProviders.map((item: TServiceNodeInfo) => (
              <Stack
                key={`${item.providerId}-${item.nodeId}`}
                paddingLeft={3}
                direction="row"
                sx={getHostStyle(item.providerId)}
              >
                <Typography
                  fontSize="small"
                  onClick={(event) => {
                    event.stopPropagation();
                    const id = `${item.providerId}${item.nodeId.replaceAll("/", "#")}`;
                    navCtx.setSelected("service-tree", [id], false);
                  }}
                >
                  {item.nodeName}
                </Typography>
              </Stack>
            ))}

            {serviceInfo.nodeRequester.length > 0 && (
              <>
                <Typography fontWeight="bold" fontSize="small">
                  Requester [{serviceInfo.nodeRequester.length}]:
                </Typography>
                {serviceInfo.nodeRequester.map((item: TServiceNodeInfo) => (
                  <Stack
                    key={`${item.providerId}-${item.nodeId}`}
                    paddingLeft={3}
                    direction="row"
                    sx={getHostStyle(item.providerId)}
                  >
                    <Typography
                      fontSize="small"
                      onClick={(event) => {
                        event.stopPropagation();
                        const id = `${item.providerId}${item.nodeId.replaceAll("/", "#")}`;
                        navCtx.setSelected("service-tree", [id], false);
                      }}
                    >
                      {item.nodeName}
                    </Typography>
                  </Stack>
                ))}
              </>
            )}
          </Stack>
        )}

        {/* Context menu for copy actions */}
        <Menu
          open={contextMenu != null}
          onClose={handleCloseMenu}
          anchorReference="anchorPosition"
          anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
        >
          <MenuItem
            sx={{ fontSize: "0.8em" }}
            onClick={(event) => {
              navigator.clipboard.writeText(serviceInfo.name);
              handleCloseMenu(event);
            }}
          >
            Copy service name
          </MenuItem>
          <MenuItem
            sx={{ fontSize: "0.8em" }}
            onClick={(event) => {
              navigator.clipboard.writeText(serviceInfo.srvType);
              handleCloseMenu(event);
            }}
          >
            Copy service type
          </MenuItem>
        </Menu>
      </Box>
    </Box>
  );
}
