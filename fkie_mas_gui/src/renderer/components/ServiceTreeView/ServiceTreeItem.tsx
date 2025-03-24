import { Box, Menu, MenuItem, Stack, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useState } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";
import { NavigationContext } from "@/renderer/context/NavigationContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { ServiceExtendedInfo, TServiceNodeInfo } from "@/renderer/models";
import { colorFromHostname } from "../UI/Colors";
import StyledTreeItem from "./StyledTreeItem";

interface ServiceTreeItemProps {
  itemId: string;
  rootPath: string;
  serviceInfo: ServiceExtendedInfo;
  selectedItem: string | null;
}

const ServiceTreeItem = forwardRef<HTMLDivElement, ServiceTreeItemProps>(function ServiceTreeItem(props, ref) {
  const { itemId, rootPath, serviceInfo, selectedItem } = props;

  // color = "#1a73e8",
  // bgColor = "#e8f0fe",
  // colorForDarkMode = "#B8E7FB",
  // bgColorForDarkMode = "#071318",
  // labelRoot = "",
  // labelIcon = null,
  // labelInfo = "",
  // labelCount = null,
  // labelText = "",
  // serviceInfo = null,
  // selectedItem = "",
  // ...other
  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const [name, setName] = useState<string>("");
  const [namespace, setNamespace] = useState<string>("");
  // state variables to show/hide extended info
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selected, setSelected] = useState(false);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);
  const [contextMenu, setContextMenu] = useState<{ mouseX: number; mouseY: number } | null>(null);

  const getHostStyle = useCallback(
    function getHostStyle(): object {
      if (serviceInfo.providerName && settingsCtx.get("colorizeHosts")) {
        return {
          flexGrow: 1,
          alignItems: "center",
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(serviceInfo.providerName),
          borderLeftWidth: "0.6em",
        };
      }
      return { flexGrow: 1, alignItems: "center" };
    },
    [serviceInfo.providerName, settingsCtx.changed]
  );

  useEffect(() => {
    if (!serviceInfo) return;
    const nameParts = serviceInfo.name.split("/");
    setName(`${nameParts.pop()}`);
    setNamespace(rootPath ? `${rootPath}/` : rootPath ? "" : "");
  }, [rootPath, serviceInfo.name, serviceInfo]);

  useEffect(() => {
    // update state variables to show/hide extended info
    if (selectedItem !== itemId) {
      if (selected) {
        setSelected(false);
        setIgnoreNextClick(true);
      }
    } else {
      if (selected) {
        setShowExtendedInfo(!showExtendedInfo);
      }
      setSelected(true);
    }
  }, [selectedItem]);

  return (
    <StyledTreeItem
      itemId={itemId}
      ref={ref as LegacyRef<HTMLLIElement>}
      label={
        <Stack
          direction="column"
          onContextMenu={(event) => {
            event.preventDefault();
            setContextMenu(
              contextMenu === null
                ? {
                    mouseX: event.clientX + 2,
                    mouseY: event.clientY - 6,
                  }
                : null
            );
          }}
        >
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
              // p: 0.3,
              pr: 0,
            }}
            onClick={() => {
              if (ignoreNextClick) {
                setIgnoreNextClick(false);
              } else {
                setShowExtendedInfo(!showExtendedInfo);
              }
            }}
          >
            <Stack spacing={1} direction="row" sx={getHostStyle()}>
              <Stack direction="row" sx={{ flexGrow: 1, alignItems: "center" }}>
                <Typography
                  variant="body2"
                  sx={{ fontSize: "inherit", userSelect: "none" }}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(serviceInfo.name);
                      logCtx.success(`${serviceInfo.name} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {namespace}
                </Typography>
                <Typography
                  variant="body2"
                  sx={{ fontSize: "inherit", fontWeight: "bold", userSelect: "none" }}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(serviceInfo.name);
                      logCtx.success(`${serviceInfo.name} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {name}
                </Typography>
              </Stack>
            </Stack>
            <Stack
              direction="row"
              spacing={1}
              sx={{
                alignItems: "center",
              }}
            >
              {serviceInfo.srvType && (
                <Typography
                  variant="caption"
                  color="inherit"
                  padding={0.2}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(serviceInfo.srvType);
                      logCtx.success(`${serviceInfo.srvType} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {serviceInfo.srvType}
                </Typography>
              )}
              {/* {serviceInfo && (
                <Stack direction="row" spacing={1}>
                  <Chip
                    size="small"
                    title="publishers"
                    // showZero={true}
                    color={serviceInfo.publishers.length > 0 ? "default" : "warning"}
                    label={serviceInfo.publishers.length}
                  />
                </Stack>
              )} */}
            </Stack>
          </Box>
          {showExtendedInfo && serviceInfo && (
            <Stack paddingLeft={3}>
              <Typography fontWeight="bold" fontSize="small">
                Provider [{serviceInfo.nodeProviders.length}]:
              </Typography>
              {serviceInfo.nodeProviders.map((item: TServiceNodeInfo) => {
                return (
                  <Stack key={item.nodeId} paddingLeft={1} direction="row">
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        navCtx.setSelectedNodes([`${serviceInfo.providerId}${item.nodeId.replaceAll("/", "#")}`]);
                      }}
                    >
                      {item.nodeName}
                    </Typography>
                    {/* <CopyButton value={item.nodeName} fontSize="0.7em" /> */}
                  </Stack>
                );
              })}
              {serviceInfo.nodeRequester.length > 0 && (
                <Stack>
                  <Typography fontWeight="bold" fontSize="small">
                    Requester [{serviceInfo.nodeRequester.length}]:
                  </Typography>
                  {serviceInfo.nodeRequester.map((item) => {
                    return (
                      <Stack key={item.nodeId} paddingLeft={1} direction="row">
                        <Typography
                          fontSize="small"
                          onClick={() => {
                            navCtx.setSelectedNodes([`${serviceInfo.providerId}${item.nodeId.replaceAll("/", "#")}`]);
                          }}
                        >
                          {item.nodeName}
                        </Typography>
                        {/* <CopyButton value={item.nodeName} fontSize="0.7em" /> */}
                      </Stack>
                    );
                  })}
                </Stack>
              )}
            </Stack>
          )}
          <Menu
            open={contextMenu != null}
            onClose={() => setContextMenu(null)}
            anchorReference="anchorPosition"
            anchorPosition={contextMenu != null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
          >
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(serviceInfo.name);
                setContextMenu(null);
              }}
            >
              Copy service name
            </MenuItem>
            <MenuItem
              sx={{ fontSize: "0.8em" }}
              onClick={async () => {
                navigator.clipboard.writeText(serviceInfo.srvType);
                setContextMenu(null);
              }}
            >
              Copy service type
            </MenuItem>
          </Menu>
        </Stack>
      }
    />
  );
});

export default ServiceTreeItem;
