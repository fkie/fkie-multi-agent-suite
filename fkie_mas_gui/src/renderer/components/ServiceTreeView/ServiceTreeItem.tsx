import { ServiceExtendedInfo, TServiceNodeInfo } from "@/renderer/models";
import { Box, Stack, Typography } from "@mui/material";
import { forwardRef, LegacyRef, useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { NavigationContext } from "../../context/NavigationContext";
import { SettingsContext } from "../../context/SettingsContext";
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
  const [label, setLabel] = useState(serviceInfo.name);
  // state variables to show/hide extended info
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selected, setSelected] = useState(false);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);

  const getHostStyle = () => {
    if (serviceInfo?.providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        alignItems: "center",
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(serviceInfo.providerName),
        borderLeftWidth: "0.6em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  useEffect(() => {
    if (!rootPath) return;
    if (!serviceInfo) return;

    if (serviceInfo?.name === rootPath) {
      setLabel(serviceInfo.providerName);
    } else {
      setLabel(serviceInfo.name.slice(rootPath.length + 1));
    }
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
        <Stack direction="column">
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
              <Typography
                variant="body2"
                sx={{ fontWeight: "inherit" }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(label);
                    logCtx.success(`${label} copied!`);
                    e.stopPropagation();
                  }
                }}
              >
                {label}
              </Typography>
              {/* {requestData && <CircularProgress size="1em" />} */}
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
                  padding={0.5}
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
        </Stack>
      }
    />
  );
});

export default ServiceTreeItem;
