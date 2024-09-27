import { Box, Stack, Typography } from "@mui/material";
import { alpha, styled } from "@mui/material/styles";
import { TreeItem, treeItemClasses } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import React, { useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { NavigationContext } from "../../context/NavigationContext";
import { SettingsContext } from "../../context/SettingsContext";
import { colorFromHostname } from "../UI/Colors";

const ServiceTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    borderTopRightRadius: theme.spacing(2),
    borderBottomRightRadius: theme.spacing(2),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    "&.Mui-expanded": {
      fontWeight: theme.typography.fontWeightRegular,
    },
    "&:hover": {
      backgroundColor: theme.palette.action.hover,
    },
    "&.Mui-focused, &.Mui-selected, &.Mui-selected.Mui-focused": {
      backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      color: "var(--tree-view-color)",
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: "inherit",
      color: "inherit",
      padding: theme.spacing(0),
    },
    [`& .${treeItemClasses.iconContainer}`]: {
      marginLeft: 0,
      marginRight: 0,
      padding: theme.spacing(0),
      width: 10,
    },
  },
  [`& .${treeItemClasses.groupTransition}`]: {
    marginLeft: 12,
    paddingLeft: 5,
    // [`& .${treeItemClasses.content}`]: {
    //   paddingLeft: theme.spacing(0),
    // },
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
    // borderColor: black[50],
  },
}));

const ServiceTreeItem = React.forwardRef(function ServiceTreeItem(
  {
    color = "#1a73e8",
    bgColor = "#e8f0fe",
    colorForDarkMode = "#B8E7FB",
    bgColorForDarkMode = "#071318",
    labelRoot = "",
    labelIcon = null,
    labelInfo = "",
    labelCount = null,
    labelText = "",
    serviceInfo = null,
    selectedItem = "",
    ...other
  },
  ref
) {
  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const settingsCtx = useContext(SettingsContext);
  const [label, setLabel] = useState(labelText);
  // state variables to show/hide extended info
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const [selected, setSelected] = useState(false);
  const [ignoreNextClick, setIgnoreNextClick] = useState(true);

  const styleProps = {
    "--tree-view-color": settingsCtx.get("useDarkMode") ? colorForDarkMode : color,
    "--tree-view-bg-color": settingsCtx.get("useDarkMode") ? bgColorForDarkMode : bgColor,
  };

  const getHostStyle = () => {
    if (serviceInfo?.providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        alignItems: "center",
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(serviceInfo?.providerName),
        borderLeftWidth: "0.6em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  useEffect(() => {
    if (!labelRoot) return;
    if (!serviceInfo) return;

    if (serviceInfo?.name === labelRoot) {
      setLabel(serviceInfo.providerName);
    } else {
      setLabel(labelText.slice(labelRoot.length + 1));
    }
  }, [labelRoot, labelText, serviceInfo]);

  useEffect(() => {
    // update state variables to show/hide extended info
    if (selectedItem !== other.itemId) {
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
    <ServiceTreeItemRoot
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
            {labelIcon && <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />}
            <Stack spacing={1} direction="row" sx={getHostStyle()}>
              <Typography
                variant="body2"
                sx={{ fontWeight: "inherit" }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(labelText);
                    logCtx.success(`${labelText} copied!`);
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
              {labelInfo && (
                <Typography
                  variant="caption"
                  color="inherit"
                  padding={0.5}
                  onClick={(e) => {
                    if (e.detail === 2) {
                      navigator.clipboard.writeText(labelInfo);
                      logCtx.success(`${labelInfo} copied!`);
                      e.stopPropagation();
                    }
                  }}
                >
                  {labelInfo}
                </Typography>
              )}
              {labelCount > 0 && (
                <Typography variant="caption" color="inherit" padding={0.5}>
                  [{labelCount}]
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
              {serviceInfo.nodeProviders.map((item) => {
                return (
                  <Stack key={item.nodeId} paddingLeft={1} direction="row">
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        navCtx.setSelectedNodes([`${serviceInfo.providerId}${item.nodeId.replaceAll("/", ".")}`]);
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
                            navCtx.setSelectedNodes([`${serviceInfo.providerId}${item.nodeId.replaceAll("/", ".")}`]);
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
      style={styleProps}
      {...other}
      ref={ref}
    />
  );
});

ServiceTreeItem.propTypes = {
  bgColor: PropTypes.string,
  color: PropTypes.string,
  labelRoot: PropTypes.string,
  labelIcon: PropTypes.object,
  labelInfo: PropTypes.string,
  labelCount: PropTypes.number,
  labelText: PropTypes.string,
  requestData: PropTypes.bool,
  colorForDarkMode: PropTypes.string,
  bgColorForDarkMode: PropTypes.string,
  serviceInfo: PropTypes.object,
  providerName: PropTypes.string,
  selectedItem: PropTypes.string,
};

export default ServiceTreeItem;
