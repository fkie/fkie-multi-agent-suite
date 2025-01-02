import LinkOffIcon from "@mui/icons-material/LinkOff";
import { Box, Chip, Stack, Tooltip, Typography } from "@mui/material";
import { alpha, styled } from "@mui/material/styles";
import { TreeItem, treeItemClasses } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import React, { useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { LoggingContext } from "../../context/LoggingContext";
import { NavigationContext } from "../../context/NavigationContext";
import { SettingsContext } from "../../context/SettingsContext";
import { LAYOUT_TABS } from "../../pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../../pages/NodeManager/layout/events";
import { removeDDSuid } from "../../utils/index";
import { colorFromHostname } from "../UI/Colors";

const TopicTreeItemRoot = styled(TreeItem)(({ theme }) => ({
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
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
  },
  ...theme.applyStyles("light", {
    color: theme.palette.grey[800],
  }),
}));

const TopicTreeItem = React.forwardRef(function TopicTreeItem(
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
    // requestData = false,
    topicInfo = null,
    // providerName = "",
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
    if (topicInfo?.providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        alignItems: "center",
        borderLeftStyle: "solid",
        borderLeftColor: colorFromHostname(topicInfo?.providerName),
        borderLeftWidth: "0.6em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  useEffect(() => {
    if (!labelRoot) return;
    if (!topicInfo) return;

    if (topicInfo?.name === labelRoot) {
      setLabel(topicInfo.providerName);
    } else {
      setLabel(labelText.slice(labelRoot.length + 1));
    }
  }, [labelRoot, labelText, topicInfo]);

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
    <TopicTreeItemRoot
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
              {topicInfo && topicInfo.subscribers.filter((sub) => sub.incompatible_qos?.length > 0).length > 0 && (
                <Tooltip title={`There are subscribers with incompatible QoS`} placement="right" disableInteractive>
                  <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                </Tooltip>
              )}
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
                // <Tag text={labelCount} color="default" copyButton={false}></Tag>
                <Typography variant="caption" color="inherit" padding={0.5}>
                  [{labelCount}]
                </Typography>
              )}
              {topicInfo && (
                <Stack direction="row" spacing={1}>
                  <Chip
                    size="small"
                    title="publishers"
                    // showZero={true}
                    color={topicInfo.publishers.length > 0 ? "default" : "warning"}
                    label={topicInfo.publishers.length}
                  />

                  <Chip
                    size="small"
                    title="subscribers"
                    // showZero={true}
                    color={topicInfo.subscribers.length > 0 ? "default" : "warning"}
                    label={topicInfo.subscribers.length}
                  />
                </Stack>
              )}
            </Stack>
          </Box>
          {showExtendedInfo && topicInfo && (
            <Stack paddingLeft={3}>
              <Typography fontWeight="bold" fontSize="small">
                Publisher [{topicInfo.publishers.length}]:
              </Typography>
              {topicInfo.publishers.map((item) => {
                const pubNodeName = removeDDSuid(item.node_id || item);
                return (
                  <Stack key={item.node_id || item} paddingLeft={3} direction="row">
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        navCtx.setSelectedNodes([`${topicInfo.providerId}${item.node_id?.replaceAll("/", "#")}`]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default", {})
                        );
                      }}
                    >
                      {pubNodeName}
                    </Typography>
                    {/* <CopyButton value={pubNodeName} fontSize="0.7em" /> */}
                  </Stack>
                );
              })}
              <Typography fontWeight="bold" fontSize="small">
                Subscriber [{topicInfo.subscribers.length}]:
              </Typography>
              {topicInfo.subscribers.map((item) => {
                const subNodeName = removeDDSuid(item.node_id || item);
                return (
                  <Stack
                    key={item.node_id || item}
                    paddingLeft={3}
                    spacing={1}
                    direction="row"
                    justifyItems="center"
                    alignItems="center"
                  >
                    <Typography
                      fontSize="small"
                      onClick={() => {
                        navCtx.setSelectedNodes([`${topicInfo.providerId}${item.node_id?.replaceAll("/", "#")}`]);
                        // inform details panel tab about selected nodes by user
                        emitCustomEvent(
                          EVENT_OPEN_COMPONENT,
                          eventOpenComponent(LAYOUT_TABS.NODE_DETAILS, "default", {})
                        );
                      }}
                    >
                      {subNodeName}
                    </Typography>
                    {item.incompatible_qos.length > 0 && (
                      <Tooltip
                        title={`Incompatible QoS: ${JSON.stringify(item.incompatible_qos)}`}
                        placement="right"
                        disableInteractive
                      >
                        <LinkOffIcon style={{ fontSize: "inherit", color: "red" }} />
                      </Tooltip>
                    )}
                    {/* <CopyButton value={subNodeName} fontSize="0.7em" /> */}
                  </Stack>
                );
              })}
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

TopicTreeItem.propTypes = {
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
  topicInfo: PropTypes.object,
  providerName: PropTypes.string,
  selectedItem: PropTypes.string,
};

export default TopicTreeItem;
