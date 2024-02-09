import { alpha, styled } from '@mui/material/styles';
import PropTypes from 'prop-types';
import React, { useContext, useEffect, useState } from 'react';

import { Box, Chip, Stack, Typography } from '@mui/material';

import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

import { LoggingContext } from '../../context/LoggingContext';
import { SettingsContext } from '../../context/SettingsContext';
import { colorFromHostname } from '../UI/Colors';
import CopyButton from '../UI/CopyButton';

const TopicTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    borderTopRightRadius: theme.spacing(2),
    borderBottomRightRadius: theme.spacing(2),
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
    // borderColor: black[50],
  },
}));

const TopicTreeItem = React.forwardRef(function TopicTreeItem(
  {
    bgColor,
    color,
    labelRoot,
    labelIcon,
    labelInfo,
    labelCount,
    labelText,
    requestData,
    colorForDarkMode,
    bgColorForDarkMode,
    topicInfo,
    providerName,
    ...other
  },
  ref,
) {
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [label, setLabel] = useState(labelText);
  const [showExtendedInfo, setShowExtendedInfo] = useState(false);
  const styleProps = {
    '--tree-view-color': settingsCtx.get('useDarkMode')
      ? colorForDarkMode
      : color,
    '--tree-view-bg-color': settingsCtx.get('useDarkMode')
      ? bgColorForDarkMode
      : bgColor,
  };

  const getHostStyle = () => {
    if (topicInfo?.providerName && settingsCtx.get('colorizeHosts')) {
      return {
        flexGrow: 1,
        alignItems: 'center',
        borderLeftStyle: 'outset',
        borderLeftColor: colorFromHostname(topicInfo?.providerName),
        borderLeftWidth: '0.6em',
      };
    }
    return { flexGrow: 1, alignItems: 'center' };
  };

  useEffect(() => {
    if (!labelRoot) return;
    if (!topicInfo) return;

    if (topicInfo?.name === labelRoot) {
      setLabel(topicInfo.providerName);
    } else {
      setLabel(labelText.slice(labelRoot.length + 1));
    }
  }, [labelRoot, topicInfo]);

  return (
    <TopicTreeItemRoot
      label={
        <Stack direction="column">
          <Box
            sx={{
              display: 'flex',
              alignItems: 'center',
              p: 0.3,
              pr: 0,
            }}
          >
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}
            <Stack spacing={1} direction="row" sx={getHostStyle()}>
              <Typography
                variant="body2"
                sx={{ fontWeight: 'inherit' }}
                onClick={(e) => {
                  if (e.detail === 2) {
                    navigator.clipboard.writeText(labelText);
                    logCtx.success(`${labelText} copied!`);
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
                alignItems: 'center',
              }}
            >
              {labelInfo && (
                <Typography variant="caption" color="inherit" padding={0.5}>
                  [{labelInfo}]
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
                    color={
                      topicInfo.publishers.length > 0 ? 'default' : 'warning'
                    }
                    label={topicInfo.publishers.length}
                    onClick={(event) => {
                      setShowExtendedInfo(!showExtendedInfo);
                      event.stopPropagation();
                    }}
                  />

                  <Chip
                    size="small"
                    title="subscribers"
                    // showZero={true}
                    color={
                      topicInfo.subscribers.length > 0 ? 'default' : 'warning'
                    }
                    label={topicInfo.subscribers.length}
                    onClick={(event) => {
                      setShowExtendedInfo(!showExtendedInfo);
                      event.stopPropagation();
                    }}
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
                return (
                  <Stack key={item} paddingLeft={3} direction="row">
                    <Typography fontSize="small">{item}</Typography>
                    <CopyButton value={item} />
                  </Stack>
                );
              })}
              <Typography fontWeight="bold" fontSize="small">
                Subscriber [{topicInfo.subscribers.length}]:
              </Typography>
              {topicInfo.subscribers.map((item) => {
                return (
                  <Stack key={item} paddingLeft={3} direction="row">
                    <Typography fontSize="small">
                      {item}
                    </Typography>
                    <CopyButton value={item} />
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

TopicTreeItem.defaultProps = {
  color: '#1a73e8',
  bgColor: '#e8f0fe',
  colorForDarkMode: '#B8E7FB',
  bgColorForDarkMode: '#071318',
  labelRoot: '',
  labelIcon: null,
  labelInfo: '',
  labelCount: null,
  labelText: '',
  requestData: false,
  topicInfo: null,
  providerName: '',
};

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
};

export default TopicTreeItem;
