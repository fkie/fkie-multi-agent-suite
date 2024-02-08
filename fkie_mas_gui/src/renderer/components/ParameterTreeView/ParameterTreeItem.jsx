import { alpha, styled } from '@mui/material/styles';
import PropTypes from 'prop-types';
import React, { useContext } from 'react';

import {
  Box,
  CircularProgress,
  Stack,
  Switch,
  TextField,
  Typography,
} from '@mui/material';

import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

import { colorFromHostname } from '../UI/Colors';
import { LoggingContext } from '../../context/LoggingContext';
import { SettingsContext } from '../../context/SettingsContext';

const ParameterTreeItemRoot = styled(TreeItem)(({ theme }) => ({
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

const ParameterTreeItem = React.forwardRef(function ParameterTreeItem(
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
    param,
    providerName,
    updateParameter,
    ...other
  },
  ref,
) {
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);

  const styleProps = {
    '--tree-view-color': settingsCtx.get('useDarkMode')
      ? colorForDarkMode
      : color,
    '--tree-view-bg-color': settingsCtx.get('useDarkMode')
      ? bgColorForDarkMode
      : bgColor,
  };

  const renderInput = () => {
    if (['int', 'float'].includes(param.type)) {
      return (
        <TextField
          type="number"
          id={param.id}
          size="small"
          variant="standard"
          defaultValue={param.value}
          placeholder={JSON.stringify(param.value)}
          // InputProps={{ inputProps: { min: 0, max: 99 } }}
          fullWidth
          onChange={(event) => {
            updateParameter(param, event.target.value);
          }}
        />
      );
    }
    if (['list'].includes(param.type)) {
      // TODO: show proper list/arrays
      return (
        <TextField
          id={param.id}
          defaultValue={`${JSON.stringify(param.value)}`}
          placeholder={`${JSON.stringify(param.value)}`}
          variant="standard"
          size="small"
          fullWidth
          onChange={(event) => {
            updateParameter(param, event.target.value);
          }}
        />
      );
    }
    if (['bool'].includes(param.type)) {
      return (
        <Switch
          id={param.id}
          checked={param.value}
          onChange={(event) => {
            updateParameter(param, event.target.checked);
          }}
        />
      );
    }

    // default render (usually string)
    return (
      <TextField
        id={param.id}
        defaultValue={`${param.value}`}
        placeholder={`${param.value}`}
        variant="standard"
        size="small"
        fullWidth
        onChange={(event) => {
          updateParameter(param, event.target.value);
        }}
      />
    );
  };

  const getHostStyle = () => {
    if (providerName && settingsCtx.get('colorizeHosts')) {
      return {
        borderLeftStyle: 'outset',
        borderLeftColor: colorFromHostname(providerName),
        borderLeftWidth: '10px',
      };
    }
    return {};
  };

  return (
    <ParameterTreeItemRoot
      sx={getHostStyle()}
      label={
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
          <Stack
            spacing={1}
            direction="row"
            sx={{
              flexGrow: 1,
              alignItems: 'center',
            }}
          >
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
              {labelRoot && param
                ? labelText.slice(labelRoot.length + 1)
                : labelText}
            </Typography>
            {requestData && <CircularProgress size="1em" />}
          </Stack>
          <Stack
            direction="row"
            spacing={1}
            sx={{
              alignItems: 'end',
            }}
          >
            {labelInfo && labelInfo !== 'bool' && (
              <Typography variant="caption" color="inherit" padding={0.5}>
                [{labelInfo}]
              </Typography>
            )}
            {labelCount && (
              // <Tag text={labelCount} color="default" copyButton={false}></Tag>
              <Typography variant="caption" color="inherit" padding={0.5}>
                [{labelCount}]
              </Typography>
            )}
            {param && renderInput()}
          </Stack>
        </Box>
      }
      style={styleProps}
      {...other}
      ref={ref}
    />
  );
});

ParameterTreeItem.defaultProps = {
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
  param: null,
  providerName: '',
  updateParameter: () => {},
};

ParameterTreeItem.propTypes = {
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
  param: PropTypes.object,
  providerName: PropTypes.string,
  updateParameter: PropTypes.func,
};

export default ParameterTreeItem;
