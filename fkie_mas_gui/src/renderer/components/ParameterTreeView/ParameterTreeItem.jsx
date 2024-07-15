import { alpha, styled } from '@mui/material/styles';
import PropTypes from 'prop-types';
import React, { useContext, useState } from 'react';

import {
  Box,
  CircularProgress,
  Stack,
  Switch,
  TextField,
  Typography,
} from '@mui/material';

import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

import { LoggingContext } from '../../context/LoggingContext';
import { SettingsContext } from '../../context/SettingsContext';
import { colorFromHostname } from '../UI/Colors';
import OverflowMenu from '../UI/OverflowMenu';

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
    color = '#1a73e8',
    bgColor = '#e8f0fe',
    colorForDarkMode = '#B8E7FB',
    bgColorForDarkMode = '#071318',
    labelRoot = '',
    labelIcon = null,
    labelInfo = '',
    labelCount = null,
    labelText = '',
    requestData = false,
    param = null,
    providerName = '',
    updateParameter = () => {},

    ...other
  },
  ref,
) {
  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [parameterType, setParameterType] = useState(
    param ? param.type : labelInfo,
  );

  const typeOptions = [
    {
      name: 'int',
      key: 'int',
      onClick: () => {
        setParameterType('int');
        updateParameter(param, param.value, 'int');
      },
    },
    {
      name: 'float',
      key: 'float',
      onClick: () => {
        setParameterType('float');
        updateParameter(param, param.value, 'float');
      },
    },
    {
      name: 'str',
      key: 'str',
      onClick: () => {
        setParameterType('str');
        updateParameter(param, param.value, 'str');
      },
    },
    {
      name: 'bool',
      key: 'bool',
      onClick: () => {
        setParameterType('bool');
        updateParameter(param, param.value, 'bool');
      },
    },
  ];

  const styleProps = {
    '--tree-view-color': settingsCtx.get('useDarkMode')
      ? colorForDarkMode
      : color,
    '--tree-view-bg-color': settingsCtx.get('useDarkMode')
      ? bgColorForDarkMode
      : bgColor,
  };

  const renderInput = () => {
    if (['int', 'float'].includes(parameterType)) {
      return (
        <TextField
          type="number"
          id={`input-${param.id}`}
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
    if (['list'].includes(parameterType)) {
      // TODO: show proper list/arrays
      return (
        <TextField
        id={`input-${param.id}`}
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
    if (['bool'].includes(parameterType)) {
      return (
        <Switch
        id={`input-${param.id}`}
          checked={param.value ? true : false}
          onChange={(event) => {
            updateParameter(param, event.target.checked);
          }}
        />
      );
    }

    // default render (usually string)
    return (
      <TextField
        key={param.id}
        id={`input-${param.id}`}
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
        borderLeftStyle: 'solid',
        borderLeftColor: colorFromHostname(providerName),
        borderLeftWidth: '0.6em',
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
            // p: 0.3,
            padding: 0,
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
            {parameterType && (
              <OverflowMenu
                icon={
                  <Typography variant="caption" color="inherit" padding={0.5}>
                    [{parameterType}]
                  </Typography>
                }
                options={typeOptions}
                id="provider-options"
              />
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
