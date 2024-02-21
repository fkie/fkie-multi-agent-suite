import { useContext, useEffect } from 'react';

import {
  Autocomplete,
  Button,
  Checkbox,
  FormControl,
  FormControlLabel,
  FormHelperText,
  InputLabel,
  MenuItem,
  Select,
  Stack,
  Switch,
  TextField,
} from '@mui/material';

import CheckBoxIcon from '@mui/icons-material/CheckBox';
import CheckBoxOutlineBlankIcon from '@mui/icons-material/CheckBoxOutlineBlank';

import { SettingsContext } from '../../context/SettingsContext';

const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;

function GuiPanel() {
  const settingsCtx = useContext(SettingsContext);

  return (
    <Stack spacing={1.5} sx={{ minHeight: 400 }} overflow="auto">
      {settingsCtx.getParamList().map(({ name, param }) => {
        if (Array.isArray(param.options)) {
          if (param.type.endsWith('[]')) {
            // multiple values can be selected
            return (
              <Autocomplete
                key={name}
                disablePortal={false}
                multiple
                id="auto-complete-debug"
                size="small"
                options={param.options}
                freeSolo={param.freeSolo}
                sx={{ margin: 0 }}
                getOptionLabel={(option) => option}
                renderInput={(params) => (
                  <TextField
                    {...params}
                    variant="outlined"
                    size="small"
                    label={param.label}
                    placeholder={param.placeholder ? param.placeholder : '...'}
                    helperText={param.description}
                  />
                )}
                value={settingsCtx.get(name)}
                onChange={(event, newValue) => {
                  settingsCtx.set(name, newValue);
                }}
                disableCloseOnSelect
                renderOption={(props, option, { selected }) => (
                  <li {...props} style={{ height: '1.5em' }}>
                    <Checkbox
                      icon={icon}
                      checkedIcon={checkedIcon}
                      checked={selected}
                      size="small"
                    />
                    {`${option}`}
                  </li>
                )}
              />
            );
          }
          // only one value
          return (
            <FormControl
              key={name}
              // sx={{ m: 1, width: '100%' }}
              variant="standard"
            >
              <InputLabel id={`label-${param.label}`}>{param.label}</InputLabel>
              <Select
                labelId={`label-${param.label}`}
                id={param.label}
                autoWidth={false}
                value={settingsCtx.get(name)}
                onChange={(event) => {
                  settingsCtx.set(name, event.target.value);
                }}
                size="small"
                displayEmpty
              >
                {param.options.map((name) => {
                  return (
                    <MenuItem key={name} value={name}>
                      {name}
                    </MenuItem>
                  );
                })}
              </Select>
              {param.description && (
                <FormHelperText>{param.description}</FormHelperText>
              )}
            </FormControl>
          );
        }
        if (param.type === 'boolean') {
          return (
            <FormControl key={name} component="fieldset" variant="standard">
              <FormControlLabel
                control={
                  <Switch
                    // color="primary"
                    checked={settingsCtx.get(name)}
                    onChange={(event) => {
                      settingsCtx.set(name, event.target.checked);
                    }}
                  />
                }
                label={param.label}
                labelPlacement="end"
                aria-label={param.label}
                id={`toggle-${param.label}`}
              />
            </FormControl>
          );
        }
        if (param.type === 'number') {
          return (
            <TextField
              type="number"
              key={`number-${param.label}`}
              min={param.min}
              max={param.max}
              label={param.label}
              size="small"
              variant="outlined"
              InputProps={{ inputProps: { min: param.min, max: param.max } }}
              fullWidth
              onChange={(e) =>
                settingsCtx.set(name, Number(`${e.target.value}`))
              }
              value={settingsCtx.get(name)}
              helperText={param.description}
            />
          );
        }
        if (param.type === 'button') {
          return (
            <Button
              key={`button-${param.label}`}
              // helperText={param.description}
              variant="outlined"
              size="small"
              onClick={(e) => settingsCtx.set(name, true)}
            >
              {param.label}
            </Button>
          );
        }
        // console.log(`Ignored PARAMETER: ${JSON.stringify(name)} of type ${param.type}`);
        return '';
      })}
    </Stack>
  );
}

export default GuiPanel;
