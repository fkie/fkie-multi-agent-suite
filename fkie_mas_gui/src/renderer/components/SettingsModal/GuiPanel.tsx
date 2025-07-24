import ArrowForwardIosSharpIcon from "@mui/icons-material/ArrowForwardIosSharp";
import CheckBoxIcon from "@mui/icons-material/CheckBox";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import UndoIcon from "@mui/icons-material/Undo";
import {
  Autocomplete,
  Box,
  Button,
  Checkbox,
  FormControl,
  FormControlLabel,
  IconButton,
  MenuItem,
  Select,
  Stack,
  Switch,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary, { AccordionSummaryProps } from "@mui/material/AccordionSummary";
import { styled } from "@mui/material/styles";
import { useContext, useEffect, useMemo, useReducer, useState } from "react";

import { ISettingsParam, SettingsContext } from "@/renderer/context/SettingsContext";
import { JSONValue } from "@/types";
import SearchBar from "../UI/SearchBar";

const icon = <CheckBoxOutlineBlankIcon fontSize="small" />;
const checkedIcon = <CheckBoxIcon fontSize="small" />;

const Accordion = styled((props: AccordionProps) => <MuiAccordion disableGutters elevation={0} square {...props} />)(
  ({ theme }) => ({
    border: `1px solid ${theme.palette.divider}`,
    "&:not(:last-child)": {
      borderBottom: 0,
    },
    "&::before": {
      display: "none",
    },
  })
);

const AccordionSummary = styled((props: AccordionSummaryProps) => (
  <MuiAccordionSummary expandIcon={<ArrowForwardIosSharpIcon sx={{ fontSize: "0.9rem" }} />} {...props} />
))(({ theme }) => ({
  backgroundColor: "rgba(0, 0, 0, .03)",
  flexDirection: "row-reverse",
  "& .MuiAccordionSummary-expandIconWrapper.Mui-expanded": {
    transform: "rotate(90deg)",
  },
  "& .MuiAccordionSummary-content": {
    marginLeft: theme.spacing(1),
  },
  "&:hover": {
    fontWeight: "bolder",
  },
  // ...theme.applyStyles("dark", {
  //   backgroundColor: "rgba(255, 255, 255, .05)",
  // }),
}));

const AccordionDetails = styled(MuiAccordionDetails)(({ theme }) => ({
  backgroundColor: theme.palette.background.default,
  padding: theme.spacing(2),
  borderTop: "1px solid rgba(0, 0, 0, .125)",
}));

export interface IGroupEntry {
  group: string;
  params: { name: string; param: ISettingsParam }[];
  forceExpanded: boolean;
}

export default function GuiPanel(): JSX.Element {
  const settingsCtx = useContext(SettingsContext);
  const [grouped, setGrouped] = useState<IGroupEntry[]>([]);
  const [expanded, setExpanded] = useState<string[]>([]);
  const [filter, setFilter] = useState("");
  const [values] = useState<{ [x: string]: JSONValue | undefined }>({});
  const [valuesChanged, forceValuesUpdate] = useReducer((x) => x + 1, 0);

  function handleChange(panel: string): void {
    if (expanded.includes(panel)) {
      setExpanded(expanded.filter((item) => panel !== item));
    } else {
      setExpanded((prev) => [...prev, panel]);
    }
  }

  function createGroups(): void {
    const groupedDict: { [group: string]: { name: string; param: ISettingsParam }[] } = {};
    for (const item of settingsCtx.getParamList()) {
      values[item.name] = settingsCtx.get(item.name);
      if (filter.length <= 1 || item.name.toLocaleLowerCase().includes(filter)) {
        const group = item.param.group ? item.param.group : "Application";
        if (!groupedDict[group]) {
          groupedDict[group] = [];
        }
        groupedDict[group].push(item);
      }
    }
    const newGrouped: IGroupEntry[] = [];
    for (const key of Object.keys(groupedDict)) {
      newGrouped.push({ group: key, params: groupedDict[key], forceExpanded: filter.length > 1 });
    }
    setGrouped(newGrouped);
  }

  useEffect(() => {
    createGroups();
  }, [settingsCtx, settingsCtx.changed, filter]);

  const generateContent = useMemo(() => {
    return (
      <Stack height="100%" width="99%">
        <SearchBar
          onSearch={(value) => setFilter(value.toLocaleLowerCase())}
          placeholder="Filter Parameter"
          defaultValue=""
        />
        <Box height="100%" width="100%" overflow="auto">
          {grouped.map(({ group, params, forceExpanded }) => {
            return (
              <Accordion
                key={`${group}-accordion`}
                expanded={expanded.includes(group) || (forceExpanded && filter.length > 1)}
                onChange={() => handleChange(group)}
              >
                <AccordionSummary aria-controls={`${group}-content`} id={`${group}-header`}>
                  <Typography style={{ fontWeight: "inherit" }}>{group}</Typography>
                </AccordionSummary>
                <AccordionDetails>
                  <Stack margin={0} spacing={"0.7em"}>
                    {params.map(({ name, param }) => {
                      if (Array.isArray(param.options)) {
                        return (
                          <Stack
                            key={`opt-${name}-array`}
                            sx={{
                              "&:hover": {
                                backgroundColor: (theme) => theme.palette.action.hover,
                              },
                            }}
                          >
                            {param.type.endsWith("[]") ? (
                              // multiple values can be selected
                              <>
                                <Typography sx={{ fontWeight: "bold" }}>{param.label}</Typography>
                                {param.description && (
                                  <Typography sx={{ typography: "body2" }}>{param.description}</Typography>
                                )}
                                <Stack direction="row" alignItems="center">
                                  <Autocomplete
                                    key={name}
                                    disablePortal={false}
                                    handleHomeEndKeys={false}
                                    multiple
                                    id="auto-complete-debug"
                                    size="small"
                                    options={param.options}
                                    freeSolo={param.freeSolo}
                                    sx={{ margin: 0 }}
                                    getOptionLabel={(option) => option as string}
                                    renderInput={(params) => (
                                      <TextField
                                        {...params}
                                        variant="outlined"
                                        size="small"
                                        // label={param.label}
                                        placeholder={param.placeholder ? param.placeholder : "..."}
                                        // helperText={param.description}
                                      />
                                    )}
                                    value={settingsCtx.get(name) as JSONValue[]}
                                    onChange={(_event, newValue) => {
                                      settingsCtx.set(name, newValue);
                                    }}
                                    disableCloseOnSelect
                                    renderOption={(props, option, { selected }) => {
                                      return (
                                        <li {...props} key={option as string} style={{ height: "1.5em" }}>
                                          <Checkbox
                                            icon={icon}
                                            checkedIcon={checkedIcon}
                                            checked={selected}
                                            size="small"
                                          />
                                          {`${option}`}
                                        </li>
                                      );
                                    }}
                                  />
                                  {param.default && param.default !== settingsCtx.get(name) && (
                                    <Tooltip title="Restore default value" placement="bottom" disableInteractive>
                                      <IconButton
                                        onClick={() => {
                                          settingsCtx.set(name, param.default);
                                        }}
                                      >
                                        <UndoIcon fontSize="inherit" />
                                      </IconButton>
                                    </Tooltip>
                                  )}
                                </Stack>
                              </>
                            ) : (
                              // only one value
                              <>
                                <Typography sx={{ fontWeight: "bold" }}>{param.label}</Typography>
                                <FormControlLabel
                                  control={
                                    <Select
                                      labelId={`label-${param.label}`}
                                      id={param.label}
                                      autoWidth={false}
                                      value={settingsCtx.get(name)}
                                      onChange={(event) => {
                                        settingsCtx.set(name, event.target.value);
                                      }}
                                      size="small"
                                      sx={{ marginRight: "0.5em", minWidth: "15em" }}
                                      displayEmpty
                                    >
                                      {param.options.map((name) => {
                                        return (
                                          <MenuItem key={name as string} value={name as string}>
                                            {name as string}
                                          </MenuItem>
                                        );
                                      })}
                                    </Select>
                                  }
                                  sx={{ margin: 0 }}
                                  label={
                                    <Stack direction="row" alignItems="center">
                                      {param.default && param.default !== settingsCtx.get(name) && (
                                        <Tooltip title="Restore default value" placement="bottom" disableInteractive>
                                          <IconButton
                                            onClick={() => {
                                              settingsCtx.set(name, param.default);
                                            }}
                                          >
                                            <UndoIcon fontSize="inherit" />
                                          </IconButton>
                                        </Tooltip>
                                      )}
                                      <Typography sx={{ typography: "body2" }}>{param.description}</Typography>
                                    </Stack>
                                  }
                                />
                              </>
                            )}
                          </Stack>
                        );
                      }
                      if (param.type === "boolean") {
                        return (
                          <Stack
                            key={`opt-${name}-boolean`}
                            sx={{
                              "&:hover": {
                                backgroundColor: (theme) => theme.palette.action.hover,
                              },
                            }}
                          >
                            {param.description ? (
                              <>
                                <Typography sx={{ fontWeight: "bold" }}>{param.label}</Typography>
                                <FormControlLabel
                                  control={
                                    <Checkbox
                                      checked={settingsCtx.get(name) as boolean}
                                      onChange={(event) => {
                                        settingsCtx.set(name, event.target.checked);
                                      }}
                                    />
                                  }
                                  sx={{ margin: 0 }}
                                  label={<Typography sx={{ typography: "body2" }}>{param.description}</Typography>}
                                />
                              </>
                            ) : (
                              <FormControl key={name} component="fieldset" variant="standard" sx={{ margin: 0 }}>
                                <FormControlLabel
                                  control={
                                    <Switch
                                      // color="primary"
                                      checked={settingsCtx.get(name) as boolean}
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
                            )}
                          </Stack>
                        );
                      }
                      if (param.type === "number") {
                        return (
                          <Stack
                            key={`opt-${name}-number`}
                            direction="column"
                            spacing={0}
                            sx={{
                              "&:hover": {
                                backgroundColor: (theme) => theme.palette.action.hover,
                              },
                            }}
                            // alignItems={"center"}
                          >
                            <Typography sx={{ fontWeight: "bold" }}>{param.label}</Typography>
                            <FormControlLabel
                              control={
                                <TextField
                                  type="number"
                                  key={`number-${param.label}`}
                                  // label={param.label}
                                  size="small"
                                  variant="outlined"
                                  InputProps={{ inputProps: { min: param.min, max: param.max } }}
                                  fullWidth={false}
                                  onChange={(e) => settingsCtx.set(name, Number(`${e.target.value}`))}
                                  value={settingsCtx.get(name)}
                                  sx={{ margin: 0, marginRight: "0.5em" }}
                                  // helperText={param.description}
                                />
                              }
                              sx={{ margin: 0 }}
                              label={
                                <Stack direction="row" alignItems="center">
                                  {param.default !== undefined && param.default !== settingsCtx.get(name) && (
                                    <Tooltip title="Restore default value" placement="bottom" disableInteractive>
                                      <IconButton
                                        onClick={() => {
                                          settingsCtx.set(name, param.default);
                                        }}
                                      >
                                        <UndoIcon fontSize="inherit" />
                                      </IconButton>
                                    </Tooltip>
                                  )}
                                  <Typography sx={{ typography: "body2" }}>{param.description}</Typography>
                                </Stack>
                              }
                            />
                          </Stack>
                        );
                      }
                      if (param.type === "button") {
                        return (
                          <Stack
                            key={`opt-${name}-button`}
                            direction="row"
                            spacing={"1em"}
                            sx={{
                              "&:hover": {
                                backgroundColor: (theme) => theme.palette.action.hover,
                              },
                            }}
                            alignItems={"center"}
                          >
                            <Typography sx={{ typography: "body2" }}>{param.description}</Typography>
                            <Button
                              key={`button-${param.label}`}
                              // helperText={param.description}
                              variant="contained"
                              size="small"
                              onClick={() => settingsCtx.set(name, true)}
                            >
                              {param.label}
                            </Button>
                          </Stack>
                        );
                      }
                      if (param.type === "string") {
                        return (
                          <Stack
                            key={`opt-${name}-array`}
                            direction="column"
                            spacing={0}
                            sx={{
                              "&:hover": {
                                backgroundColor: (theme) => theme.palette.action.hover,
                              },
                            }}
                            // alignItems={"center"}
                          >
                            <Typography sx={{ fontWeight: "bold" }}>{param.label}</Typography>
                            {param.description && (
                              <Typography sx={{ typography: "body2" }}>{param.description}</Typography>
                            )}
                            <Stack direction="row">
                              <TextField
                                key={`text-${param.label}`}
                                // label={param.label}
                                size="small"
                                variant="outlined"
                                fullWidth={true}
                                error={param.isValid && !param.isValid(values[name] as string)}
                                onChange={(e) => {
                                  const value = `${e.target.value}`;
                                  if (!param.isValid || param.isValid?.(value)) {
                                    settingsCtx.set(name, param.validate ? param.validate(value) : value);
                                  }
                                  values[name] = value;
                                  forceValuesUpdate();
                                }}
                                value={values[name]}
                                sx={{ marginRight: "0.5em" }}
                                // helperText={param.description}
                              />
                              {param.default && param.default !== settingsCtx.get(name) && (
                                <Tooltip title="Restore default value" placement="bottom" disableInteractive>
                                  <IconButton
                                    onClick={() => {
                                      settingsCtx.set(name, param.default);
                                    }}
                                  >
                                    <UndoIcon fontSize="inherit" />
                                  </IconButton>
                                </Tooltip>
                              )}
                            </Stack>
                          </Stack>
                        );
                      }
                      // console.log(`Ignored PARAMETER: ${JSON.stringify(name)} of type ${param.type}`);
                      return "";
                    })}
                  </Stack>
                </AccordionDetails>
              </Accordion>
            );
          })}
        </Box>
      </Stack>
    );
  }, [grouped, expanded, valuesChanged]);

  return generateContent;
}
