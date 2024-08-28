import ArrowForwardIosSharpIcon from "@mui/icons-material/ArrowForwardIosSharp";
import CheckBoxIcon from "@mui/icons-material/CheckBox";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import UndoIcon from "@mui/icons-material/Undo";
import {
  Autocomplete,
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
import { useCallback, useContext, useEffect, useState } from "react";
import { ISettingsParam, SettingsContext } from "../../context/SettingsContext";
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
  ...theme.applyStyles("dark", {
    backgroundColor: "rgba(255, 255, 255, .05)",
  }),
}));

const AccordionDetails = styled(MuiAccordionDetails)(({ theme }) => ({
  padding: theme.spacing(2),
  borderTop: "1px solid rgba(0, 0, 0, .125)",
}));

export interface IGroupEntry {
  group: string;
  params: { name: string; param: ISettingsParam }[];
  forceExpanded: boolean;
}

export default function GuiPanel() {
  const settingsCtx = useContext(SettingsContext);
  const [grouped, setGrouped] = useState<IGroupEntry[]>([]);
  const [expanded, setExpanded] = useState<string | false>(false);
  const [filter, setFilter] = useState("");

  const handleChange = (panel: string) => (_event: React.SyntheticEvent, newExpanded: boolean) => {
    setExpanded(newExpanded ? panel : false);
  };

  const createGroups = useCallback(() => {
    const groupedDict: { [group: string]: { name: string; param: ISettingsParam }[] } = {};
    settingsCtx.getParamList().forEach(({ name, param }) => {
      if (filter.length <= 1 || name.toLocaleLowerCase().includes(filter)) {
        const group = param.group ? param.group : "Application";
        if (!groupedDict[group]) {
          groupedDict[group] = [];
        }
        groupedDict[group].push({ name, param });
      }
    });
    const newGrouped: IGroupEntry[] = [];
    Object.keys(groupedDict).forEach(function (key) {
      newGrouped.push({ group: key, params: groupedDict[key], forceExpanded: filter.length > 1 });
    });
    setGrouped(newGrouped);
  }, [grouped, setGrouped, filter, settingsCtx]);

  useEffect(() => {
    createGroups();
  }, [settingsCtx.changed, filter]);

  return (
    <Stack sx={{ minHeight: "400px"}} overflow="hidden">
      <SearchBar
        onSearch={(value) => setFilter(value.toLocaleLowerCase())}
        placeholder="Filter Parameter"
        defaultValue=""
      />
      {grouped.map(({ group, params, forceExpanded }) => {
        return (
          <Accordion
            key={`${group}-accordion`}
            expanded={expanded === group || (forceExpanded && filter.length > 1)}
            onChange={handleChange(group)}
          >
            <AccordionSummary aria-controls={`${group}-content`} id={`${group}-header`}>
              <Typography>{group}</Typography>
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
                                getOptionLabel={(option) => option}
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
                                value={settingsCtx.get(name)}
                                onChange={(_event, newValue) => {
                                  settingsCtx.set(name, newValue);
                                }}
                                disableCloseOnSelect
                                renderOption={(props, option, { selected }) => {
                                  return (
                                    <li {...props} key={option} style={{ height: "1.5em" }}>
                                      <Checkbox icon={icon} checkedIcon={checkedIcon} checked={selected} size="small" />
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
                                      <MenuItem key={name} value={name}>
                                        {name}
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
                                  checked={settingsCtx.get(name)}
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
                        {param.description && <Typography sx={{ typography: "body2" }}>{param.description}</Typography>}
                        <Stack direction="row">
                          <TextField
                            key={`text-${param.label}`}
                            // label={param.label}
                            size="small"
                            variant="outlined"
                            fullWidth={true}
                            onChange={(e) => settingsCtx.set(name, `${e.target.value}`)}
                            value={settingsCtx.get(name)}
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
    </Stack>
  );
}
