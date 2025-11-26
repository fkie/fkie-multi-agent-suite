import PlaylistAddIcon from "@mui/icons-material/PlaylistAdd";
import PlaylistRemoveIcon from "@mui/icons-material/PlaylistRemove";
import RefreshIcon from "@mui/icons-material/Refresh";
import {
  CircularProgress,
  IconButton,
  Radio,
  RadioGroup,
  Stack,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { useCallback, useContext, useEffect, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { colorFromHostname } from "@/renderer/components/UI/Colors";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import { LoggerConfig, LogLevelType, RosNode } from "@/renderer/models";
import { EVENT_PROVIDER_ROS_NODES } from "@/renderer/providers/eventTypes";
import { findIn } from "@/renderer/utils/index";

interface NodeLoggerPanelProps {
  node: RosNode;
}

export default function NodeLoggerPanel(props: NodeLoggerPanelProps): JSX.Element {
  const { node } = props;

  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [filterText, setFilterText] = useState("");
  const [currentNode] = useState(node);
  const [isRequesting, setIsRequesting] = useState(false);
  const [addable] = useState<boolean>(node.id.indexOf("-") > 0);
  const [loggers, setLoggers] = useState<LoggerConfig[]>([]);
  const [loggersFiltered, setLoggersFiltered] = useState<LoggerConfig[]>([]);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  const setLoggersOnProvider = useCallback(
    async (rosNode: RosNode, newLoggers) => {
      // set loggers on ros node
      const provider = rosCtx.getProviderById(rosNode.providerId);
      if (provider) {
        setIsRequesting(true);
        await provider.setNodeLoggers(rosNode.name, newLoggers);
        setIsRequesting(false);
      }
    },
    [rosCtx]
  );

  /**
   * Get nodes for selected ids
   */
  const getLoggers = useCallback(
    async (rosNode: RosNode) => {
      let ownLoggers: LoggerConfig[] = [];
      if (rosNode?.providerId) {
        const provider = rosCtx.getProviderById(rosNode.providerId);
        if (provider) {
          setIsRequesting(true);
          ownLoggers = await provider.getNodeLoggers(rosNode.name, []);
          if (ownLoggers?.length > 0) {
            // fix case: c++ nodes in ROS1 returns log level in lower case
            ownLoggers = ownLoggers?.map((item) => {
              return { name: item.name, level: item.level.toLocaleUpperCase() } as LoggerConfig;
            });
          }
          setIsRequesting(false);
        }
      }
      if (ownLoggers?.length > 0 && ownLoggers[0].name !== "all") {
        ownLoggers = [{ name: "all", level: LogLevelType.UNKNOWN }, ...ownLoggers];
      }
      setLoggers(ownLoggers);
      // compare new loggers and update to set by user
      const forceUpdateLevels: LoggerConfig[] = [];
      // eslint-disable-next-line no-restricted-syntax
      for (const [key, value] of Object.entries(rosNode.rosLoggers)) {
        let changed = true;
        for (const l of ownLoggers) {
          if (l.name === key && l.level === value) {
            changed = false;
          }
        }
        if (changed) {
          forceUpdateLevels.push({ name: key, level: value as LogLevelType });
        }
      }
      if (forceUpdateLevels.length > 0) {
        setLoggersOnProvider(rosNode, forceUpdateLevels);
      }
    },
    [rosCtx, setLoggersOnProvider]
  );

  useEffect(() => {
    getLoggers(currentNode);
  }, [currentNode]);

  const debouncedCallbackFilterText = useDebounceCallback((searchTerm: string) => {
    if (searchTerm.length > 0) {
      const newFilteredLoggers: LoggerConfig[] = [];
      for (const logger of loggers) {
        const isMatch = findIn(searchTerm, [logger.name]);
        if (isMatch) {
          newFilteredLoggers.push(logger);
        }
      }
      setLoggersFiltered(newFilteredLoggers);
    } else {
      setLoggersFiltered(loggers);
    }
  }, 300);

  useEffect(() => {
    debouncedCallbackFilterText(filterText);
  }, [loggers, filterText]);

  useCustomEventListener(EVENT_PROVIDER_ROS_NODES, () => {
    getLoggers(currentNode);
  });

  const updateLoggerLevel = useCallback(
    async (loggerName: string, level: LogLevelType) => {
      let changedLoggers: LoggerConfig[] = [];
      if (loggerName === "all") {
        // change all logger levels to new level
        changedLoggers = loggers.map((logger) => {
          logger.level = level;
          return logger;
        });
        setLoggers(changedLoggers);
      } else {
        // change logger level for single logger
        changedLoggers = [{ level, name: loggerName }];
        setLoggers(
          loggers.map((logger) => {
            if (logger.name === loggerName) {
              logger.level = level;
            }
            return logger;
          })
        );
      }
      // store changed loggers by user
      for (const l of changedLoggers) {
        if (l.name !== "all") {
          currentNode.rosLoggers[l.name] = l.level;
        }
      }
      // set loggers on ros node
      setLoggersOnProvider(
        currentNode,
        changedLoggers.filter((logger) => logger.name !== "all")
      );
    },
    [loggers, currentNode, setLoggersOnProvider]
  );

  const getHostStyle = useCallback(
    function getHostStyle(providerName: string): object {
      if (settingsCtx.get("colorizeHosts")) {
        // borderLeft: `3px dashed`,
        // borderColor: colorFromHostname(provider.name()),
        return {
          borderLeftStyle: "solid",
          borderLeftColor: colorFromHostname(providerName),
          borderLeftWidth: "0.6em",
        };
      }
      return {};
    },
    [settingsCtx.changed]
  );

  useEffect(() => {
    return (): void => {
      // remove user defined changes from node
      if (currentNode) {
        currentNode.rosLoggers = {};
      }
    };
  }, []);

  const radioSize = { width: "2em", height: "2em" };

  return (
    <Stack
      spacing={1}
      height="100%"
      // width="100%"
      overflow="auto"
      sx={{ backgroundColor: backgroundColor }}
    >
      <Stack direction="row" spacing="1em" justifyItems="center">
        <Typography sx={getHostStyle(currentNode?.providerName)}>{currentNode?.name}</Typography>
        {isRequesting && <CircularProgress size="1em" />}
      </Stack>
      <Stack direction="row" spacing={0.1} justifyItems="center">
        {addable && !addable && (
          <Tooltip title={"Add a new logger name"} placement="bottom" enterDelay={tooltipDelay} disableInteractive>
            <IconButton
              size="small"
              aria-label="add logger name"
              onClick={() => {
                if (currentNode) {
                  currentNode.rosLoggers = {};
                }
              }}
            >
              <PlaylistAddIcon fontSize="inherit" />
            </IconButton>
          </Tooltip>
        )}
        <Tooltip
          title={"Remove last changed levels to prevent change them after node restart!"}
          placement="bottom"
          enterDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="small"
              aria-label="Remove user changes"
              onClick={() => {
                if (currentNode) {
                  currentNode.rosLoggers = {};
                }
              }}
              disabled={Object.keys(currentNode?.rosLoggers).length === 0}
            >
              <PlaylistRemoveIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip
          title="Refresh logger list"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <IconButton
            size="small"
            edge="start"
            aria-label="refresh logger list"
            onClick={() => getLoggers(currentNode)}
          >
            <RefreshIcon sx={{ fontSize: "inherit" }} />
          </IconButton>
        </Tooltip>
        <SearchBar
          onSearch={(value) => {
            setFilterText(value);
          }}
          placeholder="Filter loggers (OR: <space>, AND: +, NOT: !)"
          defaultValue={filterText}
          // fullWidth={true}
        />
      </Stack>
      <TableContainer>
        <Table aria-label="logger table">
          <TableBody>
            {loggersFiltered?.length > 0 &&
              loggersFiltered.map((logger) => {
                return (
                  <TableRow
                    key={logger.name}
                    style={{
                      display: "block",
                      padding: 0,
                    }}
                  >
                    <TableCell style={{ padding: 0 }}>
                      <RadioGroup
                        name={`logger-group-${logger.name}`}
                        value={logger.level}
                        onChange={(_event, value) => {
                          updateLoggerLevel(logger.name, value as LogLevelType);
                        }}
                        style={{
                          // width: 'auto',
                          // height: 'auto',
                          // display: 'flex',
                          flexWrap: "nowrap",
                          flexDirection: "row",
                        }}
                      >
                        <Tooltip title="fatal" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
                          <Radio
                            value="FATAL"
                            size="small"
                            sx={{
                              color: "#C0392B",
                              "&.Mui-checked": {
                                color: "#C0392B",
                              },
                              ...radioSize,
                            }}
                          />
                        </Tooltip>
                        <Tooltip title="error" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
                          <Radio
                            value="ERROR"
                            size="small"
                            sx={{
                              color: "#D35400",
                              "&.Mui-checked": {
                                color: "#D35400",
                              },
                              ...radioSize,
                            }}
                          />
                        </Tooltip>
                        <Tooltip title="warning" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
                          <Radio
                            value="WARN"
                            size="small"
                            sx={{
                              color: "#F39C12",
                              "&.Mui-checked": {
                                color: "#F39C12cd",
                              },
                              ...radioSize,
                            }}
                          />
                        </Tooltip>
                        <Tooltip title="info" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
                          <Radio
                            value="INFO"
                            size="small"
                            sx={{
                              color: "#2980B9",
                              "&.Mui-checked": {
                                color: "#2980B9",
                              },
                              ...radioSize,
                            }}
                          />
                        </Tooltip>
                        <Tooltip title="debug" placement="bottom" enterDelay={tooltipDelay} disableInteractive>
                          <Radio
                            value="DEBUG"
                            color="default"
                            size="small"
                            sx={{
                              ...radioSize,
                            }}
                          />
                        </Tooltip>
                      </RadioGroup>
                    </TableCell>
                    <TableCell
                      style={{
                        padding: 2,
                        flexGrow: 1,
                        width: "100%",
                      }}
                    >
                      {logger.name}
                    </TableCell>
                  </TableRow>
                );
              })}
          </TableBody>
        </Table>
      </TableContainer>
    </Stack>
  );
}
