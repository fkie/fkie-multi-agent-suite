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
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useReducer, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import { SearchBar, colorFromHostname } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import { RosNode } from "../../../models";
import { EVENT_PROVIDER_ROS_NODES } from "../../../providers/eventTypes";
import { findIn } from "../../../utils/index";

function NodeLoggerPanel(node) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [filterText, setFilterText] = useState("");
  const [currentNode] = useState(node.node);
  const [isRequesting, setIsRequesting] = useState(false);
  const [loggers, setLoggers] = useState([]);
  const [loggersFiltered, setLoggersFiltered] = useState([]);
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");
  const [changedLevels, forceUpdate] = useReducer((x) => x + 1, 0);

  const setLoggersOnProvider = useCallback(
    async (rosNode, newLoggers) => {
      // set loggers on ros node
      const provider = rosCtx.getProviderById(rosNode.providerId);
      if (provider) {
        setIsRequesting(true);
        await provider.setNodeLoggers(rosNode.id, newLoggers);
        setIsRequesting(false);
      }
    },
    [rosCtx]
  );

  /**
   * Get nodes for selected ids
   */
  const getLoggers = useCallback(
    async (rosNode) => {
      let ownLoggers = [];
      if (rosNode) {
        const provider = rosCtx.getProviderById(rosNode.providerId);
        if (provider) {
          setIsRequesting(true);
          ownLoggers = await provider.getNodeLoggers(rosNode.id);
          if (ownLoggers?.length > 0) {
            // fix case: c++ nodes in ROS1 returns log level in lower case
            ownLoggers = ownLoggers?.map((item) => {
              return { name: item.name, level: item.level.toLocaleUpperCase() };
            });
          }
          setIsRequesting(false);
        }
      }
      if (ownLoggers?.length > 0 && ownLoggers[0].name !== "all") {
        ownLoggers = [{ name: "all", level: "" }, ...ownLoggers];
      }
      setLoggers(ownLoggers);
      // compare new loggers and update to set by user
      const forceUpdateLevels = [];
      // eslint-disable-next-line no-restricted-syntax
      for (const [key, value] of Object.entries(rosNode.rosLoggers)) {
        let changed = true;
        ownLoggers.forEach((l) => {
          if (l.name === key && l.level === value) {
            changed = false;
          }
        });
        if (changed) {
          forceUpdateLevels.push({ name: key, level: value });
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
  }, [currentNode, getLoggers]);

  const debouncedCallbackFilterText = useDebounceCallback(
    (searchTerm) => {
      if (searchTerm.length > 0) {
        const newFilteredLoggers = [];
        loggers.forEach((logger) => {
          const isMatch = findIn(searchTerm, [logger.name]);
          if (isMatch) {
            newFilteredLoggers.push(logger);
          }
        });
        setLoggersFiltered(newFilteredLoggers);
      } else {
        setLoggersFiltered(loggers);
      }
    },
    [loggers],
    300
  );

  useEffect(() => {
    debouncedCallbackFilterText(filterText);
  }, [loggers, filterText, debouncedCallbackFilterText]);

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  useCustomEventListener(EVENT_PROVIDER_ROS_NODES, (data) => {
    getLoggers(currentNode);
  });

  const updateLoggerLevel = useCallback(
    async (loggerName, level) => {
      let changedLoggers = [];
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
      changedLoggers.forEach((l) => {
        if (l.name !== "all") {
          currentNode.rosLoggers[l.name] = l.level;
          forceUpdate();
        }
      });
      // set loggers on ros node
      setLoggersOnProvider(
        currentNode,
        changedLoggers.filter((logger) => logger.name !== "all"),
        true
      );
    },
    [loggers, currentNode, setLoggersOnProvider]
  );

  const getHostStyle = (providerName) => {
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
  };

  useEffect(() => {
    return () => {
      // remove user defined changes from node
      if (currentNode) {
        currentNode.rosLoggers = {};
        forceUpdate();
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
      backgroundColor={settingsCtx.get("backgroundColor")}
    >
      <Stack direction="row" spacing="1em" justifyItems="center">
        <Typography sx={getHostStyle(currentNode?.providerName)}>{currentNode?.name}</Typography>
        {isRequesting && <CircularProgress size="1em" />}
      </Stack>
      <Stack direction="row" spacing={0.5} justifyItems="center">
        <Tooltip
          title={`Remove last changed levels to prevent change them after node restart!`}
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
          disableInteractive
        >
          <span>
            <IconButton
              size="small"
              aria-label="Remove user changes"
              onClick={() => {
                if (currentNode) {
                  currentNode.rosLoggers = {};
                  forceUpdate();
                }
              }}
              disabled={Object.keys(currentNode?.rosLoggers).length === 0}
            >
              <PlaylistRemoveIcon fontSize="inherit" />
            </IconButton>
          </span>
        </Tooltip>
        <SearchBar
          onSearch={(value) => {
            setFilterText(value);
          }}
          placeholder="Filter loggers (OR: <space>, AND: +, NOT: !)"
          defaultValue={filterText}
          // fullWidth={true}
        />
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
                        onChange={(event, value) => {
                          updateLoggerLevel(logger.name, value);
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

NodeLoggerPanel.propTypes = {
  node: PropTypes.instanceOf(RosNode).isRequired,
};

export default NodeLoggerPanel;
