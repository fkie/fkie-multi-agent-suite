import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import StorageOutlinedIcon from "@mui/icons-material/StorageOutlined";
import {
  Alert,
  AlertTitle,
  Box,
  Button,
  ButtonGroup,
  CircularProgress,
  Divider,
  FormLabel,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { JSONTree } from "react-json-tree";
import { SearchBar } from "../../../components";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { LaunchCallService } from "../../../models";
import { darkThemeJson } from "../../../themes/darkTheme";
import { lightThemeJson } from "../../../themes/lightTheme";
import InputElements from "./MessageDialogPanel/InputElements";

function ServiceCallerPanel({ serviceName = null, providerId = "" }) {
  const settingsCtx = useContext(SettingsContext);
  const [history, setHistory] = useLocalStorage(`ServiceStruct:history`, {});
  const [historyLength, setHistoryLength] = useState(0);
  const rosCtx = useContext(RosContext);
  const [serviceType, setServiceType] = useState("");
  const [searchTerm, setSearchTerm] = useState("");
  const [serviceStruct, setServiceStruct] = useState(null);
  const [serviceStructOrg, setServiceStructOrg] = useState(null);
  const [provider, setProvider] = useState(null);
  const [inputElements, setInputElements] = useState(null);

  const [callServiceStatus, setCallServiceStatus] = useState("");
  const [callServiceDescription, setCallServiceDescription] = useState("");
  const [callServiceIsSubmitting, setCallServiceIsSubmitting] = useState(false);
  const [resultError, setResultError] = useState(null);
  const [resultMessage, setResultMessage] = useState(null);
  const [timeoutObj, setTimeoutObj] = useState(null);

  // Removes " from keys
  function dictToString(dict) {
    const json = JSON.stringify(dict);
    return json.replace(
      /("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)/g,
      (match) => {
        let cls = "number";
        if (/^"/.test(match)) {
          if (/:$/.test(match)) {
            cls = "key";
          } else {
            cls = "string";
          }
        } else if (/true|false/.test(match)) {
          cls = "boolean";
        } else if (/null/.test(match)) {
          cls = "null";
        }
        return cls === "key" || cls === "number" || cls === "boolean" ? match.replaceAll('"', "") : match;
      }
    );
  }

  function str2typedValue(value, valueType) {
    let result = value;
    if (valueType.search("int") !== -1) {
      result = Number(value);
    } else if (valueType.search("float") !== -1 || valueType.search("double") !== -1) {
      result = Number(value);
    } else if (valueType.startsWith("bool")) {
      try {
        result = ("yes", "true", "t", "y", "1").includes(value.toLowerCase());
      } catch {
        // Do nothing
      }
    }
    return result;
  }

  // The message struct is converted into a human-readable string.
  const messageStructToString = useCallback((msgStruct, asDict, withEmptyFields) => {
    if (!msgStruct) return "{}";
    const result = {};
    let struct = msgStruct;
    if (msgStruct.def) {
      struct = msgStruct.def;
    }
    struct.forEach((field) => {
      if (field.def.length === 0) {
        // simple types
        if (field.value || withEmptyFields) {
          if (field.value) {
            if (field.is_array) {
              const values = field.value.split(/\s*,\s*/);
              // TODO: add check for arrays with constant length
              result[field.name] = values.map((element) => {
                return str2typedValue(element, field.type);
              });
            } else {
              result[field.name] = str2typedValue(field.value, field.type);
            }
          } else if (field.default_value) {
            result[field.name] = field.default_value;
          } else {
            result[field.name] = "";
          }
        }
      } else if (field.is_array) {
        const resultArray = [];
        // it is a complex field type
        const val = field?.value ? field?.value : field.def;
        val.forEach((arrayElement) => {
          resultArray.push(messageStructToString(arrayElement, true, withEmptyFields));
        });
        // append created array
        if (resultArray.length > 0) {
          result[field.name] = resultArray;
        } else if (withEmptyFields) {
          // create a new array from definition
          result[field.name] = [messageStructToString(field.def, true, withEmptyFields)];
        }
      } else {
        // it is a complex type, call subroutine
        const subResult = messageStructToString(field.def, true, withEmptyFields);
        if (Object.keys(subResult).length > 0) {
          result[field.name] = subResult;
        }
      }
    });
    return asDict ? result : dictToString(result);
  }, []);

  // get item history after the history was loaded
  const fromHistory = useDebounceCallback((index) => {
    const historyInStruct = history[serviceType];
    if (historyInStruct) {
      const historyItem = index && index < historyInStruct.length ? historyInStruct[index] : historyInStruct[0];
      setServiceStruct(JSON.parse(JSON.stringify(historyItem.msg)));
    }
  }, 100);

  // get item history after the history was loaded
  const updateHistory = useDebounceCallback(() => {
    if (!serviceStruct) return;
    let historyInStruct = history[serviceType];
    let hasType = true;
    if (!historyInStruct) {
      hasType = false;
      historyInStruct = [];
    }
    historyInStruct.unshift({
      msg: serviceStruct,
    });
    if (historyInStruct.length > 5) {
      historyInStruct.pop();
    }
    if (historyInStruct.length > 0) {
      history[serviceType] = historyInStruct;
    } else if (hasType) {
      history.delete(serviceType);
    }
    setHistory(history);
  }, 100);

  // get item history after the history was loaded
  const clearHistory = useDebounceCallback(() => {
    if (!serviceStruct) return;
    const historyInStruct = history[serviceType];
    if (historyInStruct && historyInStruct.length > 0) {
      historyInStruct.pop();
      setHistory(history);
      setHistoryLength(historyInStruct.length);
    }
  }, 300);

  // create string from message struct and copy it to clipboard
  const onCopyToClipboard = useDebounceCallback(() => {
    if (!serviceStruct) return;
    const json = messageStructToString(serviceStruct, false, true);
    navigator.clipboard.writeText(`${serviceType} ${json}`);
  }, 300);

  const getServiceStructData = useCallback(async () => {
    if (serviceName) {
      const newProvider = rosCtx.getProviderById(providerId);
      if (newProvider && rosCtx.mapProviderRosNodes) {
        setProvider(newProvider);
        let srvType = "";
        // Get messageType from node list of the provider
        const nodeList = rosCtx.mapProviderRosNodes.get(providerId);
        nodeList.forEach((node) => {
          node.services.forEach((service) => {
            if (srvType === "" && serviceName === service.name) {
              srvType = service.srvtype;
            }
          });
        });
        if (srvType) {
          setServiceType(srvType);
          const srvStruct = await newProvider.getServiceStruct(srvType);
          if (srvStruct) {
            setServiceStructOrg(srvStruct);
            setServiceStruct(srvStruct);
            setInputElements(
              <InputElements
                key={srvStruct.type}
                messageStruct={srvStruct}
                parentName={srvStruct.type ? srvStruct.type : `${serviceName}[${srvType}]`}
                filterText={searchTerm}
                onCopyToClipboard={onCopyToClipboard}
              />
            );
          }
        }
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [serviceName, providerId, rosCtx.mapProviderRosNodes, rosCtx]);

  // debounced filter callback
  const onUpdateInputElements = useDebounceCallback((searchText) => {
    if (!serviceStruct) return;
    setInputElements(
      <InputElements
        key={serviceStruct.type}
        messageStruct={serviceStruct}
        parentName={serviceStruct.type ? serviceStruct.type : `${serviceName}[${serviceType}]`}
        filterText={searchText}
        onCopyToClipboard={onCopyToClipboard}
      />
    );
  }, 300);

  const handleCallService = async () => {
    setResultError("");
    setResultMessage(null);
    setCallServiceStatus("active");
    setCallServiceDescription("Calling service...");
    setCallServiceIsSubmitting(true);

    // store struct to history if new message
    const messageStr = messageStructToString(serviceStruct, false, false);
    const historyInStruct = history[serviceType];
    if (messageStr !== "{}" && (!historyInStruct || historyInStruct?.length === 0)) {
      updateHistory();
    } else if (historyInStruct) {
      if (historyInStruct.length === 0 || messageStr !== messageStructToString(historyInStruct[0].msg, false, false)) {
        updateHistory();
      }
    }

    console.log(`Call service with: ${messageStr}`);

    const srvResult = await provider.callService(
      new LaunchCallService(serviceName, serviceType, JSON.stringify(serviceStruct))
    );
    if (srvResult) {
      if (srvResult.error_msg) {
        setResultError(srvResult.error_msg);
      }
      if (srvResult.valid) {
        setResultMessage(srvResult.data);
      }
    }
    // close modal
    setTimeoutObj(
      setTimeout(() => {
        setCallServiceIsSubmitting(false);
        setCallServiceDescription("");
        setCallServiceStatus("inactive");
      }, 5000)
    );
  };

  useEffect(() => {
    if (!timeoutObj) return;
    if (resultMessage || resultError) {
      clearTimeout(timeoutObj);
      setTimeoutObj(null);
      setCallServiceIsSubmitting(false);
      setCallServiceDescription("");
      setCallServiceStatus("inactive");
    }
  }, [timeoutObj, resultMessage, resultError]);

  useEffect(() => {
    if (!serviceType) return;
    if (!history) return;
    const historyInStruct = history[serviceType];
    if (historyInStruct) {
      setHistoryLength(historyInStruct.length);
    } else {
      setHistoryLength(0);
    }
  }, [history, serviceType]);

  useEffect(() => {
    if (!serviceType) return;
    if (!serviceStruct) return;
    onUpdateInputElements(searchTerm);
    if (serviceStruct.def.length === 0) {
      handleCallService();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [serviceStruct]);

  // Get topic struct when mounting the component
  useEffect(() => {
    getServiceStructData();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [serviceName]);

  // Update the visible state of input fields on a filter change
  useEffect(() => {
    onUpdateInputElements(searchTerm);
    // eslint-disable-next-line
  }, [searchTerm]);

  // create input mask for an element of the array
  function createHistoryButton(index) {
    return (
      <Button
        key={`history-button-${index}`}
        onClick={(event) => {
          fromHistory(index);
          event.stopPropagation();
        }}
        startIcon={<StorageOutlinedIcon />}
        size="small"
      >
        {index + 1}
      </Button>
    );
  }

  const createJsonView = useMemo(() => {
    return (
      <JSONTree
        data={resultMessage}
        sortObjectKeys={true}
        theme={settingsCtx.get("useDarkMode") ? darkThemeJson : lightThemeJson}
        invertTheme={false}
        hideRoot={true}
        shouldExpandNodeInitially={() => {
          return true;
        }}
      />
    );
  }, [resultMessage, settingsCtx.changed]);

  return (
    <Box height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")}>
      <Stack spacing={1} margin={1}>
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography fontWeight="bold">{serviceName}</Typography>
          <Typography color="grey" fontSize="0.8em">
            {provider?.name()}
          </Typography>
        </Stack>
        {serviceStruct?.def.length > 0 && (
          <Stack direction="row" spacing={1}>
            <SearchBar
              onSearch={(value) => {
                try {
                  // eslint-disable-next-line @typescript-eslint/no-unused-vars
                  const re = new RegExp(value, "i");
                  setSearchTerm(value);
                } catch (error) {
                  // TODO: visualize error
                }
              }}
              placeholder="Filter Fields"
              defaultValue={searchTerm}
            />
          </Stack>
        )}
        <Stack direction="row" spacing={2} display="flex" alignItems="center">
          {historyLength > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              <FormLabel size="small">Publish history</FormLabel>
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                {historyLength > 0 && (
                  <Tooltip title="clear history entries" enterDelay={500}>
                    <Button
                      color="success"
                      onClick={(event) => {
                        setServiceStruct(serviceStructOrg);
                        event.stopPropagation();
                      }}
                      startIcon={<StorageOutlinedIcon />}
                      size="small"
                    >
                      x
                    </Button>
                  </Tooltip>
                )}
                {historyLength > 0 &&
                  Array.from(Array(historyLength).keys()).map((index) => createHistoryButton(index))}
                {historyLength > 0 && (
                  <Tooltip title="remove oldest history entry" enterDelay={500}>
                    <Button
                      color="error"
                      onClick={(event) => {
                        clearHistory();
                        event.stopPropagation();
                      }}
                      startIcon={<DeleteOutlineOutlinedIcon />}
                      size="small"
                    />
                  </Tooltip>
                )}
              </ButtonGroup>
            </Stack>
          )}
        </Stack>
        <Box>
          {callServiceIsSubmitting ? (
            <Stack direction="row" spacing={1}>
              <CircularProgress size="1em" />
              <div>{`${callServiceDescription} with arguments ${messageStructToString(
                serviceStruct,
                false,
                false
              )}`}</div>
            </Stack>
          ) : (
            <Button
              type="submit"
              variant="contained"
              color="success"
              onClick={handleCallService}
              disabled={serviceStruct?.type === undefined}
            >
              Call Service
            </Button>
          )}
        </Box>
        <Divider />
        {serviceStruct && (
          <Stack direction="row" spacing={2}>
            {/* <FormLabel>Message:</FormLabel>
              <Input
                defaultValue={createPublishString(messageStruct, false, true)}
                readOnly
                size="small"
                disabled
                fullWidth
              /> */}
          </Stack>
        )}
        {inputElements}
        <>---</>
        {!serviceStruct && (
          <Alert severity="error" style={{ minWidth: 0 }}>
            <AlertTitle>{`Service definition for ${serviceName}[${serviceType}] not found!`}</AlertTitle>
          </Alert>
        )}
        {resultError && (
          <Alert severity="error" style={{ minWidth: 0 }}>
            <AlertTitle>{`Service call failed with ${resultError}!`}</AlertTitle>
          </Alert>
        )}
        {resultMessage && createJsonView}
      </Stack>
    </Box>
  );
}

ServiceCallerPanel.propTypes = {
  serviceName: PropTypes.string,
  providerId: PropTypes.string,
};

export default ServiceCallerPanel;
