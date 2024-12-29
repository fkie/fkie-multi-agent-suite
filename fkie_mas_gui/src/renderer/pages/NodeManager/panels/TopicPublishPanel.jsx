import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import StorageOutlinedIcon from "@mui/icons-material/StorageOutlined";
import {
  Alert,
  AlertTitle,
  Autocomplete,
  Box,
  Button,
  ButtonGroup,
  Checkbox,
  CircularProgress,
  Divider,
  FormControlLabel,
  FormGroup,
  FormLabel,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useState } from "react";
import { colorFromHostname } from "../../../components/UI/Colors";
import SearchBar from "../../../components/UI/SearchBar";
import { LoggingContext } from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { LaunchPublishMessage } from "../../../models";
import InputElements from "./MessageDialogPanel/InputElements";

function TopicPublishPanel({ topicName = null, providerId = "" }) {
  const [history, setHistory] = useLocalStorage(`MessageStruct:history`, {});
  const [historyLength, setHistoryLength] = useState(0);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [substituteKeywords, setSubstituteKeywords] = useState(true);
  const [messageType, setMessageType] = useState("");
  const [searchTerm, setSearchTerm] = useState("");
  const [messageStruct, setMessageStruct] = useState(null);
  const [messageStructOrg, setMessageStructOrg] = useState(null);
  const [publishRate, setPublishRate] = useState("once");
  const [provider, setProvider] = useState(null);
  const [inputElements, setInputElements] = useState(null);

  const [startPublisherDescription, setStartPublisherDescription] = useState("");
  const [startPublisherIsSubmitting, setStartPublisherIsSubmitting] = useState(false);

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
      if (typeof value === "string") {
        result = ("yes", "true", "t", "y", "1").includes(value.toLoverCase());
      } else if (!value) {
        result = false;
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
          if (field.value || typeof field.value === "boolean" || field.type.startsWith("bool")) {
            if (field.is_array) {
              const values = field.value?.split(/\s*,\s*/) || [];
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
    const historyInStruct = history[messageType];
    if (historyInStruct) {
      const historyItem = index && index < historyInStruct.length ? historyInStruct[index] : historyInStruct[0];
      setPublishRate(historyItem.rate);
      setSubstituteKeywords(historyItem.skw);
      setMessageStruct(JSON.parse(JSON.stringify(historyItem.msg)));
    }
  }, 100);

  // get item history after the history was loaded
  const updateHistory = useDebounceCallback(() => {
    if (!messageStruct) return;
    let historyInStruct = history[messageType];
    let hasType = true;
    if (!historyInStruct) {
      hasType = false;
      historyInStruct = [];
    }
    historyInStruct.unshift({
      rate: publishRate,
      skw: substituteKeywords,
      msg: messageStruct,
    });
    if (historyInStruct.length > 5) {
      historyInStruct.pop();
    }
    if (historyInStruct.length > 0) {
      history[messageType] = historyInStruct;
    } else if (hasType) {
      history.delete(messageType);
    }
    setHistory(history);
  }, 100);

  // get item history after the history was loaded
  const clearHistory = useDebounceCallback(() => {
    if (!messageStruct) return;
    const historyInStruct = history[messageType];
    if (historyInStruct && historyInStruct.length > 0) {
      historyInStruct.pop();
      setHistory(history);
      setHistoryLength(historyInStruct.length);
    }
  }, 300);

  // create string from message struct and copy it to clipboard
  const onCopyToClipboard = useDebounceCallback(() => {
    if (!messageStruct) return;
    const json = messageStructToString(messageStruct, false, true);
    navigator.clipboard.writeText(json);
    logCtx.success(`${json} copied!`);
  }, 300);

  const getTopicStructData = useCallback(async () => {
    if (topicName) {
      const newProvider = rosCtx.getProviderById(providerId);
      if (newProvider) {
        setProvider(newProvider);
        let msgType = "";
        // Get messageType from node list of the provider
        newProvider.rosNodes.forEach((node) => {
          if (node.providerId === providerId) {
            node.subscribers.forEach((topic) => {
              if (msgType === "" && topicName === topic.name) {
                msgType = topic.msgtype;
              }
            });
            if (msgType === "") {
              node.publishers.forEach((topic) => {
                if (msgType === "" && topicName === topic.name) {
                  msgType = topic.msgtype;
                }
              });
            }
          }
        });
        if (msgType) {
          setMessageType(msgType);
          const msgStruct = await newProvider.getMessageStruct(msgType);
          if (msgStruct) {
            setMessageStructOrg(msgStruct.data);
            setMessageStruct(msgStruct.data);
            setInputElements(
              <InputElements
                key={msgStruct.data.type}
                messageStruct={msgStruct.data}
                parentName={msgStruct.data.type ? msgStruct.data.type : `${topicName}[${msgType}]`}
                filterText={searchTerm}
                onCopyToClipboard={onCopyToClipboard}
              />
            );
          }
        }
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topicName, providerId, rosCtx]);

  // debounced filter callback
  const onUpdateInputElements = useDebounceCallback((searchText) => {
    if (!messageStruct) return;
    setInputElements(
      <InputElements
        key={messageStruct.type}
        messageStruct={messageStruct}
        parentName={messageStruct.type ? messageStruct.type : `${topicName}[${messageType}]`}
        filterText={searchText}
        onCopyToClipboard={onCopyToClipboard}
      />
    );
  }, 300);

  useEffect(() => {
    if (!messageType) return;
    if (!history) return;
    const historyInStruct = history[messageType];
    if (historyInStruct) {
      setHistoryLength(historyInStruct.length);
    } else {
      setHistoryLength(0);
    }
  }, [history, messageType]);

  useEffect(() => {
    if (!messageType) return;
    if (!messageStruct) return;
    onUpdateInputElements(searchTerm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [messageStruct]);

  // Get topic struct when mounting the component
  useEffect(() => {
    getTopicStructData();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topicName]);

  // Update the visible state of input fields on a filter change
  useEffect(() => {
    onUpdateInputElements(searchTerm);
    // eslint-disable-next-line
  }, [searchTerm]);

  const handleStartPublisher = async () => {
    setStartPublisherDescription("Starting publisher...");
    setStartPublisherIsSubmitting(true);

    // store struct to history if new message
    const messageStr = messageStructToString(messageStruct, false, false);
    const historyInStruct = history[messageType];
    if (messageStr !== "{}" && (!historyInStruct || historyInStruct?.length === 0)) {
      updateHistory();
    } else if (historyInStruct) {
      if (historyInStruct.length === 0 || messageStr !== messageStructToString(historyInStruct[0].msg, false, false)) {
        updateHistory();
      }
    }

    console.log(`Start publisher with rate '${publishRate}' and message: ${messageStr}`);

    const once = publishRate === "once";
    const latched = publishRate === "latched";
    let rate = 0.0;
    if (!once && !latched) {
      rate = parseFloat(publishRate);
    }
    await provider.publishMessage(
      new LaunchPublishMessage(
        topicName,
        messageType,
        JSON.stringify(messageStruct),
        rate,
        once,
        latched,
        false,
        false,
        substituteKeywords
      )
    );
    // close modal
    setTimeout(() => {
      setStartPublisherIsSubmitting(false);
      setStartPublisherDescription("");
    }, 5000);
  };

  const publishRateSelections = ["once", "latched", "1"];

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

  const getHostStyle = () => {
    const providerName = provider?.name();
    if (providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        borderTopStyle: "solid",
        borderTopColor: colorFromHostname(providerName),
        borderTopWidth: "0.3em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  return (
    <Box height="100%" overflow="auto" backgroundColor={settingsCtx.get("backgroundColor")} style={getHostStyle()}>
      <Stack spacing={1} margin={0.5}>
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography fontWeight="bold">{topicName}</Typography>
          <Typography color="grey" fontSize="0.8em">
            {provider?.name()}
          </Typography>
        </Stack>
        <Stack direction="row" spacing={1}>
          {messageStruct?.def.length > 0 && (
            <SearchBar
              onSearch={(value) => {
                try {
                  // eslint-disable-next-line @typescript-eslint/no-unused-vars
                  const re = new RegExp(value, "i");
                  setSearchTerm(value);
                } catch (error) {
                  console.log(`error while search: ${error}`);
                  // TODO: visualize error
                }
              }}
              placeholder="Filter Fields"
              defaultValue={searchTerm}
              fullWidth
            />
          )}
        </Stack>
        <Stack direction="row" spacing={2} display="flex" alignItems="center">
          <Autocomplete
            id={`publish-rate-${topicName}`}
            handleHomeEndKeys={false}
            freeSolo
            options={publishRateSelections}
            size="small"
            sx={{ width: 150 }}
            renderInput={(params) => <TextField {...params} label="Publish rate" />}
            defaultValue={publishRateSelections[0]}
            inputValue={publishRate || ""}
            onInputChange={(event, newValue) => {
              setPublishRate(newValue);
            }}
            onChange={(event, newValue) => {
              setPublishRate(newValue);
            }}
          />
          <FormGroup>
            <FormControlLabel
              control={
                <Checkbox
                  id={`${topicName.replace("/", "-")}-substitute-keywords`}
                  checked={substituteKeywords}
                  onChange={(event) => {
                    setSubstituteKeywords(event.target.checked);
                  }}
                />
              }
              label="Substitute keywords"
            />
          </FormGroup>
          {historyLength > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              <FormLabel size="small">Publish history</FormLabel>
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                {historyLength > 0 && (
                  <Tooltip title="clear history entries" enterDelay={500}>
                    <Button
                      color="success"
                      onClick={(event) => {
                        setMessageStruct(messageStructOrg);
                        setPublishRate("once");
                        setSubstituteKeywords(true);
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
          {startPublisherIsSubmitting ? (
            <Stack direction="row" spacing={1}>
              <CircularProgress size="1em" />
              <div>{`${startPublisherDescription} ${messageStructToString(messageStruct, false, false)}`}</div>
            </Stack>
          ) : (
            <Button
              type="submit"
              variant="contained"
              color="success"
              onClick={handleStartPublisher}
              disabled={messageStruct?.type === undefined}
            >
              Start Publisher
            </Button>
          )}
        </Box>
        <Divider />
        {messageStruct && (
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
        {!messageStruct && (
          <Alert severity="error" style={{ minWidth: 0 }}>
            <AlertTitle>{`Message definition for ${topicName}[${messageType}] not found!`}</AlertTitle>
          </Alert>
        )}
      </Stack>
    </Box>
  );
}

TopicPublishPanel.propTypes = {
  topicName: PropTypes.string,
  providerId: PropTypes.string,
};

export default TopicPublishPanel;
