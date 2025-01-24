import { ProviderSelector } from "@/renderer/components";
import { Provider } from "@/renderer/providers";
import { JSONObject } from "@/types";
import { StopCircleOutlined } from "@mui/icons-material";
import ContentCopyOutlinedIcon from "@mui/icons-material/ContentCopyOutlined";
import DeleteOutlineOutlinedIcon from "@mui/icons-material/DeleteOutlineOutlined";
import EditIcon from "@mui/icons-material/Edit";
import StorageOutlinedIcon from "@mui/icons-material/StorageOutlined";
import {
  Alert,
  AlertTitle,
  Autocomplete,
  Box,
  Button,
  ButtonGroup,
  CircularProgress,
  Divider,
  FormControl,
  FormLabel,
  IconButton,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import { forwardRef, useCallback, useContext, useEffect, useState } from "react";
import { colorFromHostname } from "../../../components/UI/Colors";
import SearchBar from "../../../components/UI/SearchBar";
import { LoggingContext } from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { LaunchPublishMessage, rosMessageStructToString, TRosMessageStruct } from "../../../models";
import InputElements from "./MessageDialogPanel/InputElements";

type THistoryItem = {
  rate: string;
  skw: boolean;
  msg: TRosMessageStruct;
};

interface TopicPublishPanelProps {
  topicName?: string;
  topicType?: string;
  providerId?: string;
}

const TopicPublishPanel = forwardRef<HTMLDivElement, TopicPublishPanelProps>(function TopicPublishPanel(props, ref) {
  const { topicName = undefined, topicType = undefined, providerId = undefined } = props;

  const [history, setHistory] = useLocalStorage<{ [msg: string]: THistoryItem[] }>(`MessageStruct:history`, {});
  const [historyLength, setHistoryLength] = useState(0);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [substituteKeywords, setSubstituteKeywords] = useState(true);
  const [editTopicName, setEditTopicName] = useState<boolean>(false);
  const [editMessageType, setEditMessageType] = useState<boolean>(false);
  const [topicNameOptions, setTopicNameOptions] = useState<string[]>([]);
  const [messageTypeOptions, setMessageTypeOptions] = useState<string[]>([]);
  const [currentTopicName, setCurrentTopicName] = useState<string>(topicName || "unknown");
  const [currentProviderId, setCurrentProviderId] = useState<string | undefined>(providerId);
  const [currentMessageType, setCurrentMessageType] = useState<string>(topicType || "unknown");
  const [searchTerm, setSearchTerm] = useState("");
  const [messageStruct, setMessageStruct] = useState<TRosMessageStruct>();
  const [messageStructOrg, setMessageStructOrg] = useState<TRosMessageStruct>();
  const [publishRate, setPublishRate] = useState<string>("once");
  const [provider, setProvider] = useState<Provider | null>(null);
  const [inputElements, setInputElements] = useState<React.ReactNode | null>(null);
  const [hasPublisher, setHasPublisher] = useState<boolean>(false);

  const [startPublisherDescription, setStartPublisherDescription] = useState("");
  const [startPublisherIsSubmitting, setStartPublisherIsSubmitting] = useState(false);
  const publishRateSelections = ["1", "once", "latched"];

  // Make a request to provider and get known message types
  const getAvailableMessageTypes = useCallback(
    async function (): Promise<void> {
      setMessageTypeOptions([]);
      if (!currentProviderId) {
        return;
      }
      const provider = rosCtx.getProviderById(currentProviderId, true);
      if (!provider || !provider.isAvailable()) return;

      const result: string[] = await provider.getRosMessageMessageTypes();
      if (result.length === 0) return;
      setMessageTypeOptions(result);
    },
    [currentProviderId, rosCtx.providers]
  );

  const updateTopicNameOptions = useCallback((): void => {
    setMessageTypeOptions([]);
    if (!currentProviderId) {
      return;
    }
    const provider = rosCtx.getProviderById(currentProviderId, true);
    if (!provider || !provider.isAvailable()) return;
    setTopicNameOptions(provider.rosTopics.map((topic) => topic.name));
  }, []);

  useEffect(() => {
    updateTopicNameOptions();
    getAvailableMessageTypes();
  }, [rosCtx.mapProviderRosNodes, currentProviderId]);

  // get item history after the history was loaded
  const fromHistory = useCallback(
    (index: number) => {
      const historyInStruct = history[currentMessageType];
      if (historyInStruct) {
        const historyItem = index && index < historyInStruct.length ? historyInStruct[index] : historyInStruct[0];
        setPublishRate(historyItem.rate);
        setSubstituteKeywords(historyItem.skw);
        setMessageStruct(JSON.parse(JSON.stringify(historyItem.msg)));
      }
    },
    [history, setPublishRate, setSubstituteKeywords, setMessageStruct]
  );

  // get item history after the history was loaded
  const updateHistory = useCallback(() => {
    if (!messageStruct) return;
    let historyInStruct = history[currentMessageType];
    let hasType = true;
    if (!historyInStruct) {
      hasType = false;
      historyInStruct = [];
    }
    historyInStruct.unshift({
      rate: publishRate,
      skw: substituteKeywords,
      msg: messageStruct,
    } as THistoryItem);
    if (historyInStruct.length > 5) {
      historyInStruct.pop();
    }
    if (historyInStruct.length > 0) {
      history[currentMessageType] = historyInStruct;
    } else if (hasType) {
      delete history[currentMessageType];
    }
    setHistory(history);
  }, [history, messageStruct, setHistory, publishRate]);

  // get item history after the history was loaded
  const clearHistory = useCallback(() => {
    if (!messageStruct) return;
    const historyInStruct = history[currentMessageType];
    if (historyInStruct && historyInStruct.length > 0) {
      historyInStruct.pop();
      setHistory(history);
      setHistoryLength(historyInStruct.length);
    }
  }, [messageStruct, history]);

  // create string from message struct and copy it to clipboard
  const onCopyToClipboard = useCallback((): void => {
    if (!messageStruct) return;
    const json: string = rosMessageStructToString(messageStruct, false, false) as string;
    navigator.clipboard.writeText(`${currentTopicName} ${currentMessageType} '${json}'`);
    logCtx.success(`message publish object copied!`);
  }, [messageStruct, currentTopicName, currentMessageType]);

  const updateMessageTypeFromTopic = useCallback(async () => {
    const newProvider = rosCtx.getProviderById(currentProviderId || "", true);
    if (newProvider) {
      setProvider(newProvider);
      let msgType = "";
      if (currentTopicName) {
        if (msgType.length === 0 || msgType === "unknown") {
          // Get messageType from node list of the provider
          newProvider.rosNodes.forEach((node) => {
            if (node.providerId === currentProviderId) {
              node.subscribers?.forEach((topic) => {
                if (msgType === "" && currentTopicName === topic.name) {
                  msgType = topic.msg_type;
                }
              });
              if (msgType === "") {
                node.publishers?.forEach((topic) => {
                  if (msgType === "" && currentTopicName === topic.name) {
                    msgType = topic.msg_type;
                  }
                });
              }
            }
          });
        }
      }
      if (msgType && msgType !== "unknown") {
        setCurrentMessageType(msgType);
      } else {
        setCurrentMessageType("unknown");
        setInputElements(null);
        setMessageStruct(undefined);
      }
    } else {
      setCurrentMessageType("unknown");
      setInputElements(null);
      setMessageStruct(undefined);
    }
  }, [currentTopicName, currentProviderId, rosCtx]);

  const getTopicStructData = useCallback(async () => {
    if (currentMessageType && currentMessageType !== "unknown") {
      const newProvider = rosCtx.getProviderById(currentProviderId || "", true);
      if (newProvider) {
        const msgStruct = await newProvider.getMessageStruct(currentMessageType);
        if (msgStruct) {
          setMessageStructOrg(msgStruct.data);
          setMessageStruct(msgStruct.data);
          setInputElements(
            <InputElements
              key={msgStruct.data.type}
              messageStruct={msgStruct.data}
              parentName={msgStruct.data.type ? msgStruct.data.type : `${currentTopicName}[${currentMessageType}]`}
              filterText={searchTerm}
              showRoot={false}
              onCopyToClipboard={onCopyToClipboard}
            />
          );
        }
      }
    }
  }, [currentMessageType, currentProviderId, rosCtx]);

  // debounced filter callback
  const updateInputElements = useCallback(
    (searchText: string): void => {
      if (messageStruct) {
        setInputElements(
          <InputElements
            key={messageStruct.type}
            messageStruct={messageStruct}
            parentName={messageStruct.type ? messageStruct.type : `${topicName}[${currentMessageType}]`}
            filterText={searchText}
            showRoot={false}
            onCopyToClipboard={onCopyToClipboard}
          />
        );
      } else {
        setInputElements(null);
      }
    },
    [messageStruct, currentMessageType, currentTopicName]
  );

  function stopPublisher(): void {
    if (provider && topicName) {
      const publisherName = `/_mas_publisher${topicName.replaceAll("/", "_")}`;
      provider.rosNodes.forEach(async (node) => {
        if (node.name === publisherName) {
          await provider.stopNode(node.id);
        }
      });
    }
  }

  useEffect(() => {
    if (editMessageType) return;
    if (!currentMessageType || currentMessageType === "unknown") return;
    if (history) {
      const historyInStruct = history[currentMessageType];
      if (historyInStruct) {
        setHistoryLength(historyInStruct.length);
      } else {
        setHistoryLength(0);
      }
    }
  }, [history, currentMessageType, editMessageType]);

  useEffect(() => {
    if (editTopicName) return;
    updateMessageTypeFromTopic();
  }, [currentTopicName, editTopicName]);

  useEffect(() => {
    if (editMessageType) return;
    if (!currentMessageType || currentMessageType === "unknown") return;
    getTopicStructData();
  }, [currentMessageType, editMessageType]);

  useEffect(() => {
    if (!messageStruct) return;
    updateInputElements(searchTerm);
  }, [messageStruct]);

  // Update the visible state of input fields on a filter change
  useEffect(() => {
    updateInputElements(searchTerm);
  }, [searchTerm]);

  useEffect(() => {
    if (editMessageType) {
      getAvailableMessageTypes();
    }
  }, [editMessageType]);

  useEffect(() => {
    if (provider && topicName) {
      const publisherName = `/_mas_publisher${topicName.replaceAll("/", "_")}`;
      setHasPublisher(
        provider.rosNodes.filter((node) => {
          return node.name === publisherName;
        }).length > 0
      );
    }
  }, [provider, rosCtx.mapProviderRosNodes, topicName]);

  async function handleStartPublisher(): Promise<void> {
    if (!messageStruct) return;
    setStartPublisherDescription("Starting publisher...");
    setStartPublisherIsSubmitting(true);

    // store struct to history if new message
    const messageStr = rosMessageStructToString(messageStruct, false, false);
    const historyInStruct = history[currentMessageType];
    if (messageStr !== "{}" && (!historyInStruct || historyInStruct?.length === 0)) {
      updateHistory();
    } else if (historyInStruct) {
      if (
        historyInStruct.length === 0 ||
        messageStr !== rosMessageStructToString(historyInStruct[0].msg, false, false)
      ) {
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
    await provider?.publishMessage(
      new LaunchPublishMessage(
        currentTopicName,
        currentMessageType,
        messageStruct as JSONObject,
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
    }, 3000);
  }

  useEffect(() => {
    if (hasPublisher) {
      setStartPublisherIsSubmitting(false);
      setStartPublisherDescription("");
    }
  }, [hasPublisher]);

  // create input mask for an element of the array
  function createHistoryButton(index: number): JSX.Element {
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

  const getHostStyle = useCallback(
    function getHostStyle(): object {
      const providerName = provider?.name();
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          flexGrow: 1,
          borderTopStyle: "solid",
          borderTopColor: colorFromHostname(providerName),
          borderTopWidth: "0.3em",
          backgroundColor: settingsCtx.get("backgroundColor") as string,
        };
      }
      return { flexGrow: 1, backgroundColor: settingsCtx.get("backgroundColor") as string };
    },
    [provider, settingsCtx.changed]
  );

  return (
    <Box ref={ref} height="100%" overflow="auto" alignItems="center" sx={getHostStyle()}>
      <Stack spacing={1} margin={0.5}>
        <Stack direction="row" spacing={1}>
          {messageStruct && (messageStruct.def || []).length > 0 && (
            <SearchBar
              onSearch={(value) => {
                setSearchTerm(value);
              }}
              placeholder="Filter Fields"
              defaultValue={searchTerm}
              fullWidth
            />
          )}
        </Stack>
        <Stack direction="row" alignItems="center" spacing={1}>
          {editTopicName ? (
            <Autocomplete
              key={`autocomplete-topic-name`}
              size="small"
              fullWidth
              autoHighlight
              clearOnEscape
              disableListWrap
              handleHomeEndKeys={false}
              noOptionsText="No topics found"
              options={topicNameOptions}
              getOptionLabel={(option) => option}
              // This prevents warnings on invalid autocomplete values
              value={currentTopicName}
              renderInput={(params) => <TextField {...params} autoFocus label="topic name" variant="standard" />}
              onChange={(_event, newNameValue) => {
                setCurrentTopicName(newNameValue ? newNameValue : "");
              }}
              onInputChange={(_event, newInputValue) => {
                setCurrentTopicName(newInputValue ? newInputValue : "");
              }}
              isOptionEqualToValue={(option, value) => {
                return value === undefined || value === "" || option === value;
              }}
              onKeyDown={(event: React.KeyboardEvent) => {
                if (event.key === "Enter") {
                  setEditTopicName(false);
                } else if (event.key === "Escape") {
                  setCurrentTopicName(topicType || "unknown");
                  setEditTopicName(false);
                }
              }}
              onBlur={() => {
                setEditTopicName(false);
              }}
              onWheel={(event) => {
                // scroll through the options using mouse wheel
                let newIndex = -1;
                topicNameOptions.forEach((value, index) => {
                  if (value === (event.target as HTMLInputElement).value) {
                    if (event.deltaY > 0) {
                      newIndex = index + 1;
                    } else {
                      newIndex = index - 1;
                    }
                  }
                });
                if (newIndex < 0) newIndex = topicNameOptions.length - 1;
                else if (newIndex > topicNameOptions.length - 1) newIndex = 0;
                setCurrentMessageType(topicNameOptions[newIndex]);
              }}
            />
          ) : (
            <>
              <Typography fontWeight="bold">{currentTopicName}</Typography>
              <IconButton onClick={() => setEditTopicName(true)}>
                <EditIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
            </>
          )}
        </Stack>
        <Stack direction="row" alignItems="center" spacing={1}>
          <FormControl sx={{ m: 1, width: "100%" }} variant="standard">
            <ProviderSelector
              defaultProvider={provider?.id || ""}
              setSelectedProvider={(provId) => setCurrentProviderId(provId)}
            />
          </FormControl>
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
            onInputChange={(_event, newValue) => {
              setPublishRate(newValue);
            }}
            onChange={(_event, newValue) => {
              setPublishRate(newValue ? newValue : "1");
            }}
            onWheel={(event) => {
              // scroll through the options using mouse wheel
              let newIndex = -1;
              publishRateSelections.forEach((value, index) => {
                if (value === (event.target as HTMLInputElement).value) {
                  if (event.deltaY > 0) {
                    newIndex = index + 1;
                  } else {
                    newIndex = index - 1;
                  }
                }
              });
              if (newIndex < 0) newIndex = publishRateSelections.length - 1;
              else if (newIndex > publishRateSelections.length - 1) newIndex = 0;
              setPublishRate(publishRateSelections[newIndex]);
            }}
          />
          {/* <FormGroup>
            <FormControlLabel
              control={
                <Checkbox
                  id={`${topicName?.replace("/", "-")}-substitute-keywords`}
                  checked={substituteKeywords}
                  onChange={(event) => {
                    setSubstituteKeywords(event.target.checked);
                  }}
                />
              }
              label="Substitute keywords"
            />
          </FormGroup> */}
          {historyLength > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              <FormLabel sx={{ fontSize: "0.8em", lineHeight: "1em" }}>Publish history</FormLabel>
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                {historyLength > 0 && (
                  <Tooltip title="reset values" enterDelay={500}>
                    <Button
                      color="success"
                      onClick={(event) => {
                        setMessageStruct(messageStructOrg);
                        setPublishRate("1");
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
              <div>{`${startPublisherDescription} ${messageStruct ? rosMessageStructToString(messageStruct, false, false) : ""}`}</div>
            </Stack>
          ) : (
            <Button
              type="submit"
              variant="contained"
              color="success"
              onClick={() => handleStartPublisher()}
              disabled={messageStruct?.type === undefined}
            >
              Start Publisher
            </Button>
          )}
          {hasPublisher && (
            <Tooltip title="Stop running publisher node" placement="bottom">
              <IconButton
                sx={{
                  padding: "0.8em",
                }}
                onClick={() => stopPublisher()}
              >
                <StopCircleOutlined sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )}
        </Box>
        <Divider />
        <Stack direction="row" alignItems="center" spacing={1}>
          {editMessageType ? (
            <Autocomplete
              key={`autocomplete-publisher-type`}
              size="small"
              fullWidth
              autoHighlight
              clearOnEscape
              disableListWrap
              handleHomeEndKeys={false}
              noOptionsText="No message types found"
              options={messageTypeOptions}
              getOptionLabel={(option) => option}
              // This prevents warnings on invalid autocomplete values
              value={currentMessageType}
              renderInput={(params) => <TextField {...params} autoFocus label="message type" variant="standard" />}
              onChange={(_event, newNameValue) => {
                setCurrentMessageType(newNameValue ? newNameValue : "");
              }}
              onInputChange={(_event, newInputValue) => {
                setCurrentMessageType(newInputValue ? newInputValue : "");
              }}
              isOptionEqualToValue={(option, value) => {
                return value === undefined || value === "" || option === value;
              }}
              onKeyDown={(event: React.KeyboardEvent) => {
                if (event.key === "Enter") {
                  setEditMessageType(false);
                } else if (event.key === "Escape") {
                  setCurrentMessageType(topicType || "unknown");
                  setEditMessageType(false);
                }
              }}
              onBlur={() => {
                setEditMessageType(false);
              }}
              onWheel={(event) => {
                // scroll through the options using mouse wheel
                let newIndex = -1;
                messageTypeOptions.forEach((value, index) => {
                  if (value === (event.target as HTMLInputElement).value) {
                    if (event.deltaY > 0) {
                      newIndex = index + 1;
                    } else {
                      newIndex = index - 1;
                    }
                  }
                });
                if (newIndex < 0) newIndex = messageTypeOptions.length - 1;
                else if (newIndex > messageTypeOptions.length - 1) newIndex = 0;
                setCurrentMessageType(messageTypeOptions[newIndex]);
              }}
            />
          ) : (
            <Stack direction="row">
              <Typography fontWeight="bold">{currentMessageType}</Typography>
              <IconButton
                onClick={() => {
                  setInputElements(null);
                  setEditMessageType(true);
                }}
              >
                <EditIcon sx={{ fontSize: "inherit" }} />
              </IconButton>
              <Tooltip title="Copy topic name, type and data fields" placement="bottom">
                <IconButton
                  color="default"
                  onClick={(event) => {
                    onCopyToClipboard();
                    event?.stopPropagation();
                  }}
                  size="small"
                >
                  <ContentCopyOutlinedIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            </Stack>
          )}
        </Stack>

        {inputElements}
        {!messageStruct && (
          <Alert severity="error" style={{ minWidth: 0 }}>
            <AlertTitle>{`Message definition for ${topicName}[${currentMessageType}] not found!`}</AlertTitle>
          </Alert>
        )}
      </Stack>
    </Box>
  );
});

export default TopicPublishPanel;
