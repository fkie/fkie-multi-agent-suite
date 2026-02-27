import { StopCircleOutlined } from "@mui/icons-material";
import ContentCopyOutlinedIcon from "@mui/icons-material/ContentCopyOutlined";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import EditIcon from "@mui/icons-material/Edit";
import EditNoteIcon from "@mui/icons-material/EditNote";
import MonitorHeartIcon from "@mui/icons-material/MonitorHeart";
import RemoveCircleOutlineIcon from "@mui/icons-material/RemoveCircleOutline";
import StarIcon from "@mui/icons-material/Star";
import StarOutlineIcon from "@mui/icons-material/StarOutline";
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
  IconButton,
  Slider,
  Stack,
  TextField,
  ToggleButton,
  Tooltip,
  Typography,
} from "@mui/material";
import { useCallback, useContext, useEffect, useState } from "react";

import { colorFromHostname } from "@/renderer/components/UI/Colors";
import ProviderSelector from "@/renderer/components/UI/ProviderSelector";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { MsgHistoryContext, TMsgHistoryEntry, useMsgHistory } from "@/renderer/context/MsgHistoryContext";
import { RosContext } from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { LaunchPublishMessage, rosMessageStructToString, RosQos, TRosMessageStruct } from "@/renderer/models";
import { qosFromJson } from "@/renderer/models/RosQos";
import { Provider } from "@/renderer/providers";
import { JSONObject } from "@/types";
import InputElements from "./MessageDialogPanel/InputElements";

type THistoryItem = {
  id: number;
  rate: string;
  skw: boolean;
  msg: TRosMessageStruct;
};

interface TopicPublishPanelProps {
  topicName?: string;
  topicType?: string;
  providerId?: string;
}

export default function TopicPublishPanel(props: TopicPublishPanelProps): JSX.Element {
  const { topicName = undefined, topicType = undefined, providerId = undefined } = props;
  const [oldHistory, setOldHistory] = useLocalStorage<{ [msg: string]: THistoryItem[] }>("MessageStruct:history", {});
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const historyCtx = useContext(MsgHistoryContext);
  const [maxHistoryLength, setMaxHistoryLength] = useState(0);
  const { historyByType, setMaxEntries, ensureLoaded } = useMsgHistory();
  const [substituteKeywords, setSubstituteKeywords] = useState(true);
  const [editTopicName, setEditTopicName] = useState<boolean>(false);
  const [editMessageType, setEditMessageType] = useState<boolean>(false);
  const [obtainingMsgStruct, setObtainingMsgStruct] = useState<boolean>(false);
  const [topicNameOptions, setTopicNameOptions] = useState<string[]>([]);
  const [messageTypeOptions, setMessageTypeOptions] = useState<string[]>([]);
  const [currentTopicName, setCurrentTopicName] = useState<string>(topicName || "unknown");
  const [currentProviderId, setCurrentProviderId] = useState<string>(providerId || "");
  const [currentMessageType, setCurrentMessageType] = useState<string>(topicType || "unknown");
  const [searchTerm, setSearchTerm] = useState("");
  const [messageStruct, setMessageStruct] = useState<TRosMessageStruct>();
  const [messageStructOrg, setMessageStructOrg] = useState<TRosMessageStruct>();
  const [publishRate, setPublishRate] = useState<string>("1");
  const [useSimTime, setUseSimTime] = useState<boolean>(false);
  const [provider, setProvider] = useState<Provider | null>(null);
  const [inputElements, setInputElements] = useState<React.ReactNode | null>(null);
  const [hasPublisher, setHasPublisher] = useState<boolean>(false);
  const [historyEditMode, setHistoryEditMode] = useState<boolean>(false);

  const [startPublisherDescription, setStartPublisherDescription] = useState("");
  const [startPublisherIsSubmitting, setStartPublisherIsSubmitting] = useState(false);
  const publishRateSelections = ["1", "once", "latched"];

  // migrate old history
  useEffect(() => {
    if (!oldHistory) return;
    const historyList = oldHistory[currentMessageType];
    if (!historyList) return;
    for (const item of historyList) {
      historyCtx?.addEntry({
        messageType: currentMessageType,
        name: "",
        favorite: false,
        rate: item.rate,
        skw: item.skw,
        data: item.msg,
        createdAt: Date.now(),
      });
    }
    delete oldHistory[currentMessageType];
    setOldHistory(oldHistory);
  }, [oldHistory]);

  useEffect(() => {
    if (currentProviderId) {
      const provider = rosCtx.getProviderById(currentProviderId, true);
      if (!provider || !provider.isAvailable()) {
        setProvider(null);
      } else {
        setProvider(provider);
      }
    }
  }, [currentProviderId]);

  useEffect(() => {
    if (!historyCtx) return;
    setMaxHistoryLength(historyCtx.maxEntries);
  }, [historyCtx]);

  useEffect(() => {
    if (historyByType[currentMessageType]?.length === 0) setHistoryEditMode(false);
  }, [historyByType[currentMessageType]]);

  useEffect(() => {
    if (!provider) return;
    updateTopicNameOptions();
    getAvailableMessageTypes();
  }, [provider]);

  useEffect(() => {
    setMaxEntries(maxHistoryLength);
  }, [maxHistoryLength]);

  useEffect(() => {
    ensureLoaded(currentMessageType);
  }, [currentMessageType]);

  // Make a request to provider and get known message types
  const getAvailableMessageTypes = useCallback(async (): Promise<void> => {
    if (!provider || !provider.isAvailable()) return;
    const result: string[] = await provider.getRosMessageMessageTypes();
    if (result.length === 0) return;
    setMessageTypeOptions(result);
  }, [provider, rosCtx.providers]);

  const updateTopicNameOptions = useCallback((): void => {
    if (!provider) return;
    setTopicNameOptions(provider.rosTopics.map((topic) => topic.name));
  }, [provider, rosCtx.providers]);

  // get item history after the history was loaded
  const fromHistory = useCallback(
    (entry: TMsgHistoryEntry) => {
      const { rate, skw, data } = entry;
      setPublishRate(rate);
      setSubstituteKeywords(skw);
      setMessageStruct(structuredClone(data));
    },
    [setPublishRate, setSubstituteKeywords, setMessageStruct]
  );

  // add new item to the history
  const updateHistory = useCallback(async () => {
    if (!messageStruct) return;

    await historyCtx?.addEntry({
      messageType: currentMessageType,
      name: "",
      favorite: false,
      rate: publishRate,
      skw: substituteKeywords,
      data: messageStruct,
      createdAt: Date.now(),
    });
  }, [messageStruct, publishRate, substituteKeywords, currentMessageType, historyCtx]);

  // create string from message struct and copy it to clipboard
  const onCopyToClipboard = useCallback((): void => {
    if (!messageStruct) return;
    const json: string = rosMessageStructToString(messageStruct, false, false) as string;
    const qos: RosQos | undefined = findQoSFromSub();
    navigator.clipboard.writeText(
      `${currentTopicName} ${qos ? qosFromJson(qos).toString() : ""} ${currentMessageType} '${json}'`
    );
    logCtx.success("message publish object copied!", "", "message publish object copied");
  }, [messageStruct, currentTopicName, currentMessageType]);

  const updateMessageTypeFromTopic = useCallback(async () => {
    if (provider) {
      let msgType = "";
      if (currentTopicName) {
        if (msgType.length === 0 || msgType === "unknown") {
          // Get messageType from node list of the provider
          for (const node of provider.rosNodes) {
            if (node.providerId === provider.id) {
              for (const topic of node.subscribers || []) {
                if (msgType === "" && currentTopicName === topic.name) {
                  msgType = topic.msg_type;
                }
              }
              if (msgType === "") {
                for (const topic of node.publishers || []) {
                  if (msgType === "" && currentTopicName === topic.name) {
                    msgType = topic.msg_type;
                  }
                }
              }
            }
          }
        }
      }
      if (msgType && msgType !== "unknown") {
        setCurrentMessageType(msgType);
        // } else {
        //   setCurrentMessageType("unknown");
        //   setInputElements(null);
        //   setMessageStruct(undefined);
      }
      // } else {
      //   setCurrentMessageType("unknown");
      //   setInputElements(null);
      //   setMessageStruct(undefined);
    }
  }, [currentTopicName, provider, currentMessageType, rosCtx]);

  const getTopicStructData = useCallback(async () => {
    if (currentMessageType && currentMessageType !== "unknown") {
      if (provider) {
        setObtainingMsgStruct(true);
        const msgStruct = await provider.getMessageStruct(currentMessageType);
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
            />
          );
          setObtainingMsgStruct(false);
        }
      }
    }
  }, [currentMessageType, provider, rosCtx]);

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
          />
        );
      } else {
        setInputElements(null);
      }
    },
    [messageStruct, currentMessageType, currentTopicName]
  );

  function stopPublisher(): void {
    if (provider && currentTopicName) {
      const publisherName = getPublisherName();
      // biome-ignore lint/complexity/noForEach: <explanation>
      provider.rosNodes.forEach(async (node) => {
        if (node.name === publisherName) {
          const result = await provider.stopNode(node.id);
          if (result.result) {
            logCtx.success(`Stopped publisher ${publisherName}`, "", `Stopped publisher ${publisherName}`);
          } else {
            logCtx.warn(`Failed to stop publisher ${publisherName}`, result.message, `Failed to stop ${publisherName}`);
          }
        }
      });
    }
  }

  useEffect(() => {
    if (editTopicName) return;
    updateMessageTypeFromTopic();
  }, [currentTopicName, editTopicName]);

  useEffect(() => {
    if (editMessageType) return;
    if (!provider) return;
    if (!currentMessageType || currentMessageType === "unknown") return;
    getTopicStructData();
  }, [currentMessageType, editMessageType, provider]);

  useEffect(() => {
    if (!messageStruct) return;
    updateInputElements(searchTerm);
  }, [messageStruct]);

  // Update the visible state of input fields on a filter change
  useEffect(() => {
    updateInputElements(searchTerm);
  }, [searchTerm]);

  useEffect(() => {
    if (provider && currentTopicName) {
      const publisherName = getPublisherName();
      setHasPublisher(
        provider.rosNodes.filter((node) => {
          return node.name === publisherName;
        }).length > 0
      );
    }
  }, [provider, rosCtx.mapProviderRosNodes, currentTopicName]);

  function findQoSFromSub(): RosQos | undefined {
    // find first available subscriber with QoS
    let qos: RosQos | undefined = undefined;
    const topics = provider?.rosTopics || [];
    for (const topic of topics) {
      if (topic.name === currentTopicName) {
        const subscribers = topic.subscriber || [];
        for (const sub of subscribers) {
          if (qos === undefined && sub.qos) {
            qos = sub.qos;
          }
        }
      }
    }
    if (!qos) {
      for (const topic of topics) {
        if (topic.name === currentTopicName) {
          const publishers = topic.publisher || [];
          for (const pub of publishers) {
            if (qos === undefined && pub.qos) {
              qos = pub.qos;
            }
          }
        }
      }
    }
    return qos;
  }

  function getPublisherName(): string {
    const prefix = currentTopicName.startsWith("/") ? "" : "_";
    return `/_mas_publisher${prefix}${currentTopicName.replaceAll("/", "_")}`;
  }

  async function handleStartPublisher(): Promise<void> {
    if (!messageStruct) return;
    setStartPublisherDescription("Starting publisher...");
    setStartPublisherIsSubmitting(true);

    // store struct to history if new message
    const messageStr = rosMessageStructToString(messageStruct, false, false);
    if (messageStr !== "{}") {
      const historyList = historyByType[currentMessageType] ?? [];
      const exists = historyList.some((item) => rosMessageStructToString(item.data, false, false) === messageStr);
      if (!exists) {
        updateHistory();
      }
    }

    const once = publishRate === "once";
    const latched = publishRate === "latched";
    let rate = 0.0;
    if (!once && !latched) {
      rate = Number.parseFloat(publishRate);
    }
    // find first available subscriber with QoS
    const qos: RosQos | undefined = findQoSFromSub();

    const result = await provider?.publishMessage(
      new LaunchPublishMessage(
        currentTopicName,
        currentMessageType,
        messageStruct as JSONObject,
        rate,
        once,
        latched,
        false,
        useSimTime,
        substituteKeywords,
        qos
      )
    );
    const publisherName = getPublisherName();
    if (result?.result) {
      logCtx.success(
        `Started publisher for ${currentTopicName} with rate '${publishRate}' and message type: ${currentMessageType}`,
        `${messageStr}`,
        `started publisher ${publisherName}`
      );
    } else {
      logCtx.warn(`Failed to start publisher ${publisherName}`, result?.message, `Failed to start ${publisherName}`);
    }
    setStartPublisherIsSubmitting(false);
    // close modal
    setTimeout(() => {
      setStartPublisherDescription("");
    }, 5000);
  }

  useEffect(() => {
    if (hasPublisher) {
      setStartPublisherIsSubmitting(false);
    }
  }, [hasPublisher]);

  // create input mask for an element of the array
  function createHistoryButton(entry: TMsgHistoryEntry): JSX.Element {
    return (
      <Tooltip title={`${entry.name}`} placement="bottom" disableInteractive>
        <Button
          key={`history-button-${entry.id}`}
          onClick={(event) => {
            fromHistory(entry);
            event.stopPropagation();
          }}
          startIcon={<StorageOutlinedIcon />}
          size="small"
        >
          {entry.id}
        </Button>
      </Tooltip>
    );
  }
  // create edit mask for an element of the history array
  function createHistoryEditItem(entry: TMsgHistoryEntry): JSX.Element {
    return (
      <Stack direction="row" key={`history-edit-item-${entry.id}`} spacing="0.5em">
        <Tooltip title="favorite entries are not automatically deleted" placement="bottom" disableInteractive>
          <IconButton
            key={`history-edit-star-${entry.id}`}
            onClick={(event) => {
              historyCtx?.updateEntryMeta(currentMessageType, entry.id, { favorite: !entry.favorite });
              event.stopPropagation();
            }}
            size="small"
          >
            {entry.favorite ? <StarIcon sx={{ color: "yellow" }} /> : <StarOutlineIcon />}
          </IconButton>
        </Tooltip>
        <TextField
          key={`history-edit-name-${entry.id}`}
          variant="standard"
          size="small"
          defaultValue={entry.name || entry.id}
          onBlur={(event) => {
            if (event.target.value && event.target.value !== entry.name) {
              historyCtx?.updateEntryMeta(currentMessageType, entry.id, { name: event.target.value });
            }
          }}
        />
        <IconButton
          color="error"
          key={`history-edit-delete-${entry.id}`}
          onClick={(event) => {
            historyCtx?.deleteEntry(currentMessageType, entry.id);
            event.stopPropagation();
          }}
          size="small"
        >
          <RemoveCircleOutlineIcon />
        </IconButton>
      </Stack>
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
    <Box height="100%" overflow="auto" alignItems="center" sx={getHostStyle()}>
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
              key={"autocomplete-topic-name"}
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
        <Stack direction="row" spacing={1} display="flex" alignItems="center">
          {!historyEditMode && (
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
          )}
          {!historyEditMode && (
            <Tooltip title="use sim time" placement="bottom" disableInteractive>
              <ToggleButton
                size="small"
                value="use-sim-time"
                selected={useSimTime}
                onChange={() => setUseSimTime((prev) => !prev)}
              >
                <MonitorHeartIcon sx={{ fontSize: "inherit" }} />
              </ToggleButton>
            </Tooltip>
          )}
          {historyByType[currentMessageType]?.length > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              {/* <FormLabel sx={{ fontSize: "0.8em", lineHeight: "1em" }}>Publish history</FormLabel> */}
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                <Tooltip title="edit history" enterDelay={500}>
                  <Button
                    color="success"
                    onClick={(event) => {
                      setMessageStruct(messageStructOrg);
                      setPublishRate("1");
                      setSubstituteKeywords(true);
                      setHistoryEditMode((prev) => !prev);
                      event.stopPropagation();
                    }}
                    // startIcon={<EditNoteIcon />}
                    size="small"
                  >
                    <EditNoteIcon />
                  </Button>
                </Tooltip>
                {historyByType[currentMessageType]?.map((entry) => createHistoryButton(entry))}
              </ButtonGroup>
              {historyEditMode && (
                <Stack direction="column" alignContent="start">
                  <Stack direction="row" alignContent="start">
                    <Slider
                      aria-label="Temperature"
                      defaultValue={maxHistoryLength}
                      valueLabelFormat={(index) => `max history length: ${index}`}
                      valueLabelDisplay="auto"
                      shiftStep={1}
                      step={1}
                      marks
                      min={1}
                      max={10}
                      onChange={(_event: Event, newValue: number) => {
                        setMaxHistoryLength(newValue);
                      }}
                      sx={{ maxWidth: "80%", marginRight: "1.5em" }}
                    />
                    <Tooltip title="Remove all non-favorite entries" enterDelay={500}>
                      <IconButton
                        color="error"
                        onClick={(event) => {
                          for (const entry of historyByType[currentMessageType] ?? []) {
                            if (!entry.favorite) {
                              historyCtx?.deleteEntry(currentMessageType, entry.id);
                            }
                          }
                          event.stopPropagation();
                        }}
                        size="small"
                      >
                        <DeleteForeverIcon />
                      </IconButton>
                    </Tooltip>
                  </Stack>
                  {historyByType[currentMessageType]?.map((entry) => createHistoryEditItem(entry))}
                </Stack>
              )}
            </Stack>
          )}
        </Stack>

        <Box>
          {startPublisherIsSubmitting ? (
            <Stack direction="row" spacing={1}>
              <CircularProgress size="1em" />
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
                  paddingLeft: "0.8em",
                }}
                onClick={() => stopPublisher()}
              >
                <StopCircleOutlined sx={{ fontSize: "inherit" }} />
              </IconButton>
            </Tooltip>
          )}
          {startPublisherDescription && (
            <Typography
              variant="body1"
              sx={{
                fontFamily: "monospace",
                fontSize: "0.8em",
                overflow: "hidden",
                whiteSpace: "normal",
                overflowWrap: "anywhere",
                wordBreak: "break-word",
                textOverflow: "ellipsis",
                // whiteSpace: "pre-line",
              }}
            >{`Message: ${messageStruct ? rosMessageStructToString(messageStruct, false, false) : ""}`}</Typography>
          )}
        </Box>
        <Divider />
        <Stack direction="row" alignItems="center" spacing={1}>
          {editMessageType ? (
            <Autocomplete
              key={"autocomplete-publisher-type"}
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
            <Stack direction="row" alignItems="center">
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
        {obtainingMsgStruct && (
          <Alert severity="info" style={{ minWidth: 0 }}>
            <AlertTitle>{`Obtaining message definition for ${currentMessageType}`}</AlertTitle>{" "}
            <CircularProgress size="1em" />
          </Alert>
        )}
        {!obtainingMsgStruct && !messageStruct && !editMessageType && (
          <Alert severity="error" style={{ minWidth: 0 }}>
            <AlertTitle>{`Message definition for ${currentTopicName}[${currentMessageType}] not found!`}</AlertTitle>
          </Alert>
        )}
        {!obtainingMsgStruct && !messageStruct && editMessageType && (
          <Alert severity="info" style={{ minWidth: 0 }}>
            <AlertTitle>{`Finish editing to obtain message definition for ${currentMessageType}`}</AlertTitle>
          </Alert>
        )}
      </Stack>
    </Box>
  );
}
