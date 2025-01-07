import { Provider } from "@/renderer/providers";
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
  topicName: string;
  providerId: string;
}

const TopicPublishPanel = forwardRef<HTMLDivElement, TopicPublishPanelProps>(function TopicPublishPanel(props, ref) {
  const { topicName = null, providerId = "" } = props;

  const [history, setHistory] = useLocalStorage<{ [msg: string]: THistoryItem[] }>(`MessageStruct:history`, {});
  const [historyLength, setHistoryLength] = useState(0);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [substituteKeywords, setSubstituteKeywords] = useState(true);
  const [messageType, setMessageType] = useState("");
  const [searchTerm, setSearchTerm] = useState("");
  const [messageStruct, setMessageStruct] = useState<TRosMessageStruct>();
  const [messageStructOrg, setMessageStructOrg] = useState<TRosMessageStruct>();
  const [publishRate, setPublishRate] = useState<string>("once");
  const [provider, setProvider] = useState<Provider | null>(null);
  const [inputElements, setInputElements] = useState<React.ReactNode | null>(null);

  const [startPublisherDescription, setStartPublisherDescription] = useState("");
  const [startPublisherIsSubmitting, setStartPublisherIsSubmitting] = useState(false);

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
    } as THistoryItem);
    if (historyInStruct.length > 5) {
      historyInStruct.pop();
    }
    if (historyInStruct.length > 0) {
      history[messageType] = historyInStruct;
    } else if (hasType) {
      delete history[messageType];
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
    const json: string = rosMessageStructToString(messageStruct, false, true) as string;
    navigator.clipboard.writeText(json);
    logCtx.success(`${json} copied!`);
  }, 300);

  const getTopicStructData = useCallback(async () => {
    if (topicName) {
      const newProvider = rosCtx.getProviderById(providerId, true);
      if (newProvider) {
        setProvider(newProvider);
        let msgType = "";
        // Get messageType from node list of the provider
        newProvider.rosNodes.forEach((node) => {
          if (node.providerId === providerId) {
            node.subscribers.forEach((topic) => {
              if (msgType === "" && topicName === topic.name) {
                msgType = topic.msg_type;
              }
            });
            if (msgType === "") {
              node.publishers.forEach((topic) => {
                if (msgType === "" && topicName === topic.name) {
                  msgType = topic.msg_type;
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
    if (!messageStruct) return;
    setStartPublisherDescription("Starting publisher...");
    setStartPublisherIsSubmitting(true);

    // store struct to history if new message
    const messageStr = rosMessageStructToString(messageStruct, false, false);
    const historyInStruct = history[messageType];
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
        topicName as string,
        messageType,
        messageStruct,
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
        backgroundColor: settingsCtx.get("backgroundColor") as string,
      };
    }
    return { flexGrow: 1, backgroundColor: settingsCtx.get("backgroundColor") as string };
  };

  return (
    <Box ref={ref} height="100%" overflow="auto" alignItems="center" sx={getHostStyle()}>
      <Stack spacing={1} margin={0.5}>
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography fontWeight="bold">{topicName}</Typography>
          <Typography color="grey" fontSize="0.8em">
            {provider?.name()}
          </Typography>
        </Stack>
        <Stack direction="row" spacing={1}>
          {messageStruct && messageStruct.def.length > 0 && (
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
              setPublishRate(newValue ? newValue : "once");
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
          <FormGroup>
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
          </FormGroup>
          {historyLength > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              <FormLabel>Publish history</FormLabel>
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
              <div>{`${startPublisherDescription} ${messageStruct ? rosMessageStructToString(messageStruct, false, false) : ""}`}</div>
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
});

export default TopicPublishPanel;
