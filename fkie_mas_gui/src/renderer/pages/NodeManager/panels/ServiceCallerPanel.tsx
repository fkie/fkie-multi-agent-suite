import { Provider } from "@/renderer/providers";
import ContentCopyOutlinedIcon from "@mui/icons-material/ContentCopyOutlined";
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
  IconButton,
  Stack,
  Tooltip,
  Typography,
} from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { forwardRef, useCallback, useContext, useEffect, useMemo, useState } from "react";
import JsonView from "react18-json-view";
import { SearchBar } from "../../../components";
import { colorFromHostname } from "../../../components/UI/Colors";
import LoggingContext from "../../../context/LoggingContext";
import { RosContext } from "../../../context/RosContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { LaunchCallService, rosMessageStructToString, TRosMessageStruct } from "../../../models";
import InputElements from "./MessageDialogPanel/InputElements";

interface ServiceCallerPanelProps {
  serviceName: string;
  serviceType: string;
  providerId: string;
}

const ServiceCallerPanel = forwardRef<HTMLDivElement, ServiceCallerPanelProps>(function ServiceCallerPanel(props, ref) {
  const { serviceName, serviceType, providerId } = props;

  const logCtx = useContext(LoggingContext);
  const settingsCtx = useContext(SettingsContext);
  const [history, setHistory] = useLocalStorage(`ServiceStruct:history`, {});
  const [historyLength, setHistoryLength] = useState(0);
  const rosCtx = useContext(RosContext);
  const [searchTerm, setSearchTerm] = useState("");
  const [serviceStruct, setServiceStruct] = useState<TRosMessageStruct>();
  const [serviceStructOrg, setServiceStructOrg] = useState<TRosMessageStruct>();
  const [provider, setProvider] = useState<Provider | null>(null);
  const [inputElements, setInputElements] = useState<React.ReactNode | null>(null);

  const [callServiceDescription, setCallServiceDescription] = useState("");
  const [callServiceIsSubmitting, setCallServiceIsSubmitting] = useState(false);
  const [resultError, setResultError] = useState<string>();
  const [resultMessage, setResultMessage] = useState<TRosMessageStruct>();
  const [timeoutObj, setTimeoutObj] = useState<NodeJS.Timeout | null>(null);

  // get item history after the history was loaded
  const fromHistory = useDebounceCallback((index) => {
    const historyInStruct = history[serviceType];
    if (historyInStruct) {
      const historyItem = index && index < historyInStruct.length ? historyInStruct[index] : historyInStruct[0];
      setServiceStruct(JSON.parse(JSON.stringify(historyItem.msg)));
    }
  }, 100);

  // get item history after the history was loaded
  const updateHistory = useCallback(() => {
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
      delete history[serviceType];
    }
    setHistory(history);
  }, [serviceStruct, history, setHistory]);

  // get item history after the history was loaded
  const clearHistory = useCallback(() => {
    if (!serviceStruct) return;
    const historyInStruct = history[serviceType];
    if (historyInStruct && historyInStruct.length > 0) {
      historyInStruct.pop();
      setHistory(history);
      setHistoryLength(historyInStruct.length);
    }
  }, [history, serviceStruct]);

  // create string from message struct and copy it to clipboard
  const onCopyToClipboard = useCallback(() => {
    const json = serviceStruct ? rosMessageStructToString(serviceStruct, false, false) : "{}";
    navigator.clipboard.writeText(`${serviceName} ${serviceType} "${json}"`);
    logCtx.success(`service call input copied!`);
  }, [serviceStruct]);

  const getServiceStructData = useCallback(async () => {
    if (serviceName) {
      const newProvider = rosCtx.getProviderById(providerId);
      if (newProvider) {
        setProvider(newProvider);
        if (serviceType) {
          const srvStruct = await newProvider.getServiceStruct(serviceType);
          if (srvStruct) {
            setServiceStructOrg(srvStruct.data);
            setServiceStruct(srvStruct.data);
            setInputElements(
              <InputElements
                key={srvStruct.data.type}
                messageStruct={srvStruct.data}
                parentName={srvStruct.data.type ? srvStruct.data.type : `${serviceName}[${serviceType}]`}
                filterText={searchTerm}
              />
            );
          }
        }
      }
    }
  }, [serviceName, providerId, rosCtx]);

  // debounced filter callback
  const onUpdateInputElements = useCallback(
    (searchText) => {
      if (!serviceStruct) return;
      setInputElements(
        <InputElements
          key={serviceStruct.type}
          messageStruct={serviceStruct}
          parentName={serviceStruct.type ? serviceStruct.type : `${serviceName}[${serviceType}]`}
          filterText={searchText}
        />
      );
    },
    [serviceStruct]
  );

  async function handleCallService(): Promise<void> {
    setResultError("");
    setResultMessage(undefined);
    setCallServiceDescription("Calling service...");
    setCallServiceIsSubmitting(true);

    // store struct to history if new message
    const messageStr = rosMessageStructToString(serviceStruct, false, false);
    const historyInStruct = history[serviceType];
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

    if (provider) {
      const srvResult = await provider.callService(new LaunchCallService(serviceName, serviceType, serviceStruct));
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
        }, 5000)
      );
    } else {
      setCallServiceIsSubmitting(false);
      setCallServiceDescription("");
    }
  }

  useEffect(() => {
    if (!timeoutObj) return;
    if (resultMessage || resultError) {
      clearTimeout(timeoutObj);
      setTimeoutObj(null);
      setCallServiceIsSubmitting(false);
      setCallServiceDescription("");
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
    if ((serviceStruct.def || []).length === 0) {
      handleCallService();
    }
  }, [serviceStruct]);

  // Get topic struct when mounting the component
  useEffect(() => {
    getServiceStructData();
  }, [serviceName]);

  // Update the visible state of input fields on a filter change
  useEffect(() => {
    onUpdateInputElements(searchTerm);
  }, [searchTerm]);

  // create input mask for an element of the array
  function createHistoryButton(index): JSX.Element {
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

  const createJsonView = useMemo(() => {
    return (
      <JsonView
        src={resultMessage}
        dark={settingsCtx.get("useDarkMode") as boolean}
        theme="a11y"
        enableClipboard={false}
        ignoreLargeArray={false}
        collapseObjectsAfterLength={3}
        displaySize={"collapsed"}
        collapsed={(params: {
          node: Record<string, unknown> | Array<unknown>; // Object or array
          indexOrName: number | string | undefined;
          depth: number;
          size: number; // Object's size or array's length
        }) => {
          if (params.indexOrName === undefined) return false;
          if (Array.isArray(params.node) && params.node.length === 0) return true;
          return false;
        }}
      />
    );
  }, [resultMessage, settingsCtx.changed]);

  return (
    <Box ref={ref} height="100%" overflow="auto" alignItems="center" sx={getHostStyle()}>
      <Stack spacing={1} margin={1}>
        {serviceStruct && (serviceStruct.def || []).length > 0 && (
          <Stack direction="row" spacing={1}>
            <SearchBar
              onSearch={(value) => {
                try {
                  // const re = new RegExp(value, "i");
                  setSearchTerm(value);
                } catch {
                  // TODO: visualize error
                }
              }}
              placeholder="Filter Fields"
              defaultValue={searchTerm}
            />
          </Stack>
        )}
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography fontWeight="bold">{serviceName}</Typography>
        </Stack>
        <Stack direction="row" alignItems="center" spacing={1}>
          <Typography color="grey" fontSize="0.8em">
            {provider?.name()}
          </Typography>
        </Stack>
        <Stack direction="row" spacing={2} display="flex" alignItems="center">
          {historyLength > 0 && (
            <Stack direction="column" spacing={1} alignItems="left">
              <FormLabel sx={{ fontSize: "0.8em", lineHeight: "1em" }}>call history</FormLabel>
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                {historyLength > 0 && (
                  <Tooltip title="reset values" enterDelay={500} disableInteractive>
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
                  <Tooltip title="remove oldest history entry" enterDelay={500} disableInteractive>
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
              <div>{`${callServiceDescription} with arguments ${rosMessageStructToString(
                serviceStruct,
                false,
                false
              )}`}</div>
            </Stack>
          ) : (
            <Stack direction="row" spacing="0.5em">
              <Button
                type="submit"
                variant="contained"
                color="success"
                onClick={handleCallService}
                disabled={serviceStruct?.type === undefined}
              >
                Call Service
              </Button>
              <Tooltip title="Copy service name, type and data fields" placement="bottom">
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
});

export default ServiceCallerPanel;
