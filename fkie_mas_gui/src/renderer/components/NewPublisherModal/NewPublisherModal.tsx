import { LAYOUT_TAB_SETS, LayoutTabConfig } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import TopicPublishPanel from "@/renderer/pages/NodeManager/panels/TopicPublishPanel";
import DeleteIcon from "@mui/icons-material/Delete";
import {
  Autocomplete,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  Stack,
  TextField,
  Typography,
} from "@mui/material";
import { ForwardedRef, forwardRef, HTMLAttributes, useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { RosContext } from "../../context/RosContext";
import useLocalStorage from "../../hooks/useLocalStorage";
import DraggablePaper from "../UI/DraggablePaper";

interface NewPublisherModalProps {
  providerId: string;
  onClose: () => void;
}

const NewPublisherModal = forwardRef<HTMLDivElement, NewPublisherModalProps>(function NewPublisherModal(props, ref) {
  const { providerId, onClose = (): void => {} } = props;

  const rosCtx = useContext(RosContext);
  const [open, setOpen] = useState(true);
  const [topicName, setTopicName] = useState<string>("");
  const [messageType, setMessageType] = useState<string>("");
  const [messageTypeOptions, setMessageTypeOptions] = useState<string[]>([]);
  const [nameHistory, setNameHistory] = useLocalStorage<string[]>("history:publisherNames", []);

  // Make a request to provider and get known message types
  const getAvailableMessageTypes = useCallback(
    async function (): Promise<void> {
      const provider = rosCtx.getProviderById(providerId, true);
      if (!provider || !provider.isAvailable()) return;

      const result: string[] = await provider.getRosMessageMessageTypes();
      if (result.length === 0) return;
      setMessageTypeOptions(result);
    },
    [providerId, rosCtx.providers]
  );

  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    if (reason === "confirmed") {
      setNameHistory((prev) => [topicName, ...prev.filter((item) => item !== topicName)]);
    }
    setOpen(false);
    onClose();
  }

  function openPublisherPanel(): void {
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `publish-${topicName}-${providerId}`,
        topicName,
        <TopicPublishPanel topicName={topicName} topicType={messageType} providerId={providerId} />,
        true,
        LAYOUT_TAB_SETS.BORDER_RIGHT,
        new LayoutTabConfig(true, "publish")
      )
    );
  }

  const deleteNameHistoryOption = useCallback(
    function (option: string): void {
      setNameHistory((prev) => prev.filter((item) => item !== option));
    },
    [nameHistory, setNameHistory]
  );

  useEffect(() => {
    getAvailableMessageTypes();
  }, [rosCtx.providers, providerId]);

  const dialogRef = useRef(ref);

  return (
    <Dialog
      open={open}
      onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
      fullWidth
      scroll="paper"
      ref={dialogRef as ForwardedRef<HTMLDivElement>}
      PaperProps={{
        component: DraggablePaper,
        dialogRef: dialogRef,
      }}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
        Define new publisher
      </DialogTitle>
      <DialogContent>
        <Stack>
          <Stack>
            <Autocomplete
              key={`autocomplete-publisher-name`}
              size="small"
              fullWidth
              autoHighlight
              clearOnEscape
              disableListWrap
              handleHomeEndKeys={false}
              // noOptionsText="Package not found"
              options={nameHistory}
              getOptionLabel={(option) => option}
              // This prevents warnings on invalid autocomplete values
              value={topicName}
              renderInput={(params) => (
                <TextField
                  {...params}
                  label="Topic Name"
                  color="info"
                  variant="outlined"
                  margin="dense"
                  size="small"
                  autoFocus
                />
              )}
              renderOption={(props, option) => {
                return (
                  <Stack {...(props as HTMLAttributes<HTMLDivElement>)} key={option} direction="row">
                    <Typography style={{ overflowWrap: "anywhere" }} width="stretch">
                      {option}
                    </Typography>
                    <IconButton
                      component="label"
                      onClick={(event) => {
                        deleteNameHistoryOption(option);
                        event.stopPropagation();
                      }}
                    >
                      <DeleteIcon sx={{ fontSize: "1em" }} />
                    </IconButton>
                  </Stack>
                );
              }}
              onChange={(_event, newNameValue) => {
                setTopicName(newNameValue ? newNameValue : "");
              }}
              onInputChange={(_event, newInputValue) => {
                setTopicName(newInputValue ? newInputValue : "");
              }}
              isOptionEqualToValue={(option, value) => {
                return value === undefined || value === "" || option === value;
              }}
              onWheel={(event) => {
                // scroll through the options using mouse wheel
                let newIndex = -1;
                nameHistory.forEach((value, index) => {
                  if (value === (event.target as HTMLInputElement).value) {
                    if (event.deltaY > 0) {
                      newIndex = index + 1;
                    } else {
                      newIndex = index - 1;
                    }
                  }
                });
                if (newIndex < 0) newIndex = nameHistory.length - 1;
                else if (newIndex > nameHistory.length - 1) newIndex = 0;
                setTopicName(nameHistory[newIndex]);
              }}
            />
            <Autocomplete
              key={`autocomplete-publisher-type`}
              size="small"
              fullWidth
              autoHighlight
              clearOnEscape
              disableListWrap
              handleHomeEndKeys={false}
              // noOptionsText="Package not found"
              options={messageTypeOptions}
              getOptionLabel={(option) => option}
              // This prevents warnings on invalid autocomplete values
              value={messageType}
              renderInput={(params) => (
                <TextField
                  {...params}
                  label="Message Type"
                  color="info"
                  variant="outlined"
                  margin="dense"
                  size="small"
                  // autoFocus
                />
              )}
              // renderOption={(props, option) => {
              //   return (
              //     <Stack {...(props as HTMLAttributes<HTMLDivElement>)} key={option} direction="row">
              //       <Typography style={{ overflowWrap: "anywhere" }} width="stretch">
              //         {option}
              //       </Typography>
              //       <IconButton
              //         component="label"
              //         onClick={(event) => {
              //           deleteTypeHistoryOption(option);
              //           event.stopPropagation();
              //         }}
              //       >
              //         <DeleteIcon sx={{ fontSize: "1em" }} />
              //       </IconButton>
              //     </Stack>
              //   );
              // }}
              onChange={(_event, newNameValue) => {
                setMessageType(newNameValue ? newNameValue : "");
              }}
              onInputChange={(_event, newInputValue) => {
                setMessageType(newInputValue ? newInputValue : "");
              }}
              isOptionEqualToValue={(option, value) => {
                return value === undefined || value === "" || option === value;
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
                setMessageType(messageTypeOptions[newIndex]);
              }}
            />
          </Stack>
        </Stack>
      </DialogContent>
      <DialogActions>
        <Button
          autoFocus
          onClick={() => {
            handleClose("cancel");
          }}
        >
          Cancel
        </Button>
        <Button
          disabled={!topicName || !messageType}
          color="success"
          variant="contained"
          onClick={() => {
            openPublisherPanel();
            handleClose("confirmed");
          }}
        >
          Next
        </Button>
      </DialogActions>
    </Dialog>
  );
});

export default NewPublisherModal;
