import {
  Autocomplete,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Stack,
  TextField,
} from "@mui/material";
import { useState } from "react";
import DraggablePaper from "../UI/DraggablePaper";

interface SelectDomainIdModalProps {
  domainIds: string[];
  onClose: (domainId?: string) => void;
}

export default function SelectDomainIdModal(props: SelectDomainIdModalProps): JSX.Element {
  const { domainIds, onClose = (): void => {} } = props;

  const [open, setOpen] = useState(true);
  const [domainId, setDomainId] = useState<string>(domainIds.length > 0 ? domainIds[0] : "");

  function handleClose(reason: "backdropClick" | "escapeKeyDown" | "confirmed" | "cancel"): void {
    if (reason && reason === "backdropClick") return;
    setOpen(false);
    onClose(reason === "confirmed" ? domainId : undefined);
  }

  return (
    <Dialog
      open={open}
      onClose={(reason: "backdropClick" | "escapeKeyDown") => handleClose(reason)}
      fullWidth
      scroll="paper"
      PaperComponent={DraggablePaper}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
        Select ROS_DOMAIN_ID
      </DialogTitle>
      <DialogContent>
        <Stack>
          <Autocomplete
            key={"autocomplete-publisher-name"}
            size="small"
            fullWidth
            autoHighlight
            clearOnEscape
            disableListWrap
            handleHomeEndKeys={false}
            // noOptionsText="Package not found"
            options={domainIds}
            getOptionLabel={(option) => option}
            // This prevents warnings on invalid autocomplete values
            value={domainId}
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
            onChange={(_event, newNameValue) => {
              setDomainId(newNameValue ? newNameValue : "");
            }}
            onInputChange={(_event, newInputValue) => {
              setDomainId(newInputValue ? newInputValue : "");
            }}
            isOptionEqualToValue={(option, value) => {
              return value === undefined || value === "" || option === value;
            }}
            onWheel={(event) => {
              // scroll through the options using mouse wheel
              let newIndex = -1;
              domainIds.forEach((value, index) => {
                if (value === (event.target as HTMLInputElement).value) {
                  if (event.deltaY > 0) {
                    newIndex = index + 1;
                  } else {
                    newIndex = index - 1;
                  }
                }
              });
              if (newIndex < 0) newIndex = domainIds.length - 1;
              else if (newIndex > domainIds.length - 1) newIndex = 0;
              setDomainId(domainIds[newIndex]);
            }}
          />
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
          disabled={!domainId && domainId.length > 0}
          color="success"
          variant="contained"
          onClick={() => {
            handleClose("confirmed");
          }}
        >
          Next
        </Button>
      </DialogActions>
    </Dialog>
  );
}
