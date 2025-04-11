import { Checkbox, FormControlLabel, FormGroup, FormLabel, Stack, TextField } from "@mui/material";
import { useEffect, useState } from "react";

import { TRosMessageStruct } from "@/renderer/models";

interface BoolInputProps {
  id: string;
  messageStruct: TRosMessageStruct;
  filterText?: string;
}

export default function BoolInput(props: BoolInputProps): JSX.Element {
  const { id, messageStruct, filterText = "" } = props;
  const [value, setValue] = useState<string | boolean>(messageStruct.is_array ? "" : false);
  const [isVisible, setVisible] = useState("");

  useEffect(() => {
    if (messageStruct?.value !== undefined) {
      if (typeof messageStruct.value === "boolean") {
        setValue(messageStruct.value);
      } else {
        setValue(messageStruct.value as string);
      }
    } else if (messageStruct.default_value !== undefined) {
      setValue(Boolean(messageStruct.default_value));
    } else if (messageStruct.is_array !== undefined && !messageStruct.is_array) {
      setValue(false);
    }
  }, [messageStruct.value, messageStruct.default_value, messageStruct.is_array]);

  // check if this item pass the filter
  useEffect(() => {
    let vis = "";
    if (filterText.length > 1) {
      const re = new RegExp(filterText, "i");
      const pos = messageStruct.name.search(re);
      vis = pos !== -1 ? "" : "none";
    }
    setVisible(vis);
  }, [filterText, messageStruct.name]);

  const isArray = messageStruct.is_array;

  return (
    <Stack sx={{ margin: 0, padding: 0, display: `${isVisible}` }}>
      {isArray ? (
        // for list of boolean values we use a text field
        <TextField
          id={`string-input-${id}`}
          key={`string-input-${id}`}
          type="text"
          label={
            <Stack direction="row" spacing={1}>
              <FormLabel
                sx={{
                  color: "#808080",
                  // fontSize: 'small',
                }}
              >
                {messageStruct.name && `${messageStruct.name}`}
                {!messageStruct.name && `${messageStruct.type}`}
              </FormLabel>
              <FormLabel
                sx={{
                  fontSize: "small",
                  paddingTop: "4px",
                  color: "#808B96",
                }}
              >
                {messageStruct.name && messageStruct.type && `${messageStruct.type}`}
              </FormLabel>
            </Stack>
          }
          size="small"
          placeholder=""
          defaultValue={value}
          onChange={(event) => {
            messageStruct.value = event.target.value;
            setValue(event.target.value);
          }}
        />
      ) : (
        <FormGroup id={`bool-input-${id}`} key={`string-input-${id}`}>
          <FormControlLabel
            control={
              <Checkbox
                checked={value as boolean}
                onChange={(event) => {
                  messageStruct.value = event.target.checked;
                  setValue(event.target.checked);
                }}
              />
            }
            label={
              <Stack direction="row" spacing={1}>
                <FormLabel
                  sx={{
                    color: "#808080",
                  }}
                >
                  {messageStruct.name && `${messageStruct.name}`}
                  {!messageStruct.name && `${messageStruct.type}`}
                </FormLabel>
              </Stack>
            }
          />
        </FormGroup>
      )}
    </Stack>
  );
}
