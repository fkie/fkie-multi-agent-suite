import { TRosMessageStruct } from "@/renderer/models";
import { Autocomplete, FormLabel, Stack, TextField } from "@mui/material";
import { useEffect, useState } from "react";
import useLocalStorage from "../../../../hooks/useLocalStorage";

interface StringInputProps {
  id: string;
  messageStruct: TRosMessageStruct;
  filterText?: string;
}

export default function StringInput(props: StringInputProps): JSX.Element {
  const { id, messageStruct, filterText = "" } = props;

  const [historyStruct, setHistoryStruct] = useLocalStorage(`MessageStruct:history`, {});
  const [history, setHistory] = useState<string[]>([]);
  const [value, setValue] = useState<string>(messageStruct?.value ? (messageStruct.value as string) : "");
  const [isVisible, setVisible] = useState<string>("");
  const [isError, setIsError] = useState<boolean>(false);
  const [helperText, setHelperText] = useState<string>("");
  const [checkForValidNumber] = useState<boolean>(messageStruct?.type.search("str") === -1 || false);

  useEffect(() => {
    setValue(messageStruct?.value ? (messageStruct.value as string) : "");
    if (messageStruct.default_value) {
      setHistory([messageStruct.default_value as string]);
    }
  }, [messageStruct.value, messageStruct.default_value]);

  // get item history after the history was loaded
  function updateHistory(val: string): void {
    if (!val) return;
    if (!history.includes(val)) {
      history.unshift(val);
      if (history.length > 5) {
        history.pop();
      }
      setHistory(history);
      historyStruct[id] = history;
      setHistoryStruct(historyStruct);
    }
  }
  // get item history after the history was loaded
  useEffect(() => {
    const historyInStruct = historyStruct[id];
    if (historyInStruct) setHistory(historyInStruct);
  }, [historyStruct, id]);

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

  // check value for valid range
  function checkValue(newValue: string): void {
    // TODO: add additional tests
    if (messageStruct.is_array) {
      return;
    }
    if (!checkForValidNumber) return;
    let msg = "";
    if (newValue.length > 0) {
      let cleanedValue = newValue.replace(" ", "");
      if (newValue.search("[.]") !== -1) {
        cleanedValue = newValue.replace(/[0,]*$/, "");
        cleanedValue = cleanedValue.replace(/[.]*$/, "");
      }
      const parsed = Number.parseFloat(newValue);
      if (`${parsed}` !== cleanedValue) {
        msg = "it is not a number";
      } else if (messageStruct.type.search("int") !== -1 && newValue.search(/[. ,]/) !== -1) {
        msg = `invalid literal for int() with base 10: '${newValue}'`;
      } else if (messageStruct.type.startsWith("u")) {
        if (parsed < 0) {
          msg = "accepts only positive values";
        }
      }
    }
    setHelperText(msg);
    setIsError(msg.length > 0);
  }

  return (
    <Autocomplete
      sx={{ display: `${isVisible}` }}
      id={`string-input-${id}`}
      key={`string-input-${id}`}
      handleHomeEndKeys={false}
      freeSolo
      options={history}
      size="small"
      // sx={{ width: 150 }}
      inputValue={value || ""}
      onInputChange={(event, newValue) => {
        messageStruct.value = newValue;
        setValue(newValue);
        checkValue((event.target as HTMLInputElement).value);
      }}
      renderInput={(params) => (
        <TextField
          {...params}
          // type="text"
          error={isError}
          helperText={helperText}
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
        />
      )}
      onChange={(event, newValue) => {
        const val: string = newValue ? newValue : "";
        messageStruct.value = val;
        updateHistory(val);
        setValue(val);
        checkValue((event.target as HTMLInputElement).value);
      }}
    />
  );
}
