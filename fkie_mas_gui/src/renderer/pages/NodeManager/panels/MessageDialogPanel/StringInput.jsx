import { Autocomplete, FormLabel, Stack, TextField } from '@mui/material';
import PropTypes from 'prop-types';
import { useEffect, useState } from 'react';
import useLocalStorage from '../../../../hooks/useLocalStorage';

function StringInput({ id = '', messageStruct = null, filterText = '' }) {
  const [historyStruct, setHistoryStruct] = useLocalStorage(
    `MessageStruct:history`,
    {},
  );
  const [history, setHistory] = useState([]);
  const [value, setValue] = useState(
    messageStruct?.value ? messageStruct.value : '',
  );
  const [isVisible, setVisible] = useState('');
  const [isError, setIsError] = useState(false);
  const [helperText, setHelperText] = useState('');
  const [checkForValidNumber] = useState(
    messageStruct?.type.search('str') === -1 || false,
  );

  useEffect(() => {
    setValue(messageStruct?.value ? messageStruct.value : '');
    if (messageStruct.default_value !== undefined) {
      setHistory([messageStruct.default_value]);
    }
  }, [messageStruct.value, messageStruct.default_value]);

  // get item history after the history was loaded
  const updateHistory = (val) => {
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
  };
  // get item history after the history was loaded
  useEffect(() => {
    const historyInStruct = historyStruct[id];
    if (historyInStruct) setHistory(historyInStruct);
  }, [historyStruct, id]);

  // check if this item pass the filter
  useEffect(() => {
    let vis = '';
    if (filterText.length > 1) {
      const re = new RegExp(filterText, 'i');
      const pos = messageStruct.name.search(re);
      vis = pos !== -1 ? '' : 'none';
    }
    setVisible(vis);
  }, [filterText, messageStruct.name]);

  // check value for valid range
  function checkValue(newValue) {
    // TODO: add additional tests
    if (messageStruct.is_array) {
      return;
    }
    if (!checkForValidNumber) return;
    let msg = '';
    if (newValue.length > 0) {
      let cleanedValue = newValue.replace(' ', '');
      if (newValue.search('[.]') !== -1) {
        cleanedValue = newValue.replace(/[0,]*$/, '');
        cleanedValue = cleanedValue.replace(/[.]*$/, '');
      }
      const parsed = Number.parseFloat(newValue);
      if (`${parsed}` !== cleanedValue) {
        msg = 'it is not a number';
      } else if (
        messageStruct.type.search('int') !== -1 &&
        newValue.search(/[. ,]/) !== -1
      ) {
        msg = `nvalid literal for int() with base 10: '${newValue}'`;
      } else if (messageStruct.type.startsWith('u')) {
        if (parsed < 0) {
          msg = 'accepts only positive values';
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
      freeSolo
      options={history}
      size="small"
      // sx={{ width: 150 }}
      inputValue={value || ''}
      onInputChange={(event, newValue) => {
        messageStruct.value = newValue;
        setValue(newValue);
        checkValue(event.target.value);
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
                  color: '#808080',
                  // fontSize: 'small',
                }}
              >
                {messageStruct.name && `${messageStruct.name}`}
                {!messageStruct.name && `${messageStruct.type}`}
              </FormLabel>
              <FormLabel
                sx={{
                  fontSize: 'small',
                  paddingTop: '4px',
                  color: '#808B96',
                }}
              >
                {messageStruct.name &&
                  messageStruct.type &&
                  `${messageStruct.type}`}
              </FormLabel>
            </Stack>
          }
        />
      )}
      onChange={(event, newValue) => {
        messageStruct.value = newValue;
        updateHistory(newValue);
        setValue(newValue);
        checkValue(event.target.value);
      }}
    />
  );
}

StringInput.propTypes = {
  id: PropTypes.string,
  messageStruct: PropTypes.any,
  filterText: PropTypes.string,
};

export default StringInput;
