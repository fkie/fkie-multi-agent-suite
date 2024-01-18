import ArrowForwardIosSharpIcon from '@mui/icons-material/ArrowForwardIosSharp';
import ContentCopyOutlinedIcon from '@mui/icons-material/ContentCopyOutlined';
import {
  Alert,
  AlertTitle,
  Button,
  ButtonGroup,
  Divider,
  FormLabel,
  IconButton,
  Stack,
} from '@mui/material';
import MuiAccordion from '@mui/material/Accordion';
import MuiAccordionDetails from '@mui/material/AccordionDetails';
import MuiAccordionSummary from '@mui/material/AccordionSummary';
import { styled } from '@mui/material/styles';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import React, { useCallback, useEffect, useState } from 'react';

import BoolInput from './BoolInput';
import StringInput from './StringInput';

const Accordion = styled((props) => (
  <MuiAccordion disableGutters elevation={0} {...props} />
))(({ theme }) => ({
  border: 0,
  borderLeft: `1px solid ${theme.palette.divider}`,
  // '&:not(:last-child)': {
  //   borderBottom: 0,
  // },
  // '&:before': {
  //   display: 'none',
  // },
}));

const AccordionSummary = styled((props) => <MuiAccordionSummary {...props} />)(
  ({ theme }) => ({
    backgroundColor:
      theme.palette.mode === 'dark'
        ? 'rgba(255, 255, 255, .05)'
        : 'rgba(0, 0, 0, .03)',
    flexDirection: 'row-reverse',
    '& .MuiAccordionSummary-expandIconWrapper.Mui-expanded': {
      transform: 'rotate(90deg)',
    },
    '& .MuiAccordionSummary-content': {
      margin: 0,
      marginLeft: theme.spacing(1),
      padding: 0,
    },
    minHeight: 1,
    padding: '6px',
  }),
);

const AccordionDetails = styled(MuiAccordionDetails)(({ theme }) => ({
  padding: theme.spacing(1),
  // borderTop: '1px solid rgba(0, 0, 0, .125)',
}));

const Components = {
  bool: BoolInput,
  boolean: BoolInput,
  int8: StringInput,
  uint8: StringInput,
  int16: StringInput,
  uint16: StringInput,
  int32: StringInput,
  uint32: StringInput,
  int64: StringInput,
  uint64: StringInput,
  float32: StringInput,
  float64: StringInput,
  float: StringInput,
  double: StringInput,
  string: StringInput,
  time: StringInput,
};

function InputElements({
  messageStruct,
  parentName,
  filterText,
  onCopyToClipboard,
}) {
  const [arrayCount, setArrayCount] = useState(0);
  const [idSuffix] = useState(`${parentName}-${messageStruct.name}`);
  const [expandedElement, setExpanded] = useState(
    messageStruct.type !== 'std_msgs/Header',
  );

  useEffect(() => {
    if (messageStruct?.value !== undefined) {
      setArrayCount(messageStruct?.value?.length);
    }
  }, [messageStruct.value]);

  const addArrayElement = useCallback(() => {
    if (!messageStruct.value) {
      messageStruct.value = [];
    }
    messageStruct.value.push(JSON.parse(JSON.stringify(messageStruct.def)));
  }, [messageStruct]);

  const removeArrayElement = useCallback(() => {
    messageStruct.value.pop();
  }, [messageStruct.value]);

  // component visibility based on filter input text
  const [isVisible, setVisible] = useState('');
  useEffect(() => {
    let vis = '';
    if (filterText.length > 1) {
      // TODO: improve struct search by ignoring non-visible fields
      const re = new RegExp(filterText, 'i');
      const pos = JSON.stringify(messageStruct).search(re);
      vis = pos !== -1 ? '' : 'none';
    }
    setVisible(vis);
  }, [filterText, messageStruct]);

  const copyToClipboard = useDebounceCallback(() => {
    if (onCopyToClipboard) {
      onCopyToClipboard();
    }
  });

  if (messageStruct === undefined || messageStruct === null) {
    return [];
  }

  if (messageStruct.def === undefined) {
    messageStruct.def = [];
  }

  if (!messageStruct?.type) {
    return (
      <Alert severity="error" style={{ minWidth: 0 }}>
        <AlertTitle>{`Invalid message struct for ${parentName}`}</AlertTitle>
      </Alert>
    );
  }

  const fieldType = messageStruct.type.replace(/\[\d*\]/, '');
  if (messageStruct.is_array && !messageStruct?.value) {
    messageStruct.value = '';
  }

  // const toggleAccordion = () => {
  //   setExpanded((prev) => !prev);
  // };

  // create element depending on the base type defined in Components
  if (typeof Components[fieldType] !== 'undefined') {
    return React.createElement(Components[fieldType], {
      id: idSuffix,
      messageStruct,
      filterText,
    });
  }
  // create input mask for an element of the array
  function createListEntry(element, index) {
    return (
      <Stack
        direction="column"
        spacing={1}
        key={`liststack-${messageStruct.name}-${index}`}
      >
        <Divider
          size="sm"
          textAlign="left"
        >{`${messageStruct.name}[${index}]`}</Divider>
        {element.map((struct) => (
          <InputElements
            key={`input-elements-${messageStruct.name}-${struct.name}-${index}`}
            parentName={idSuffix}
            messageStruct={struct}
            filterText={filterText}
            expanded={false}
          />
        ))}
      </Stack>
    );
  }

  // for nested types we create an accordion with collapsible elements
  if (messageStruct.def.length > 0) {
    return (
      <Accordion
        id={`accordion-${idSuffix}`}
        expanded={expandedElement}
        // use this action to collapse by click on whole summary
        onChange={(e, state) => {
          setExpanded(state);
        }}
        sx={{ display: `${isVisible}` }}
      >
        <AccordionSummary
          // aria-controls="ttyd_panel-content"
          id={`accordion-summary-${idSuffix}`}
          sx={{ pl: 0 }}
          expandIcon={
            <ArrowForwardIosSharpIcon
              sx={{ fontSize: '0.9rem' }}
              // use this onclick action to collapse by click on the arrow
              // onClick={() => {
              //   toggleAccordion();
              // }}
            />
          }
        >
          <Stack direction="row" spacing={1}>
            <FormLabel
              sx={{
                typography: 'body1',
                color: '#000080',
                // fontSize: 'small',
              }}
            >
              {messageStruct.name && `${messageStruct.name}`}
              {!messageStruct.name && `${messageStruct.type}`}
            </FormLabel>
            <FormLabel
              sx={{
                typography: 'body1',
                fontSize: 'small',
                paddingTop: '4px',
              }}
            >
              {messageStruct.name &&
                messageStruct.type &&
                `${messageStruct.type}`}
            </FormLabel>
            {!messageStruct.name && (
              <ButtonGroup sx={{ maxHeight: '24px' }}>
                <IconButton
                  aria-label="ContentCopyOutlined"
                  size="small"
                  onClick={(event) => {
                    copyToClipboard();
                    event.stopPropagation();
                  }}
                >
                  <ContentCopyOutlinedIcon fontSize="inherit" />
                </IconButton>
              </ButtonGroup>
            )}
            {messageStruct.is_array && (
              <ButtonGroup sx={{ maxHeight: '24px' }}>
                <Button
                  onClick={(event) => {
                    addArrayElement();
                    setArrayCount(messageStruct.value.length);
                    event.stopPropagation();
                  }}
                >
                  +
                </Button>
                <Button
                  onClick={(event) => {
                    if (arrayCount > 0) {
                      removeArrayElement();
                      setArrayCount(messageStruct.value.length);
                      event.stopPropagation();
                    }
                  }}
                >
                  -
                </Button>
              </ButtonGroup>
            )}
            <FormLabel
              sx={{
                typography: 'body1',
                fontSize: 'small',
                fontWeight: 'bold',
                color: '#3cb371',
                paddingTop: '4px',
                paddingLeft: '1rem',
              }}
            >
              {messageStruct.is_array &&
                arrayCount > 0 &&
                `count [${arrayCount}]`}
            </FormLabel>
          </Stack>
        </AccordionSummary>
        <AccordionDetails id={`accordion-details-${idSuffix}`}>
          <Stack direction="column" spacing={1}>
            {!messageStruct.is_array &&
              messageStruct.def.map((struct) => (
                <InputElements
                  key={`input-elements-${messageStruct.name}-${struct.name}`}
                  parentName={idSuffix}
                  messageStruct={struct}
                  filterText={filterText}
                  expanded={false}
                />
              ))}
            {messageStruct.is_array &&
              messageStruct.value &&
              arrayCount > 0 &&
              messageStruct.value.map((item, index) =>
                createListEntry(item, index),
              )}
          </Stack>
        </AccordionDetails>
      </Accordion>
    );
  }
  if (!messageStruct.name) {
    return (
      <Alert severity="info" style={{ minWidth: 0 }}>
        <AlertTitle>
          {`No input for "${messageStruct.type}" defined.`}
        </AlertTitle>
      </Alert>
    );
  }
  // component doesn't exist yet
  return (
    <Alert severity="warning" style={{ minWidth: 0 }}>
      <AlertTitle>
        {`The input mask for "${messageStruct.type}" has not been created
      yet. Parameter "${messageStruct.name}" ignored.`}
      </AlertTitle>
    </Alert>
  );
}

InputElements.defaultProps = {
  messageStruct: null,
  parentName: 'undefined',
  filterText: '',
  onCopyToClipboard: null,
};

InputElements.propTypes = {
  messageStruct: PropTypes.any,
  parentName: PropTypes.string,
  filterText: PropTypes.string,
  onCopyToClipboard: PropTypes.func,
};

export default InputElements;
