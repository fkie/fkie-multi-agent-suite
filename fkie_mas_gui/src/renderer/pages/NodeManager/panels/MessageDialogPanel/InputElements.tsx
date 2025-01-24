import ArrowForwardIosSharpIcon from "@mui/icons-material/ArrowForwardIosSharp";
import {
  Alert,
  AlertTitle,
  Button,
  ButtonGroup,
  Checkbox,
  Divider,
  FormControlLabel,
  FormLabel,
  Stack,
} from "@mui/material";
import MuiAccordion, { AccordionProps } from "@mui/material/Accordion";
import MuiAccordionDetails from "@mui/material/AccordionDetails";
import MuiAccordionSummary, { AccordionSummaryProps } from "@mui/material/AccordionSummary";
import { styled } from "@mui/material/styles";
import React, { useCallback, useContext, useEffect, useState } from "react";

import SettingsContext from "@/renderer/context/SettingsContext";
import { TRosMessageStruct } from "@/renderer/models";
import { blue } from "@mui/material/colors";
import BoolInput from "./BoolInput";
import StringInput from "./StringInput";

const Accordion = styled((props: AccordionProps) => <MuiAccordion disableGutters elevation={0} square {...props} />)(
  ({ theme }) => ({
    border: 0,
    borderLeft: `1px solid ${theme.palette.divider}`,
    // '&:not(:last-child)': {
    //   borderBottom: 0,
    // },
    // '&:before': {
    //   display: 'none',
    // },
  })
);

const AccordionSummary = styled((props: AccordionSummaryProps) => <MuiAccordionSummary {...props} />)(({ theme }) => ({
  // backgroundColor: theme.palette.mode === "dark" ? "rgba(255, 255, 255, .05)" : "rgba(0, 0, 0, .03)",
  flexDirection: "row-reverse",
  "& .MuiAccordionSummary-expandIconWrapper.Mui-expanded": {
    transform: "rotate(90deg)",
  },
  "& .MuiAccordionSummary-content": {
    width: "1em",
    margin: 0,
    marginLeft: theme.spacing(1),
    padding: 0,
  },
  minHeight: 1,
  padding: "6px",
}));

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

interface InputElementsProps {
  messageStruct: TRosMessageStruct;
  parentName: string;
  filterText?: string;
  expanded?: boolean;
  showRoot?: boolean;
}

export default function InputElements(props: InputElementsProps): JSX.Element {
  const { messageStruct, parentName = "undefined", filterText = "", expanded = true, showRoot = true } = props;

  const settingsCtx = useContext(SettingsContext);
  const [arrayCount, setArrayCount] = useState<number>(0);
  const [idSuffix] = useState<string>(`${parentName}-${messageStruct.name}`);
  const [expandedElement, setExpanded] = useState<boolean>(
    expanded || (messageStruct.type !== "std_msgs/Header" && messageStruct.type !== "builtin_interfaces/Time")
  );
  const [useNow, setUseNow] = useState<boolean>(messageStruct.useNow || false);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  useEffect(() => {
    if (messageStruct?.value !== undefined) {
      if (Array.isArray(messageStruct.value)) {
        setArrayCount(messageStruct?.value?.length);
      }
    }
  }, [messageStruct.value]);

  useEffect(() => {
    messageStruct.useNow = useNow;
  }, [useNow]);

  const addArrayElement = useCallback(
    function (): void {
      if (!messageStruct.value) {
        messageStruct.value = [];
      }
      if (Array.isArray(messageStruct.value)) {
        messageStruct.value.push(JSON.parse(JSON.stringify(messageStruct.def)));
      } else {
        console.error(`can't add array element, it is set to ${messageStruct.value}`);
      }
    },
    [messageStruct]
  );

  const removeArrayElement = useCallback(
    function (): void {
      if (Array.isArray(messageStruct.value)) {
        messageStruct.value.pop();
      } else {
        console.error(`can't remove array element, it is set to ${messageStruct.value}`);
      }
    },
    [messageStruct.value]
  );

  function structToSearchableString(msgStruct: TRosMessageStruct): string {
    if (msgStruct) {
      const subStr = msgStruct.def?.map((item: TRosMessageStruct) => structToSearchableString(item)).join(" ") || "";
      return `${msgStruct.name} ${msgStruct.type} ${subStr}`;
    }
    return "";
  }

  // component visibility based on filter input text
  const [isVisible, setVisible] = useState("");
  useEffect(() => {
    let vis = "";
    if (filterText.length > 0) {
      // TODO: improve struct search by ignoring non-visible fields
      const re = new RegExp(filterText, "i");
      const pos = structToSearchableString(messageStruct).search(re);
      vis = pos !== -1 ? "" : "none";
    }
    setVisible(vis);
  }, [filterText, messageStruct]);

  if (messageStruct === undefined || messageStruct === null) {
    return <></>;
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

  const fieldType = messageStruct.type.replace(/\[\d*\]/, "");
  if (messageStruct.is_array && !messageStruct?.value) {
    messageStruct.value = "";
  }

  // const toggleAccordion = () => {
  //   setExpanded((prev) => !prev);
  // };

  // create element depending on the base type defined in Components
  if (typeof Components[fieldType] !== "undefined") {
    if (isVisible !== "") {
      return <></>;
    }
    return React.createElement(Components[fieldType], {
      id: idSuffix,
      messageStruct,
      filterText,
    });
  }
  // create input mask for an element of the array
  function createListEntry(element, index: number): JSX.Element {
    return (
      <Stack direction="column" spacing={1} key={`liststack-${messageStruct.name}-${index}`}>
        <Divider textAlign="left">{`${messageStruct.name}[${index}]`}</Divider>
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
    if (showRoot) {
      return (
        <Accordion
          id={`accordion-${idSuffix}`}
          expanded={expandedElement}
          // use this action to collapse by click on whole summary
          onChange={(_e, state) => {
            setExpanded(state);
          }}
          sx={{ display: `${isVisible}` }}
        >
          <Stack direction="row" alignItems="center" sx={{ backgroundColor: backgroundColor }}>
            <AccordionSummary
              // aria-controls="ttyd_panel-content"
              id={`accordion-summary-${idSuffix}`}
              sx={{ pl: 0 }}
              expandIcon={
                <ArrowForwardIosSharpIcon
                  sx={{ fontSize: "0.9rem" }}
                  // use this onclick action to collapse by click on the arrow
                  // onClick={() => {
                  //   toggleAccordion();
                  // }}
                />
              }
            >
              <Stack direction="row" spacing={1} sx={{ width: "100%" }} alignItems="center">
                <FormLabel
                  sx={{
                    typography: "body1",
                    color: blue[800],
                    textOverflow: "ellipsis",
                    overflow: "hidden",
                    whiteSpace: "nowrap",
                    minWidth: "5em",
                  }}
                >
                  {messageStruct.name && `${messageStruct.name}`}
                  {!messageStruct.name && `${messageStruct.type}`}
                </FormLabel>
                <FormLabel
                  sx={{
                    fontSize: "0.8em",
                    fontWeight: "bold",
                    color: "#3cb371",
                  }}
                >
                  {messageStruct.is_array && `[${arrayCount > 0 ? arrayCount : ""}]`}
                </FormLabel>
                <FormLabel
                  sx={{
                    typography: "body1",
                    textOverflow: "ellipsis",
                    overflow: "hidden",
                    whiteSpace: "nowrap",
                    fontSize: "0.8em",
                    minWidth: "2em",
                  }}
                >
                  {messageStruct.name && messageStruct.type && `${messageStruct.type}`}
                </FormLabel>

                {!messageStruct.is_array && messageStruct.type === "builtin_interfaces/Time" && (
                  <FormControlLabel
                    control={
                      <Checkbox
                        checked={useNow}
                        onChange={(event) => {
                          setUseNow((prev) => !prev);
                          event.stopPropagation();
                        }}
                        onClick={(event) => {
                          event.stopPropagation();
                        }}
                      />
                    }
                    label="use now"
                    labelPlacement="end"
                  />
                )}
              </Stack>
            </AccordionSummary>
            {messageStruct.is_array && (
              <ButtonGroup sx={{ maxHeight: "24px" }}>
                <Button
                  onClick={(event) => {
                    addArrayElement();
                    setArrayCount((messageStruct.value as TRosMessageStruct[]).length);
                    event.stopPropagation();
                  }}
                >
                  +
                </Button>
                <Button
                  onClick={(event) => {
                    if (arrayCount > 0) {
                      removeArrayElement();
                      setArrayCount((messageStruct.value as TRosMessageStruct[]).length);
                      event.stopPropagation();
                    }
                  }}
                >
                  -
                </Button>
              </ButtonGroup>
            )}
          </Stack>
          <AccordionDetails id={`accordion-details-${idSuffix}`}>
            <Stack direction="column" spacing={1}>
              {!messageStruct.is_array &&
                !useNow &&
                messageStruct.def.map((struct: TRosMessageStruct) => (
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
                (messageStruct.value as TRosMessageStruct[]).map((item, index) => createListEntry(item, index))}
            </Stack>
          </AccordionDetails>
        </Accordion>
      );
    } else {
      return (
        <Stack direction="column" spacing={1}>
          {!messageStruct.is_array &&
            !useNow &&
            messageStruct.def.map((struct: TRosMessageStruct) => (
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
            (messageStruct.value as TRosMessageStruct[]).map((item, index) => createListEntry(item, index))}
        </Stack>
      );
    }
  }
  if (!messageStruct.name) {
    // {`No input for "${messageStruct.type}" defined.`}
    return <></>;
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
