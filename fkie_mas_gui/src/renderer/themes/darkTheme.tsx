import darkScrollbar from "@mui/material/darkScrollbar";
import { ThemeOptionsExt } from "./ThemeOptionsExt";

const background = "#282828";

const jsonProperty = "#ffd700";
const jsonIndex = "#00e0e0";
const jsonNumber = "#00e0e0";
const jsonString = "#abe338";
const jsonBoolean = "#ffa07a";
const jsonNull = "#ffa07a";

const darkThemeDef: ThemeOptionsExt = {
  palette: {
    mode: "dark",
    background: {
      default: background,
    },
  },
  typography: {
    fontSize: 12,
    fontFamily: "IBM Plex, sans",
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        html: {
          ...darkScrollbar(undefined),
          // scrollbarWidth for Firefox
          scrollbarWidth: "thin",
        },
        body: {
          // flexlayout-react theme changes
          "& .flexlayout__layout": {
            "--color-text": "#eeeeee",
            "--color-background": "black",
            "--color-base": "black",
            "--color-1": "#121212",
            "--color-2": "#1a1a1a",
            "--color-3": "#262626",
            "--color-4": "#333333",
            "--color-5": "#404040",
            "--color-6": "#4d4d4d",
            "--color-drag1": "rgb(207, 232, 255)",
            "--color-drag2": "rgb(183, 209, 181)",
            "--color-drag1-background": "rgba(128, 128, 128, 0.15)",
            "--color-drag2-background": "rgba(128, 128, 128, 0.15)",
            // '--color-tabset-background': 'var(--color-background)',
            // '--color-tabset-header-background': 'var(--color-background)',
            // '--color-border-background': 'var(--color-background)',
            // '--color-splitter': 'var(--color-1)',
            // '--color-splitter-drag': 'var(--color-4)',
            // '--color-drag-rect-border': 'var(--color-6)',
            // '--color-drag-rect-background': 'var(--color-4)',
            // '--color-popup-unselected-background': 'white',
            // '--color-popup-selected-background': 'var(--color-3)',
            "--color-edge-marker": "gray",
            "--color-edge-icon": "#eee",
          },
          "& .flexlayout__tabset_header": {
            boxShadow: "inset 0 0 3px 0 rgba(136, 136, 136, 0.54)",
          },
          "& .flexlayout__tabset-selected": {
            backgroundImage: "linear-gradient(var(--color-background), var(--color-4))",
          },
          "& .flexlayout__tab_button_top": {
            boxShadow: "inset -2px 0px 5px rgba(0, 0, 0, 0.1)",
            borderTopLeftRadius: "1px",
            borderBottomRightRadius: "1px",
          },
          "& .flexlayout__tab_button_bottom": {
            boxShadow: "inset -2px 0px 5px rgba(0, 0, 0, 0.1)",
            borderTopLeftRadius: "1px",
            borderBottomRightRadius: "1px",
          },
          "& .flexlayout__border_button": {
            boxShadow: "inset 0 0 5px rgba(0, 0, 0, 0.15)",
            borderRadius: "1px",
          },
          // react18-json-view styles
          ".json-view": {
            display: "block",
            color: "#d1d1d1",
            textAlign: "left",
          },
          ".json-view .json-view--property": {
            color: jsonProperty,
          },
          ".json-view .json-view--index": {
            color: jsonIndex,
          },
          ".json-view .json-view--number": {
            color: jsonNumber,
          },
          ".json-view .json-view--string": {
            color: jsonString,
          },
          ".json-view .json-view--boolean": {
            color: jsonBoolean,
          },
          ".json-view .json-view--null": {
            color: jsonNull,
          },

          ".json-view .jv-indent": {
            paddingLeft: "1em",
          },
          ".json-view .jv-chevron": {
            display: "inline-block",
            verticalAlign: "-20%",
            cursor: "pointer",
            opacity: 0.4,
            width: "1em",
            height: "1em",
          },
          ":is(.json-view .jv-chevron:hover, .json-view .jv-size:hover + .jv-chevron)": {
            opacity: 0.8,
          },
          ".json-view .jv-size": {
            cursor: "pointer",
            opacity: 0.4,
            fontSize: "0.875em",
            fontStyle: "italic",
            marginLeft: "0.5em",
            verticalAlign: "-5%",
            lineHeight: 1,
          },

          ".json-view :is(.json-view--copy, .json-view--edit), .json-view .json-view--link svg": {
            display: "none",
            width: "1em",
            height: "1em",
            marginLeft: "0.25em",
            cursor: "pointer",
          },

          ".json-view .json-view--input": {
            width: "120px",
            marginLeft: "0.25em",
            borderRadius: "4px",
            border: "1px solid currentColor",
            padding: "0px 4px",
            fontSize: "87.5%",
            lineHeight: 1.25,
            background: "transparent",
          },
          ".json-view .json-view--deleting": {
            outline: "1px solid #da0000",
            backgroundColor: "#da000011",
            textDecorationLine: "line-through",
          },

          ":is(.json-view:hover, .json-view--pair:hover) > :is(.json-view--copy, .json-view--edit), :is(.json-view:hover, .json-view--pair:hover) > .json-view--link svg":
            {
              display: "inline-block",
            },

          ".json-view .jv-button": {
            background: "transparent",
            outline: "none",
            border: "none",
            cursor: "pointer",
            color: "inherit",
          },
          ".json-view .cursor-pointer": {
            cursor: "pointer",
          },

          ".json-view svg": {
            verticalAlign: "-10%",
          },
          ".jv-size-chevron ~ svg": {
            verticalAlign: "-16%",
          },
          // END: react18-json-view styles
        },
      },
    },
    MuiContainer: {
      styleOverrides: {
        // Name of the slot
        root: {
          // Some CSS
          paddingLeft: "4px",
          paddingRight: "4px",
          paddingTop: "1px",
          paddingBottom: "1px",
        },
      },
    },
    MuiButton: {
      defaultProps: {
        size: "small",
      },
    },
    MuiFilledInput: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiFormControl: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiFormHelperText: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiIconButton: {
      defaultProps: {
        size: "small",
      },
    },
    MuiInputBase: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiInputLabel: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiListItem: {
      defaultProps: {
        dense: true,
      },
    },
    MuiOutlinedInput: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiFab: {
      defaultProps: {
        size: "small",
      },
    },
    MuiTable: {
      defaultProps: {
        size: "small",
      },
    },
    MuiTextField: {
      defaultProps: {
        margin: "dense",
      },
    },
    MuiToolbar: {
      defaultProps: {
        variant: "dense",
      },
    },
  },
};

export default darkThemeDef;
