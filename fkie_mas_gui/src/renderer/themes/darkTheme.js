import darkScrollbar from "@mui/material/darkScrollbar";

const background = "#282828";

export const darkThemeJson = {
  scheme: "dark",
  base00: background,
  base01: "#333333",
  base02: "#555555",
  base03: "#777777",
  base04: "#999999",
  base05: "#bbbbbb",
  base06: "#dddddd",
  base07: "#ffffff",
  base08: "#ff4136",
  base09: "#ff851b",
  base0A: "#ffdc00",
  base0B: "#2ecc40",
  base0C: "#7fdbff",
  base0D: "#0074d9",
  base0E: "#b10dc9",
  base0F: "#85144b",
};

const darkThemeDef = {
  backgroundColor: background,
  palette: {
    mode: "dark",
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
