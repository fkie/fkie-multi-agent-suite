const background = "#F0F0F0";

export const lightThemeJson = {
  scheme: "light",
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

const lightThemeDef = {
  backgroundColor: background,
  palette: {
    mode: "light",
  },
  typography: {
    fontSize: 12,
    fontFamily: "IBM Plex, sans",
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        html: {
          // scrollbarWidth for Firefox
          scrollbarWidth: "thin",
        },
        body: {
          // flexlayout-react theme changes
          "& .flexlayout__layout": {
            "--font-size": "medium",
            "--color-text": "black",
            "--color-background": "white",
            "--color-base": "white",
            "--color-1": "#f7f7f7",
            "--color-2": "#f0f0f0",
            "--color-3": "#e9e9e9",
            "--color-4": "#e2e2e2",
            "--color-5": "#dbdbdb",
            "--color-6": "#d4d4d4",
            "--color-drag1": "rgb(95, 134, 196)",
            "--color-drag2": "rgb(119, 166, 119)",
            "--color-drag1-background": "rgba(95, 134, 196, 0.1)",
            "--color-drag2-background": "rgba(119, 166, 119, 0.075)",
            // '--color-tabset-background': 'var(--color-background)',
            // '--color-tabset-header-background': 'var(--color-background)',
            // '--color-border-background': 'var(--color-background)',
            // '--color-splitter': 'var(--color-1)',
            // '--color-splitter-drag': 'var(--color-4)',
            // '--color-drag-rect-border': 'var(--color-6)',
            // '--color-drag-rect-background': 'var(--color-4)',
            // '--color-popup-unselected-background': 'white',
            // '--color-popup-selected-background': 'var(--color-3)',
            "--color-edge-marker": "#aaa",
            "--color-edge-icon": "#555",
          },
          "& .flexlayout__tabset_header": {
            boxShadow: "None",
          },
          "& .flexlayout__tabset-selected": {
            backgroundImage: "None",
          },
          "& .flexlayout__tab_button_top": {
            boxShadow: "inset -2px 0px 5px rgba(0, 0, 0, 0.1)",
            borderTopLeftRadius: "1px",
            borderTopRightRadius: "1px",
          },
          "& .flexlayout__tab_button_bottom": {
            boxShadow: "inset -2px 0px 5px rgba(0, 0, 0, 0.1)",
            borderBottomLeftRadius: "1px",
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

export default lightThemeDef;
