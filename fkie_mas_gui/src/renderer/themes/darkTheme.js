import { createTheme } from '@mui/material';
import darkScrollbar from '@mui/material/darkScrollbar';

const background = '#282828'

const darkTheme = createTheme({
  backgroundColor: background,
  palette: {
    mode: 'dark',
  },
  typography: {
    fontFamily: 'IBM Plex, sans',
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        html: {
          ...darkScrollbar(undefined),
          //scrollbarWidth for Firefox
          scrollbarWidth: 'thin',
        },
        body: {
          // rc-dock theme changes
          '& .dragging-layer > *:first-of-type': {
            boxShadow:
              '0 0 8px rgba(255, 255, 255, 0.1), inset 0 0 8px rgba(255, 255, 255, 0.1)',
          },
          '& .dock-tab': {
            background: background,
            borderBottom: '1px solid #434343;',
          },
          '& .dock-tab:hover': {
            color: '#40a5ed',
          },
          '& .dock-top .dock-bar': {
            background: background,
            borderBottom: '1px solid #383838',
          },
          '& .dock-tab-close-btn': {
            color: '#444',
          },
          '& .dock-tab-close-btn:focus': {
            color: '#fff',
          },
          '& .dock-nav-more': {
            color: 'rgba(255, 255, 255, 0.85)',
          },
          '& .dock-dropdown': {
            color: 'rgba(255, 255, 255, 0.85)',
          },
          '& .dock-dropdown-menu': {
            backgroundColor: background,
          },
          '& .dock-dropdown-menu-item': {
            color: 'rgba(255, 255, 255, 0.85)',
          },
          '& .dock-dropdown-menu-item:hover': {
            color: '#2b2b2b',
          },
          '& .dock-panel': {
            color: 'rgba(255, 255, 255, 0.85)',
            background: background,
            border: '1px solid #434343',
          },
          '& .dock-panel-drag-size-b-r': {
            backgroundImage: `url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='17' height='17' viewBox='0 0 17 17'%3E%3Cpath fill='rgba(255, 255, 255, 0.15)' d='M2 12 L12 2 L12 12z'/%3E%3C/svg%3E")`,
          },
          '& .dock-fbox > .dock-panel': {
            boxShadow: '0 0 4px #444',
          },
          '& .dock-mbox > .dock-panel': {
            boxShadow: '0 0 4px #444',
          },
          '& .dock-layout > .dock-drop-indicator': {
            background: '#125d94',
            boxShadow: '0 0 4px #434343',
          },
          '& .dock-drop-layer .dock-drop-square': {
            color: '#444',
            background: background,
          },
          '& .dock-drop-layer .dock-drop-square .dock-drop-square-box': {
            border: '1px solid #434343',
          },
          '& .dock-drop-layer .dock-drop-square-dropping': {
            background: '#125d94',
          },
          '& .dock-panel-max-btn:before, & .dock-panel-min-btn:before': {
            border: '2px solid #444',
          },
          '& .dock-panel.dock-style-main .dock-bar': {
            borderBottom: '1px solid #282828;',
          },
          '& .dock-panel.dock-style-main .dock-tab': {
            background: background,
          },
          '& .dock-panel.dock-style-card .dock-tab': {
            border: '1px solid #434343',
          },
          '& .dock-panel.dock-style-card .dock-tab.dock-tab-active': {
            background: background,
            borderBottom: '1px solid #141414;',
          },
          '& .dock-panel.dock-style-card .dock-bar': {
            borderBottom: '1px solid #434343;',
          },
          '& .dock-panel.dock-style-card .dock-ink-bar': {
            background: background,
          },
        },
      },
    },
    MuiContainer: {
      styleOverrides: {
        // Name of the slot
        root: {
          // Some CSS
          paddingLeft: '4px',
          paddingRight: '4px',
          paddingTop: '1px',
          paddingBottom: '1px',
        },
      },
    },
    MuiButton: {
      defaultProps: {
        size: 'small',
      },
    },
    MuiFilledInput: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiFormControl: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiFormHelperText: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiIconButton: {
      defaultProps: {
        size: 'small',
      },
    },
    MuiInputBase: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiInputLabel: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiListItem: {
      defaultProps: {
        dense: true,
      },
    },
    MuiOutlinedInput: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiFab: {
      defaultProps: {
        size: 'small',
      },
    },
    MuiTable: {
      defaultProps: {
        size: 'small',
      },
    },
    MuiTextField: {
      defaultProps: {
        margin: 'dense',
      },
    },
    MuiToolbar: {
      defaultProps: {
        variant: 'dense',
      },
    },
  },
});

export default darkTheme;
