import { ThemeOptions } from "@mui/material";

export type ThemeOptionsExt = ThemeOptions & {
  typography: {
    fontSize: number;
  };
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: object;
      };
    };
  };
};
