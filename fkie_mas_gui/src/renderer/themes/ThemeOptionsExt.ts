import { ThemeOptions } from "@mui/material";

export type ThemeOptionsExt = ThemeOptions & {
  backgroundColor: string;
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
