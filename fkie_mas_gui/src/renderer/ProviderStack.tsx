// load default style for flexlayout-react. Dark/Light theme changes are in ./themes
import { createTheme, CssBaseline } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import "flexlayout-react/style/gray.css";
import { SnackbarProvider } from "notistack";
import React, { useContext, useEffect, useState } from "react";
import { BrowserRouter } from "react-router-dom";
import { ElectronProvider } from "./context/ElectronContext";
import { LoggingProvider } from "./context/LoggingContext";
import { NavigationProvider } from "./context/NavigationContext";
import { RosProviderReact } from "./context/RosContext";
import { SettingsContext } from "./context/SettingsContext";
import { darkThemeDef, lightThemeDef } from "./themes";

export default function ProviderStack({ children }: { children: React.ReactNode }): JSX.Element {
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef));

  const handleWindowError = (e): void => {
    // fix "ResizeObserver loop limit exceeded" while change size of the editor
    if (
      ["ResizeObserver loop limit exceeded", "ResizeObserver loop completed with undelivered notifications."].includes(
        e.message
      )
    ) {
      const resizeObserverErrDiv = document.getElementById("webpack-dev-server-client-overlay-div");
      const resizeObserverErr = document.getElementById("webpack-dev-server-client-overlay");
      if (resizeObserverErr) {
        resizeObserverErr.setAttribute("style", "display: none");
      }
      if (resizeObserverErrDiv) {
        resizeObserverErrDiv.setAttribute("style", "display: none");
      }
    }
  };

  useEffect(() => {
    // update font size globally
    const fontSize = settingsCtx?.get("fontSize");
    if (fontSize) {
      lightThemeDef.typography.fontSize = fontSize as number;
      lightThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] = fontSize;
      darkThemeDef.typography.fontSize = fontSize as number;
      darkThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] = fontSize;
      setDarkTheme(createTheme(darkThemeDef));
      setLightTheme(createTheme(lightThemeDef));
    }
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener("error", handleWindowError);
    return (): void => {
      // Anything in here is fired on component unmount.
      window.removeEventListener("error", handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider theme={settingsCtx.get("useDarkMode") ? darkTheme : lightTheme}>
      <CssBaseline />
      <SnackbarProvider
        maxSnack={4}
        autoHideDuration={5000}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "right",
        }}
        dense
        // preventDuplicate <= Do not use here: this causes the editor to lose focus. Use in enqueueSnackbar instead.
      >
        <LoggingProvider>
          <ElectronProvider>
            <RosProviderReact>
              <NavigationProvider>
                <BrowserRouter
                  future={{
                    v7_startTransition: true,
                    v7_relativeSplatPath: true,
                  }}
                >
                  {children}
                </BrowserRouter>
              </NavigationProvider>
            </RosProviderReact>
          </ElectronProvider>
        </LoggingProvider>
      </SnackbarProvider>
    </ThemeProvider>
  );
}
