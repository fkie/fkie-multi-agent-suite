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

export default function ProviderStack({ children }: { children: React.ReactNode }) {
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef));

  const handleWindowError = (e) => {
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
    lightThemeDef.typography.fontSize = settingsCtx.get("fontSize") as number;
    lightThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] =
      `${settingsCtx.get("fontSize")}`;
    setDarkTheme(createTheme(darkThemeDef));
    setLightTheme(createTheme(lightThemeDef));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener("error", handleWindowError);
    return () => {
      // Anything in here is fired on component unmount.
      window.removeEventListener("error", handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider theme={settingsCtx.get("useDarkMode") ? darkTheme : lightTheme}>
      <CssBaseline />
      <SnackbarProvider
        maxSnack={4}
        autoHideDuration={4000}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "right",
        }}
        dense
        preventDuplicate
      >
        <LoggingProvider>
          <ElectronProvider>
            <RosProviderReact>
              <NavigationProvider>
                <BrowserRouter>{children}</BrowserRouter>
              </NavigationProvider>
            </RosProviderReact>
          </ElectronProvider>
        </LoggingProvider>
      </SnackbarProvider>
    </ThemeProvider>
  );
}
