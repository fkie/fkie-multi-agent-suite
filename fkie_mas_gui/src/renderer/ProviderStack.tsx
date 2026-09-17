// load default style for flexlayout-react. Dark/Light theme changes are in ./themes
import { CssBaseline, createTheme } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import { SnackbarProvider } from "notistack";
import React, { useEffect, useState } from "react";
import { BrowserRouter } from "react-router-dom";

import packageJson from "../../package.json";
import darkThemeUrl from "./assets/flexlayout/alpha_dark.css?url";
import lightThemeUrl from "./assets/flexlayout/alpha_light.css?url";
import { ElectronProvider } from "./context/ElectronContext";
import { LoggingProvider } from "./context/LoggingContext";
import { NavigationProvider } from "./context/NavigationContext";
import { RosProviderReact } from "./context/RosContext";
import { useSetting } from "./hooks/useSetting";
import { setupEditorWindowBridge } from "./monaco/EditorEventBridge";
import { darkThemeDef, lightThemeDef } from "./themes";

export default function ProviderStack({ children }: { children: React.ReactNode }): JSX.Element {
  const [useDarkMode] = useSetting<boolean>("useDarkMode");
  const [fontSize] = useSetting<number>("fontSize");
  const [tooltipDelay] = useSetting<number>("tooltipEnterDelay");
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
    lightThemeDef.typography.fontSize = fontSize;
    lightThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] = fontSize;
    darkThemeDef.typography.fontSize = fontSize;
    darkThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] = fontSize;
    setDarkTheme(createTheme(darkThemeDef));
    setLightTheme(createTheme(lightThemeDef));
  }, [fontSize]);

  useEffect(() => {
    if (tooltipDelay === undefined) return;
    if (lightThemeDef.components.MuiTooltip?.defaultProps) {
      lightThemeDef.components.MuiTooltip.defaultProps.enterDelay = tooltipDelay;
      lightThemeDef.components.MuiTooltip.defaultProps.enterNextDelay = tooltipDelay;
    }
    if (darkThemeDef.components.MuiTooltip?.defaultProps) {
      darkThemeDef.components.MuiTooltip.defaultProps.enterDelay = tooltipDelay;
      darkThemeDef.components.MuiTooltip.defaultProps.enterNextDelay = tooltipDelay;
    }
    setDarkTheme(createTheme(darkThemeDef));
    setLightTheme(createTheme(lightThemeDef));
  }, [tooltipDelay]);

  // FlexLayout-Theme umschalten
  useEffect(() => {
    const link = document.getElementById("flexlayout-theme") as HTMLLinkElement | null;
    if (!link) return;

    link.href = useDarkMode ? darkThemeUrl : lightThemeUrl;
  }, [useDarkMode]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.APP_VERSION = packageJson.version;
    window.addEventListener("error", handleWindowError);
    setupEditorWindowBridge();
    return (): void => {
      // Anything in here is fired on component unmount.
      window.removeEventListener("error", handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider theme={useDarkMode ? darkTheme : lightTheme}>
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
