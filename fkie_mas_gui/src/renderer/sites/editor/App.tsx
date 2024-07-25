// import { useWindowHeight } from '@react-hook/window-size/throttled';
import { useCallback, useContext, useEffect, useState } from "react";
// https://github.com/azouaoui-med/react-pro-sidebar/blob/master/storybook/Playground.tsx
import { createTheme, CssBaseline, Stack, ThemeOptions } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getFileName } from "../../models";
import FileEditorPanel from "../../pages/NodeManager/panels/FileEditorPanel";
import EditorProvider from "../../providers/EditorProvider";
// load default style for flexlayout-react. Dark/Light theme changes are in ./themes
import "../../App.scss";
import { darkThemeDef, lightThemeDef } from "../../themes";

interface IRange {
  startLineNumber: number;
  endLineNumber: number;
  startColumn: number;
  endColumn: number;
}

interface ILaunchInfo {
  id: string;
  provider: EditorProvider;
  launch: string;
  rootLaunch: string;
  fileRange: IRange | null;
}

export default function EditorApp() {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef as ThemeOptions));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef as ThemeOptions));
  const [launchInfo, setLaunchInfo] = useState<ILaunchInfo | null>(null);

  const initProvider = useCallback(async () => {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const path = urlParams.get("path");
    const host = urlParams.get("host");
    const port = urlParams.get("port");
    const rootLaunch = urlParams.get("root");
    const startLineStr = urlParams.get("sl");
    const endLineStr = urlParams.get("el");
    const startColumnStr = urlParams.get("sc");
    const endColumnStr = urlParams.get("ec");
    let fileRange: IRange | null = null;
    if (startLineStr && endLineStr && startColumnStr && endColumnStr) {
      fileRange = {
        startLineNumber: parseInt(startLineStr),
        endLineNumber: parseInt(endLineStr),
        startColumn: parseInt(startColumnStr),
        endColumn: parseInt(endColumnStr),
      };
    }
    console.log(`path: ${path}`);
    console.log(`startLineNumber: ${endColumnStr}`);
    if (!host || !port) {
      logCtx.error(`invalid address ${host}:${port}`, "", false);
      return;
    }
    if (!path) {
      logCtx.error(`launch path ${path}`, "", false);
      return;
    }
    if (!rootLaunch) {
      logCtx.error(`root launch path ${rootLaunch}`, "", false);
      return;
    }
    document.title = `Editor - ${getFileName(rootLaunch)}`;
    const prov = new EditorProvider(settingsCtx, host, "", parseInt(port), false, logCtx);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setLaunchInfo({
        id: `${host}-${port}-${path}`,
        provider: prov,
        launch: path,
        rootLaunch: rootLaunch,
        fileRange: fileRange,
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "", false);
    }
  }, [setLaunchInfo]);

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
    lightThemeDef.typography.fontSize = settingsCtx.get("fontSize");
    lightThemeDef.components.MuiCssBaseline.styleOverrides.body["& .flexlayout__layout"]["--font-size"] =
      settingsCtx.get("fontSize");
    setDarkTheme(createTheme(darkThemeDef as ThemeOptions));
    setLightTheme(createTheme(lightThemeDef as ThemeOptions));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener("error", handleWindowError);
    initProvider();
    return () => {
      if (launchInfo) {
        launchInfo.provider.close();
      }
      // Anything in here is fired on component unmount.
      window.removeEventListener("error", handleWindowError);
    };
  }, []);

  return (
    <ThemeProvider theme={settingsCtx.get("useDarkMode") ? darkTheme : lightTheme}>
      <CssBaseline />
      <Stack
        width="100%"
        height="100vh"
        // style={{
        //   position: "absolute",
        //   left: 2,
        //   top: 2,
        //   right: 2,
        //   bottom: 2,
        // }}
      >
        {launchInfo && (
          <FileEditorPanel
            tabId={launchInfo.id}
            providerId={launchInfo.provider.id}
            rootFilePath={launchInfo.rootLaunch}
            currentFilePath={launchInfo.launch}
            fileRange={launchInfo.fileRange}
          />
        )}
      </Stack>
    </ThemeProvider>
  );
}
