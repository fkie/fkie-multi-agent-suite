import { createTheme, CssBaseline, Stack, ThemeOptions } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import { useCallback, useContext, useEffect, useState } from "react";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import SingleTerminalPanel from "../../pages/NodeManager/panels/SingleTerminalPanel";
import { CmdType, cmdTypeFromString } from "../../providers";
import TerminalProvider from "../../providers/TerminalProvider";
// load default style for flexlayout-react. Dark/Light theme changes are in ./themes
import "../../App.scss";
import { darkThemeDef, lightThemeDef } from "../../themes";

interface ITerminalInfo {
  id: string;
  provider: TerminalProvider;
  info: CmdType;
  node: string;
  screen: string;
  cmd: string;
}

export default function TerminalApp() {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef as ThemeOptions));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef as ThemeOptions));
  const [paramInfo, setParamInfo] = useState<ITerminalInfo | null>(null);

  const initProvider = useCallback(async () => {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const id = urlParams.get("id");
    const host = urlParams.get("host");
    const port = urlParams.get("port");
    const info = urlParams.get("info");
    const node = urlParams.get("node");
    const screen = urlParams.get("screen");
    const cmd = urlParams.get("cmd");
    if (!host || !port) {
      logCtx.error(`invalid address ${host}:${port}`, "", false);
      return;
    }
    if (!id) {
      logCtx.error(`no id found ${id}`, "", false);
      return;
    }
    const nodeName = node ? node : "bash";
    document.title = `${info} - ${nodeName}`;
    const prov = new TerminalProvider(settingsCtx, host, "", parseInt(port), false, logCtx);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setParamInfo({
        id: id,
        provider: prov,
        info: cmdTypeFromString(info),
        node: node ? node : "",
        screen: screen ? screen : "",
        cmd: cmd ? cmd : "",
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "", false);
    }
  }, [setParamInfo]);

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
    setDarkTheme(createTheme(darkThemeDef as ThemeOptions));
    setLightTheme(createTheme(lightThemeDef as ThemeOptions));
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // Anything in here is fired on component mount.
    window.addEventListener("error", handleWindowError);
    window.terminalManager?.onClose((id: string) => {
      // close window on stop request
      window.terminalManager?.close(id);
    });
    initProvider();
    return () => {
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
        {paramInfo && rosCtx.mapProviderRosNodes.size > 0 && (
          <SingleTerminalPanel
            id={paramInfo.id}
            type={paramInfo.info}
            providerId={paramInfo.provider.id}
            nodeName={paramInfo.node}
            screen={paramInfo.screen}
            cmd={paramInfo.cmd}
          />
        )}
      </Stack>
    </ThemeProvider>
  );
}
