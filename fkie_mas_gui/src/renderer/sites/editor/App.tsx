import {
  Button,
  createTheme,
  CssBaseline,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  Stack,
  ThemeOptions,
} from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import DraggablePaper from "../../components/UI/DraggablePaper";
import { LoggingContext } from "../../context/LoggingContext";
import { ModifiedTabsInfo, MonacoContext } from "../../context/MonacoContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getBaseName, getFileName } from "../../models";
import FileEditorPanel from "../../pages/NodeManager/panels/FileEditorPanel";
import EditorProvider from "../../providers/EditorProvider";
import { EVENT_CLOSE_COMPONENT } from "../../utils/events";
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
  const monacoCtx = useContext(MonacoContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [lightTheme, setLightTheme] = useState(createTheme(lightThemeDef as ThemeOptions));
  const [darkTheme, setDarkTheme] = useState(createTheme(darkThemeDef as ThemeOptions));
  const [launchInfo, setLaunchInfo] = useState<ILaunchInfo | null>(null);
  const [modifiedEditorTabs, setModifiedEditorTabs] = useState<ModifiedTabsInfo[]>([]);
  const dialogRef = useRef(null);
  let escapePressCount = 0;

  const initProvider = useCallback(async () => {
    const queryString = window.location.search;
    console.log(`queryString: ${queryString}`);
    const urlParams = new URLSearchParams(queryString);
    const id = urlParams.get("id");
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
    if (!id) {
      logCtx.error(`no id found ${id}`, "", false);
      return;
    }
    document.title = `Editor - ${getFileName(rootLaunch)}`;
    const prov = new EditorProvider(settingsCtx, host, "", parseInt(port), false, logCtx);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setLaunchInfo({
        id: id,
        provider: prov,
        launch: path,
        rootLaunch: rootLaunch,
        fileRange: fileRange,
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "", false);
    }
  }, [setLaunchInfo]);

  useCustomEventListener(
    EVENT_CLOSE_COMPONENT,
    (data: { id: string }) => {
      const mTab = monacoCtx.getModifiedFilesByTab(data.id);
      if (mTab) {
        setModifiedEditorTabs([mTab]);
        return;
      }
      window.editorManager.close(data.id);
    },
    [monacoCtx]
  );

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      escapePressCount++;
      if (escapePressCount === 2) {
        window.close();
      }
      // Reset after 500 ms
      setTimeout(() => {
        escapePressCount = 0;
      }, 500);
    }
  });

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
      settingsCtx.get("fontSize") as string;
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
        {launchInfo && modifiedEditorTabs.length > 0 && (
          <Dialog
            open={modifiedEditorTabs.length > 0}
            onClose={() => setModifiedEditorTabs([])}
            fullWidth
            maxWidth="sm"
            ref={dialogRef}
            PaperProps={{
              component: DraggablePaper,
              dialogRef: dialogRef,
            }}
            aria-labelledby="draggable-dialog-title"
          >
            <DialogTitle className="handle" style={{ cursor: "move" }} id="draggable-dialog-title">
              Changed Files
            </DialogTitle>

            <DialogContent aria-label="list">
              {modifiedEditorTabs.map((tab) => {
                return (
                  <DialogContentText key={tab.tabId} id="alert-dialog-description">
                    {`There are ${tab.uriPaths.length} unsaved files in "${getBaseName(tab.tabId)}" editor.`}
                  </DialogContentText>
                );
              })}
            </DialogContent>

            <DialogActions>
              <Button
                color="warning"
                onClick={() => {
                  setModifiedEditorTabs([]);
                  window.editorManager.close(launchInfo.id);
                }}
              >
                Don&apos;t save
              </Button>
              <Button
                color="primary"
                onClick={() => {
                  setModifiedEditorTabs([]);
                }}
              >
                Cancel
              </Button>

              <Button
                autoFocus
                color="primary"
                onClick={async () => {
                  // save all files
                  const result = await Promise.all(
                    modifiedEditorTabs.map(async (tab) => {
                      const tabResult = await monacoCtx.saveModifiedFilesOfTabId(tab.tabId);
                      return tabResult;
                    })
                  );
                  // TODO inform about error on failed save
                  let failed = false;
                  result.forEach((item) => {
                    if (item[0].result) {
                      // OK
                    } else {
                      failed = true;
                    }
                  });
                  if (!failed) {
                    window.editorManager.close(launchInfo.id);
                  }
                  setModifiedEditorTabs([]);
                }}
              >
                Save all
              </Button>
            </DialogActions>
          </Dialog>
        )}
      </Stack>
    </ThemeProvider>
  );
}
