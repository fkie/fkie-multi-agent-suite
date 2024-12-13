import { TFileRange, TLaunchArg } from "@/types";
import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle, Stack } from "@mui/material";
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import DraggablePaper from "../../components/UI/DraggablePaper";
import { LoggingContext } from "../../context/LoggingContext";
import { ModifiedTabsInfo, MonacoContext, MonacoProvider } from "../../context/MonacoContext";
import { RosContext } from "../../context/RosContext";
import { SettingsContext } from "../../context/SettingsContext";
import { getBaseName, getFileName } from "../../models";
import { EVENT_CLOSE_COMPONENT } from "../../pages/NodeManager/layout/events";
import FileEditorPanel from "../../pages/NodeManager/panels/FileEditorPanel";
import EditorProvider from "../../providers/EditorProvider";

type TLaunchInfo = {
  id: string;
  provider: EditorProvider;
  launch: string;
  rootLaunch: string;
  fileRange: TFileRange | null;
  launchArgs: TLaunchArg[];
};

export default function EditorApp() {
  const logCtx = useContext(LoggingContext);
  const monacoCtx = useContext(MonacoContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [launchInfo, setLaunchInfo] = useState<TLaunchInfo | null>(null);
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
    const range = urlParams.get("range");
    const launchArgsStr = urlParams.get("launchArgs");
    let fileRange: TFileRange | null = null;
    let launchArgs: TLaunchArg[] = [];
    if (range) {
      fileRange = JSON.parse(range);
    }
    if (launchArgsStr) {
      launchArgs = JSON.parse(launchArgsStr);
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
        launchArgs: launchArgs,
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
      window.editorManager?.close(data.id);
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

  useEffect(() => {
    // Anything in here is fired on component mount.
    initProvider();
    return () => {
      if (launchInfo) {
        launchInfo.provider.close();
      }
      // Anything in here is fired on component unmount.
    };
  }, []);

  return (
    <MonacoProvider>
      <Stack width="100%" height="100vh">
        {launchInfo && (
          <FileEditorPanel
            tabId={launchInfo.id}
            providerId={launchInfo.provider.id}
            rootFilePath={launchInfo.rootLaunch}
            currentFilePath={launchInfo.launch}
            fileRange={launchInfo.fileRange}
            launchArgs={launchInfo.launchArgs}
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
            <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
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
                  window.editorManager?.close(launchInfo.id);
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
                    window.editorManager?.close(launchInfo.id);
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
    </MonacoProvider>
  );
}
