import { Alert, Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle, Stack } from "@mui/material";
import * as monaco from "monaco-editor";
import { useCallback, useEffect, useState } from "react";
import { useCustomEventListener } from "react-custom-events";

import { useAlwaysCurrentRef } from "@/renderer/hooks/useAlwaysCurrentRef";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useMonacoInitContext } from "@/renderer/hooks/useMonacoInitContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { SaveResult } from "@/renderer/monaco/types";
import { TFileRange, TLaunchArg } from "@/types";
import DraggablePaper from "../../components/UI/DraggablePaper";
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

export default function EditorApp(): JSX.Element {
  const logCtx = useLoggingContext();
  const monacoInitCtx = useMonacoInitContext();
  const monacoCtx = monacoInitCtx.monacoCtx;
  const rosCtx = useRosContext();
  const settingsCtx = useSettingsContext();
  const logCtxRef = useAlwaysCurrentRef(logCtx);
  const settingsCtxRef = useAlwaysCurrentRef(settingsCtx);
  const [connectingHost, setConnectingHost] = useState<string>("");
  const [launchInfo, setLaunchInfo] = useState<TLaunchInfo | null>(null);
  const [dirtyModels, setDirtyModels] = useState<monaco.editor.ITextModel[]>([]);

  async function initProvider(): Promise<void> {
    rosCtx.setShowSnackbarReloadLaunchNotification(false);
    rosCtx.setShowSnackbarBinaryChangedNotification(false);
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
      logCtx.error(`invalid address ${host}:${port}`, "");
      return;
    }
    if (!path) {
      logCtx.error(`launch path ${path}`, "");
      return;
    }
    if (!rootLaunch) {
      logCtx.error(`root launch path ${rootLaunch}`, "");
      return;
    }
    if (!id) {
      logCtx.error(`no id found ${id}`, "");
      return;
    }
    document.title = `Editor - ${getFileName(rootLaunch)}`;
    const prov = new EditorProvider(logCtxRef, settingsCtxRef, host, "", Number.parseInt(port), false);
    setConnectingHost(`${prov.connection.uri}`);
    if (await prov.init()) {
      rosCtx.addProvider(prov);
      setConnectingHost("")
      setLaunchInfo({
        id: id,
        provider: prov,
        launch: path,
        rootLaunch: rootLaunch,
        fileRange: fileRange,
        launchArgs: launchArgs,
      });
    } else {
      logCtx.error(`connection to ${host}:${port} failed`, "");
    }
  }

  useCustomEventListener(EVENT_CLOSE_COMPONENT, (data: { id: string }) => {
    const mTab = monacoCtx.getModifiedFilesByEditor(data.id);
    if (mTab.length > 0) {
      setDirtyModels(mTab);
      return;
    }
    window.editorManager?.close(data.id);
  });

  useEffect(() => {
    // Anything in here is fired on component mount.
    initProvider();
    return (): void => {
      if (launchInfo) {
        launchInfo.provider.close();
      }
      // Anything in here is fired on component unmount.
    };
  }, []);

  const saveAllDirty = useCallback(async () => {
    // save all files
    const results: SaveResult[] = await Promise.all(dirtyModels.map((model) => monacoCtx.saveFile(model)));

    const allTabs = new Set<string>();
    const failedTabs = new Set<string>();

    for (const { editorIds = [], result } of results) {
      for (const editorId of editorIds) {
        allTabs.add(editorId);
        if (!result) {
          failedTabs.add(editorId);
        }
      }
    }
    // close editor if no errors
    if (launchInfo && failedTabs.size === 0) {
      window.editorManager?.close(launchInfo.id);
    }

    setDirtyModels([]);
  }, [dirtyModels, launchInfo, monacoCtx.saveFile]);

  return (
    <Stack width="100%" height="100vh">
      {connectingHost && (
        <Alert severity="info" style={{ minWidth: 0 }}>
          connecting to {connectingHost}
        </Alert>
      )}
      {launchInfo && (
        <FileEditorPanel
          editorId={launchInfo.id}
          provider={launchInfo.provider}
          rootFilePath={launchInfo.rootLaunch}
          currentFilePath={launchInfo.launch}
          fileRange={launchInfo.fileRange}
          launchArgs={launchInfo.launchArgs}
        />
      )}
      {launchInfo && dirtyModels.length > 0 && (
        <Dialog
          open={dirtyModels.length > 0}
          onClose={() => setDirtyModels([])}
          fullWidth
          maxWidth="sm"
          PaperComponent={DraggablePaper}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
            Changed Files
          </DialogTitle>

          <DialogContent aria-label="list">
            {monacoCtx
              .modelRegistry()
              ?.getEditorsByModels(dirtyModels)
              .map((editorId) => {
                const models = monacoCtx.modelRegistry()?.getByEditor(editorId);
                return (
                  <DialogContentText key={editorId} id="alert-dialog-description">
                    {`There are ${models?.size} unsaved files in "${getBaseName(editorId)}" tab.`}
                  </DialogContentText>
                );
              })}
          </DialogContent>

          <DialogActions>
            <Button
              color="warning"
              onClick={() => {
                const editorIds = monacoCtx.modelRegistry()?.getEditorsByModels(dirtyModels);
                monacoCtx.closeEditors(editorIds);
                setDirtyModels([]);
                window.editorManager?.close(launchInfo.id);
              }}
            >
              Don&apos;t save
            </Button>
            <Button
              color="primary"
              onClick={() => {
                setDirtyModels([]);
              }}
            >
              Cancel
            </Button>

            <Button
              autoFocus
              color="primary"
              onClick={async () => {
                saveAllDirty();
              }}
            >
              Save all
            </Button>
          </DialogActions>
        </Dialog>
      )}
    </Stack>
  );
}
