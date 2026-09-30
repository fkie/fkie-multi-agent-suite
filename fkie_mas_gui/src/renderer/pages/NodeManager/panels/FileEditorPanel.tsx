import * as Monaco from "@monaco-editor/react";
import { Stack } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { editor } from "monaco-editor";
import { ForwardedRef, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import SplitPane, { Pane, SashContent } from "split-pane-react";
import { useMonacoEditor } from "@/renderer/hooks/editor/useMonacoEditor";
import "split-pane-react/esm/themes/default.css";

import { AlertsBar, EditorSidebar, EditorToolbar, THistoryModel } from "@/renderer/components/FileEditorPanel";
import { PendingEditStyles } from "@/renderer/components/FileEditorPanel/PendingEditStyles";
import {
  EVENT_EDITOR_SELECT_RANGE,
  emitCloseComponent,
  TEventEditorSelectRange,
} from "@/renderer/components/layout/events";
import { useEditorKeyboard } from "@/renderer/hooks/editor/useEditorKeyboard";
import { useEditorLayout } from "@/renderer/hooks/editor/useEditorLayout";
import { usePendingParameterEdit } from "@/renderer/hooks/editor/usePendingParameterEdit";
import { useIncludedFiles } from "@/renderer/hooks/useIncludedFiles";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useMonacoInitContext } from "@/renderer/hooks/useMonacoInitContext";
import { useSetting } from "@/renderer/hooks/useSetting";
import { getFileName } from "@/renderer/models";
import { locateNodeParameter } from "@/renderer/monaco/ParameterEditing";
import { cleanUpXmlComment } from "@/renderer/monaco/setup";
import { TModelResult } from "@/renderer/monaco/types";
import { createEditorId, createUriPath, fileFromUriPath } from "@/renderer/monaco/utils";
import { Provider } from "@/renderer/providers";
import { EventProviderLaunchLoaded, EventProviderPathEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_LAUNCH_LOADED, EVENT_PROVIDER_PATH_EVENT } from "@/renderer/providers/eventTypes";
import { TFileRange, TLaunchArg, TParameterRequest } from "@/types";
import "./FileEditorPanel.css";

type TAlertNotification = {
  message?: string;
  messageSeverity?: "success" | "info" | "warning" | "error";
};

/** file extensions that may contain includes and therefore need an include scan */
const INCLUDE_AWARE_EXTENSIONS: string[] = ["launch", "xml", "xacro", "py"];

interface FileEditorPanelProps {
  editorId: string;
  provider: Provider;
  rootFilePath: string;
  currentFilePath: string;
  fileRange: TFileRange | null;
  launchArgs: TLaunchArg[];
  topLevelLaunchArgs: TLaunchArg[];
  selectParameter?: TParameterRequest;
}

export default function FileEditorPanel(props: FileEditorPanelProps): JSX.Element {
  const {
    editorId,
    provider,
    rootFilePath,
    currentFilePath,
    fileRange,
    launchArgs,
    topLevelLaunchArgs,
    selectParameter,
  } = props;

  const logCtx = useLoggingContext();
  const monacoInitCtx = useMonacoInitContext();
  const monacoCtx = monacoInitCtx.monacoCtx;

  /** the monaco instance - a value ref, never a callback */
  const editorIdRef = useRef(editorId);
  const editorRef = useRef<editor.IStandaloneCodeEditor | undefined>(undefined);
  const closeEditorsRef = useRef(monacoCtx.closeEditors);
  /** becomes true in onMount and re-triggers the initial load */
  const [editorMounted, setEditorMounted] = useState<boolean>(false);
  /** guards the initial load against parent re-renders with fresh prop objects */
  const loadedKeyRef = useRef<string>("");
  /** the selectParameter prop is a one-shot request on panel open */
  const selectParameterAppliedRef = useRef<boolean>(false);

  const [providerName, setProviderName] = useState<string>("");
  const [packageName, setPackageName] = useState<string>("");
  const [currentFileState, setCurrentFileState] = useState({ name: "", requesting: false, path: "" });

  const [selectionRange, setSelectionRange] = useState<TFileRange | undefined>();
  const [currentLaunchArgs, setCurrentLaunchArgs] = useState<TLaunchArg[]>(launchArgs);
  const [notificationDescription, setNotificationDescription] = useState<TAlertNotification | undefined>();
  const [isDarkMode] = useSetting<boolean>("useDarkMode");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [historyModel, setHistoryModel] = useState<THistoryModel | undefined>();
  const [eventButton, setEventButton] = useState<React.MouseEvent<HTMLDivElement, MouseEvent> | undefined>(undefined);
  const [keyboardEvent, setKeyboardEvent] = useState<React.KeyboardEvent | undefined>();
  /** save markers - must be readable synchronously, the path event races the render */
  const savedFilesRef = useRef<Set<string>>(new Set());
  /** latest-ref for the imperative monaco keybinding registration */
  const saveModelRef = useRef<((model: editor.ITextModel) => Promise<void>) | null>(null);

  // a requested parameter is applied by an effect, never inside setEditorModel:
  // the model must be active and dirty-tracked before the insert happens
  const [parameterRequest, setParameterRequest] = useState<TParameterRequest | null>(null);

  const {
    panelRef,
    toolbarRef,
    alertRef,
    fontSize,

    sideBarWidth,
    setSideBarWidth,
    sideBarMinSize,

    editorWidth,
    setEditorWidth,
    editorHeight,

    savedSideBarUserWidth,
    setSavedSideBarUserWidth,
  } = useEditorLayout();

  const includeResolver = useIncludedFiles(provider, rootFilePath, topLevelLaunchArgs);
  const { includedFiles, fetchIncludedFiles, clearIncludedFiles } = includeResolver;

  // side effects belong into effects, never into the render body
  useEffect(() => {
    monacoCtx.setResolver(editorId, includeResolver);
  }, [editorId, includeResolver, monacoCtx]);

  const mEditor = useMonacoEditor({
    editorId,
    editorRef,
    saveModel: (model: editor.ITextModel) => {
      void saveModelRef.current?.(model);
    },
  });
  const { setCurrentModel, activeModel, activeModelDirty, modifiedFiles, initialized: editorInitialized } = mEditor;

  const onPendingEditAccepted = useCallback(
    (request: TParameterRequest): void => {
      const model = editorRef.current?.getModel();
      if (!model) return;
      const result = locateNodeParameter(model, request, provider.rosVersion === "1" ? "1" : "2");
      if (result.found && result.range) setSelectionRange(result.range);
    },
    [provider]
  );

  const onPendingEditReverted = useCallback((): void => {
    // nothing to do - the widget restores the previous content itself
  }, []);

  const { startPendingEdit, rejectPendingEdit, clearPendingState, pendingEditWidget } = usePendingParameterEdit(
    editorRef,
    monacoCtx.monaco,
    onPendingEditAccepted,
    onPendingEditReverted
  );

  const ownUriPaths: Set<string> = useMemo(
    () =>
      new Set([
        createUriPath(provider.id, rootFilePath),
        ...includedFiles.map((f) => createUriPath(provider.id, f.inc_path)),
      ]),
    [provider.id, rootFilePath, includedFiles]
  );

  const onCloseEditor = useCallback((): void => {
    emitCloseComponent({ id: createEditorId(rootFilePath, provider.id) });
  }, [rootFilePath, provider.id]);

  useEditorKeyboard(onCloseEditor);

  // keep refs up to date without re-running the cleanup effect
  useEffect(() => {
    editorIdRef.current = editorId;
    closeEditorsRef.current = monacoCtx.closeEditors;
  }, [editorId, monacoCtx.closeEditors]);

  // dispose own models on unmount only - editorId is stable for the panel lifetime
  useEffect(() => {
    return (): void => {
      editorRef.current?.setModel(null);
      // dispose all own models
      closeEditorsRef.current([editorIdRef.current]);
    };
  }, []);

  useEffect(() => {
    const ed = editorRef.current;
    if (!selectionRange || !ed) return;

    const { startLineNumber, endLineNumber, startColumn, endColumn } = selectionRange;
    const isSingleCursor = startLineNumber === endLineNumber && startColumn === endColumn;
    const adjustedEndLineNumber = isSingleCursor ? endLineNumber + 1 : endLineNumber;

    ed.revealRangeInCenter(selectionRange);
    ed.setPosition({ lineNumber: startLineNumber, column: startColumn });
    ed.setSelection({ startLineNumber, endLineNumber: adjustedEndLineNumber, startColumn, endColumn });
    ed.focus();
  }, [selectionRange]);

  const updatePackageName = useCallback(
    async (uriPath: string, forcePackageReload: boolean = false): Promise<void> => {
      if (forcePackageReload) {
        await provider.getPackageList(false);
      }
      const name = provider.getPackageName(fileFromUriPath(uriPath));
      if (!name && !forcePackageReload) {
        await updatePackageName(uriPath, true); // the recursive call must be awaited
        return;
      }
      setPackageName(name || "");
    },
    [provider]
  );

  /** set the current model to the editor based on [uriPath], and update its decorations */
  const setEditorModel = useCallback(
    async (
      uriPath: string,
      range: TFileRange | null = null,
      modelLaunchArgs: TLaunchArg[] = [],
      forceReload: boolean = false,
      appendToHistory: boolean = true
    ): Promise<boolean> => {
      if (!uriPath) return false;

      // an unconfirmed parameter insert must not survive a model switch or a reload
      if (editorRef.current?.getModel()?.uri.path !== uriPath || forceReload) {
        rejectPendingEdit(); // idempotent - no hasPendingEdit dependency needed
      }

      setNotificationDescription({ message: "Getting file from provider...", messageSeverity: "info" });
      const result: TModelResult = await monacoCtx.getModel(editorId, uriPath, forceReload);
      setNotificationDescription(undefined);
      setCurrentFileState({ name: getFileName(uriPath), requesting: false, path: uriPath });

      if (!result.model) {
        logCtx.error(`Could not get model for file: ${uriPath}`, result.error || "");
        setNotificationDescription({
          message: result.error || `Could not get model for file: ${uriPath}`,
          messageSeverity: "error",
        });
        setCurrentModel(null);
        return false;
      }

      setCurrentModel(result.model);
      await updatePackageName(result.model.uri.path);

      if (range) {
        setSelectionRange(range);
      }
      setCurrentLaunchArgs(modelLaunchArgs);
      if (appendToHistory) {
        setHistoryModel({ uriPath: result.model.uri.path, range: range, launchArgs: modelLaunchArgs });
      }
      return true;
    },
    [editorId, monacoCtx, logCtx, setCurrentModel, rejectPendingEdit, updatePackageName]
  );

  const saveModel = useCallback(
    async (editorModel: editor.ITextModel): Promise<void> => {
      if (editorModel.isDisposed()) return;
      clearPendingState(); // idempotent
      const uriPath = editorModel.uri.path;
      // mark synchronously - the path event of this very save may arrive before the next render
      savedFilesRef.current.add(uriPath);

      const saveResult = await monacoCtx.saveFile(editorModel);
      if (saveResult.result) {
        setCurrentModel(editorModel);
        return;
      }
      savedFilesRef.current.delete(uriPath);
      setNotificationDescription({
        message: `Could not save file: ${saveResult.message}`,
        messageSeverity: "warning",
      });
      logCtx.error("Could not save file", saveResult.message, "save failed");
    },
    [clearPendingState, monacoCtx, setCurrentModel, logCtx]
  );

  // keep the latest save action reachable for the keybinding without re-registering it
  useEffect(() => {
    saveModelRef.current = saveModel;
  }, [saveModel]);

  // Most important function:
  //  - get the content of [filePath] and create the corresponding monaco model
  //  - scan for includes when the extension may contain them
  // Models are downloaded per request. A recursive text search downloads all files.
  const loadFiles = useCallback(
    async (filePath: string, range: TFileRange | null, modelLaunchArgs: TLaunchArg[]): Promise<void> => {
      if (!monacoCtx.monaco) {
        setNotificationDescription({ message: "monaco is not yet available", messageSeverity: "error" });
        return;
      }
      // validate the argument, not the prop
      if (!filePath) {
        setNotificationDescription({ message: "[filePath] Invalid file path", messageSeverity: "warning" });
        return;
      }
      if (!rootFilePath) {
        setNotificationDescription({ message: "[rootFilePath] Invalid file path", messageSeverity: "warning" });
        return;
      }
      if (!provider.host()) {
        logCtx.error("The provider does not have configured any host.", "Please check your provider configuration");
        setNotificationDescription({
          message: "The provider does not have configured any host.",
          messageSeverity: "warning",
        });
        return;
      }

      setProviderName(provider.name());
      setNotificationDescription({ message: "Getting file from provider...", messageSeverity: "info" });
      setCurrentFileState({ name: getFileName(filePath), requesting: true, path: filePath });

      const result: TModelResult = await monacoCtx.getModel(editorId, filePath, false);
      setCurrentFileState({ name: getFileName(filePath), requesting: false, path: filePath });

      if (!result.model) {
        setNotificationDescription({
          message: result.error || `Could not get file: [${filePath}]`,
          messageSeverity: "warning",
        });
        return;
      }

      // check the extension first - no include roundtrip for plain files
      if (INCLUDE_AWARE_EXTENSIONS.includes(result.file?.extension || "")) {
        const includes = await fetchIncludedFiles();
        // a failed include scan must not prevent showing the file
        if (!includes.result) {
          setNotificationDescription({ message: includes.error, messageSeverity: "warning" });
        }
      } else {
        clearIncludedFiles();
      }

      await setEditorModel(result.model.uri.path, range, modelLaunchArgs);
    },
    [monacoCtx, editorId, rootFilePath, provider, logCtx, fetchIncludedFiles, clearIncludedFiles, setEditorModel]
  );

  const reloadCurrentFile = useCallback(async (): Promise<void> => {
    const path = editorRef.current?.getModel()?.uri.path;
    if (!path) return;
    rejectPendingEdit(); // drop the insert before forceReload disposes the model
    const ok = await setEditorModel(path, selectionRange ?? null, currentLaunchArgs, true, false);
    if (ok) {
      logCtx.success(`File reloaded [${getFileName(path)}]`, "", `${getFileName(path)} reloaded`);
    }
  }, [selectionRange, currentLaunchArgs, logCtx, setEditorModel, rejectPendingEdit]);

  /** select the parameter definition, or insert it as pending edit */
  const applyParameterRequest = useCallback(
    (request: TParameterRequest): void => {
      const model = editorRef.current?.getModel();
      if (!model) return;
      const result = locateNodeParameter(model, request, provider.rosVersion === "1" ? "1" : "2");

      if (result.found && result.range) {
        setSelectionRange(result.range);
        return;
      }
      if (result.insert && !monacoCtx.isReadOnly(model)) {
        startPendingEdit(request, result.insert);
        return;
      }
      setNotificationDescription({
        message: result.error || `Parameter [${request.paramName}] could not be inserted (read-only file)`,
        messageSeverity: "warning",
      });
    },
    [provider, monacoCtx, startPendingEdit]
  );

  // one-shot: queue the selectParameter prop as soon as its model is really in the editor
  useEffect(() => {
    if (!selectParameter || selectParameterAppliedRef.current) return;
    if (!activeModel || editorRef.current?.getModel()?.uri.path !== activeModel.uri.path) return;
    selectParameterAppliedRef.current = true;
    setParameterRequest(selectParameter);
  }, [selectParameter, activeModel]);

  // apply a queued parameter request once the model is active in the editor
  useEffect(() => {
    if (!parameterRequest || !activeModel) return;
    // the editor must already show this model, otherwise the insert hits the wrong buffer
    if (editorRef.current?.getModel()?.uri.path !== activeModel.uri.path) return;
    setParameterRequest(null);
    applyParameterRequest(parameterRequest);
  }, [parameterRequest, activeModel, applyParameterRequest]);

  // initial load - editorMounted makes this retry after onMount
  useEffect(() => {
    if (!monacoInitCtx.initialized || !editorInitialized || !editorMounted) return;
    const key = `${provider.id}:${currentFilePath}`;
    if (loadedKeyRef.current === key) return; // load each file only once per panel
    loadedKeyRef.current = key;
    void loadFiles(currentFilePath, fileRange, launchArgs);
  }, [
    monacoInitCtx.initialized,
    editorInitialized,
    editorMounted,
    provider.id,
    currentFilePath,
    fileRange,
    launchArgs,
    loadFiles,
  ]);

  useEffect(() => {
    if (monacoInitCtx.initialized && monacoCtx.monaco) {
      monacoCtx.monaco.editor.setTheme(isDarkMode ? "vs-ros-dark" : "vs-ros-light");
    }
  }, [monacoInitCtx.initialized, isDarkMode, monacoCtx.monaco]);

  // report dirty state of the active model to the external editor window
  useEffect(() => {
    if (!activeModel) return;
    window.editorManager?.changed(
      createEditorId(rootFilePath, provider.id),
      fileFromUriPath(activeModel.uri.path),
      activeModelDirty
    );
  }, [activeModel, activeModelDirty, rootFilePath, provider.id]);

  /** select node definition on event */
  useCustomEventListener(
    EVENT_EDITOR_SELECT_RANGE,
    (data: TEventEditorSelectRange) => {
      if (data.editorId !== editorId) return;
      void setEditorModel(data.filePath, data.fileRange, data.launchArgs).then((ok) => {
        if (ok && data.selectParameter) {
          selectParameterAppliedRef.current = true;
          setParameterRequest(data.selectParameter);
        }
      });
    },
    [editorId, setEditorModel]
  );

  useCustomEventListener(
    EVENT_PROVIDER_LAUNCH_LOADED,
    (data: EventProviderLaunchLoaded) => {
      // reload included files to update provided parameters
      if (data.provider.id !== provider.id || data.launchFile !== rootFilePath) return;
      const activePath = editorRef.current?.getModel()?.uri.path;
      const filePath = activePath ? fileFromUriPath(activePath) : currentFilePath;
      void loadFiles(filePath, null, currentLaunchArgs);
    },
    [provider.id, rootFilePath, currentFilePath, currentLaunchArgs, loadFiles]
  );

  /** handle events caused by changed files */
  useCustomEventListener(
    EVENT_PROVIDER_PATH_EVENT,
    async (data: EventProviderPathEvent) => {
      if (data.provider.id !== provider.id) return; // ignore events from other providers

      const changedUri: string = createUriPath(provider.id, data.path.srcPath);
      if (!ownUriPaths.has(changedUri)) return;

      // our own save - consume the marker and stop, never recreate the model here
      if (savedFilesRef.current.has(changedUri)) {
        savedFilesRef.current.delete(changedUri);
        return;
      }

      // check the dirty state BEFORE anything touches the buffer
      if (modifiedFiles.includes(changedUri)) {
        setNotificationDescription({
          message: `${getFileName(changedUri)} was changed on remote host! Save your file or reload manually!`,
          messageSeverity: "warning",
        });
        return;
      }

      // reuse the regular load path - it disposes and re-attaches the model consistently
      if (editorRef.current?.getModel()?.uri.path === changedUri) {
        await setEditorModel(changedUri, null, currentLaunchArgs, true, false);
        return;
      }
      // inactive and clean model: just refresh the cache entry
      await monacoCtx.getModel(editorId, changedUri, true);
    },
    [provider, ownUriPaths, modifiedFiles, editorId, monacoCtx, currentLaunchArgs, setEditorModel]
  );

  const debouncedWidthUpdate = useDebounceCallback((newWidth: number) => {
    setEditorWidth(newWidth);
  }, 50);

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(panelRef.current.getBoundingClientRect().width - sideBarWidth);
    }
  }, [sideBarWidth, debouncedWidthUpdate, panelRef]);

  function onKeyDown(event: React.KeyboardEvent): void {
    setKeyboardEvent(event);
  }

  function handleEditorDidMount(ed: editor.IStandaloneCodeEditor): void {
    editorRef.current = ed;
    setEditorMounted(true); // retry trigger - loadFiles no longer bails out silently
  }

  const handleEditorChange = useCallback(
    (_value: string | undefined, event: editor.IModelContentChangedEvent): void => {
      // use the editor model directly - activeModel may still be stale on the first change
      const model = editorRef.current?.getModel();
      if (!model || model.isDisposed()) return;
      cleanUpXmlComment(event.changes, model);
      // refresh dirty state (toolbar, sidebar, external window)
      setCurrentModel(model);
    },
    [setCurrentModel]
  );

  const onStateChange = useCallback(
    (collapsed: boolean): void => {
      if (collapsed) {
        setSideBarWidth(sideBarMinSize);
      } else if (sideBarWidth <= sideBarMinSize) {
        setSideBarWidth(savedSideBarUserWidth);
      }
    },
    [sideBarMinSize, sideBarWidth, savedSideBarUserWidth, setSideBarWidth]
  );

  return (
    <Stack
      direction="row"
      height="100%"
      width="100%"
      onKeyDown={onKeyDown}
      onMouseDown={(event) => setEventButton(event)}
      ref={panelRef as ForwardedRef<HTMLDivElement>}
      overflow="hidden"
    >
      <SplitPane
        sizes={[sideBarWidth, "auto"]}
        onChange={([size]) => {
          if (size !== sideBarMinSize && size >= sideBarMinSize) {
            setSavedSideBarUserWidth(size);
          }
          setSideBarWidth(size);
        }}
        split="vertical"
        resizerSize={6}
        sashRender={(_index, active) => <SashContent className={`sash-wrap-line ${active ? "active" : "inactive"}`} />}
      >
        <Pane minSize={sideBarMinSize} style={{ backgroundColor: backgroundColor }}>
          <EditorSidebar
            editorId={editorId}
            provider={provider}
            rootFilePath={rootFilePath}
            includedFiles={includedFiles}
            selectedFile={{ uriPath: activeModel?.uri.path || "", launchArgs: currentLaunchArgs }}
            modifiedUriPaths={modifiedFiles}
            sideBarWidth={sideBarWidth}
            keyboardEvent={keyboardEvent}
            panelRef={panelRef}
            onStateChange={onStateChange}
          />
        </Pane>
        <Pane>
          <Stack sx={{ flex: 1, margin: 0 }} overflow="hidden">
            <EditorToolbar
              refEl={toolbarRef as ForwardedRef<HTMLDivElement>}
              providerId={provider.id}
              providerName={providerName}
              packageName={packageName}
              rootFilePath={rootFilePath}
              currentFileState={currentFileState}
              activeModel={activeModel}
              activeModelDirty={activeModelDirty}
              historyModel={historyModel}
              includedFiles={includedFiles}
              modifiedFiles={modifiedFiles}
              eventButton={eventButton}
              setEditorModel={setEditorModel}
              saveModel={saveModel}
              reloadCurrentFile={() => void reloadCurrentFile()}
            />
            <PendingEditStyles />
            {/* portal target of the pending edit buttons - must stay mounted */}
            {/* the span wrapper avoids the prop-types warning of mui containers */}
            <span style={{ display: "contents" }}>{pendingEditWidget}</span>
            <AlertsBar
              refEl={alertRef as ForwardedRef<HTMLDivElement>}
              activeModel={activeModel}
              message={notificationDescription?.message}
              messageSeverity={notificationDescription?.messageSeverity}
              onClose={() => setNotificationDescription(undefined)}
            />
            <Monaco.Editor
              key="editor"
              height={editorHeight}
              width={editorWidth}
              theme={isDarkMode ? "vs-ros-dark" : "vs-ros-light"}
              onMount={handleEditorDidMount}
              onChange={handleEditorChange}
              options={{
                // TODO: make a global config for these parameters
                readOnly: activeModel ? monacoCtx.isReadOnly(activeModel) : false,
                colorDecorators: true,
                mouseWheelZoom: true,
                scrollBeyondLastLine: false,
                smoothScrolling: false,
                wordWrap: "off",
                fontSize: fontSize,
                minimap: { enabled: false },
                selectOnLineNumbers: true,
                guides: { bracketPairs: true },
                definitionLinkOpensInPeek: false,
                comments: { ignoreEmptyLines: false, insertSpace: true },
              }}
            />
          </Stack>
        </Pane>
      </SplitPane>
    </Stack>
  );
}
