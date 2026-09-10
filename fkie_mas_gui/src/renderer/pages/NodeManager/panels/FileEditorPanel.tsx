import { useMonacoEditor } from "@/renderer/hooks/editor/useMonacoEditor";
import * as Monaco from "@monaco-editor/react";
import { Stack } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { editor } from "monaco-editor";
import { ForwardedRef, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useCustomEventListener } from "react-custom-events";
import SplitPane, { Pane, SashContent } from "split-pane-react";
import "split-pane-react/esm/themes/default.css";

import { AlertsBar, EditorSidebar, EditorToolbar, THistoryModel } from "@/renderer/components/FileEditorPanel";
import { PendingEditStyles } from "@/renderer/components/FileEditorPanel/PendingEditStyles";
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
import {
  emitCloseComponent,
  EVENT_EDITOR_SELECT_RANGE,
  TEventEditorSelectRange,
} from "@/renderer/pages/NodeManager/layout/events";
import { Provider } from "@/renderer/providers";
import { EventProviderLaunchLoaded, EventProviderPathEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_LAUNCH_LOADED, EVENT_PROVIDER_PATH_EVENT } from "@/renderer/providers/eventTypes";
import { TFileRange, TLaunchArg, TParameterRequest } from "@/types";
import "./FileEditorPanel.css";

type TAlertNotification = {
  message?: string;
  messageSeverity?: "success" | "info" | "warning" | "error";
};

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

  const editorRef = useRef<editor.IStandaloneCodeEditor>();

  const [providerName, setProviderName] = useState<string>("");
  const [packageName, setPackageName] = useState<string>("");
  const [currentFileState, setCurrentFileState] = useState({ name: "", requesting: false, path: "" });

  const [selectionRange, setSelectionRange] = useState<TFileRange>();
  const [currentLaunchArgs, setCurrentLaunchArgs] = useState<TLaunchArg[]>(launchArgs);
  const [notificationDescription, setNotificationDescription] = useState<TAlertNotification | undefined>();
  const [isDarkMode] = useSetting<boolean>("useDarkMode");
  const [backgroundColor] = useSetting<string>("backgroundColor");
  const [historyModel, setHistoryModel] = useState<THistoryModel | undefined>();
  const [eventButton, setEventButton] = useState<React.MouseEvent<HTMLDivElement, MouseEvent> | undefined>(undefined);
  const [keyboardEvent, setKeyboardEvent] = useState<React.KeyboardEvent | undefined>();
  const [savedFiles, setSavedFiles] = useState<string[]>([]);

  // a requested parameter is applied by an effect, never inside setEditorModel:
  // the model must be active and dirty-tracked before the insert happens
  const [parameterRequest, setParameterRequest] = useState<TParameterRequest | null>(null);

  const { hasPendingEdit, startPendingEdit, rejectPendingEdit, clearPendingState, pendingEditWidget } =
    usePendingParameterEdit(
      editorRef,
      monacoCtx.monaco,
      (request) => {
        // onAccepted
        const model = editorRef.current?.getModel();
        if (!model) return;
        const result = locateNodeParameter(model, request, provider.rosVersion === "1" ? "1" : "2");
        if (result.found && result.range) setSelectionRange(result.range);
      },
      () => {
        // onReverted
      }
    );

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
  monacoCtx.setResolver(editorId, includeResolver);

  const mEditor = useMonacoEditor({
    editorId: editorId,
    editorRef: editorRef,
    saveModel: (model) => {
      saveModel(model);
    },
  });

  // the selectParameter prop is a one-shot request on panel open
  const selectParameterAppliedRef = useRef<boolean>(false);

  const ownUriPaths: Set<string> = useMemo(() => {
    const result = new Set([
      createUriPath(provider.id, rootFilePath),
      ...(includeResolver.includedFiles?.map((f) => createUriPath(provider.id, f.inc_path)) || []),
    ]);
    return result;
  }, [provider, rootFilePath, includeResolver.includedFiles]);

  useEditorKeyboard(() => {
    const id = createEditorId(rootFilePath, provider.id);
    emitCloseComponent({ id: id });
  });

  useEffect(() => {
    return (): void => {
      editorRef.current?.setModel(null);
      // dispose all own models
      monacoCtx.closeEditors([editorId]);
    };
  }, []);

  useEffect(() => {
    const editor = editorRef.current;
    if (!selectionRange || !editor) return;

    const { startLineNumber, endLineNumber, startColumn, endColumn } = selectionRange;

    const isSingleCursor = startLineNumber === endLineNumber && startColumn === endColumn;

    const adjustedEndLineNumber = isSingleCursor ? endLineNumber + 1 : endLineNumber;

    editor.revealRangeInCenter(selectionRange);
    editor.setPosition({
      lineNumber: startLineNumber,
      column: startColumn,
    });

    editor.setSelection({
      startLineNumber,
      endLineNumber: adjustedEndLineNumber,
      startColumn,
      endColumn,
    });

    editor.focus();
  }, [selectionRange]);

  // set the current model to the editor based on [uriPath], and update its decorations
  const setEditorModel = useCallback(
    async (
      uriPath: string,
      range: TFileRange | null = null,
      launchArgs: TLaunchArg[] = [],
      forceReload: boolean = false,
      appendToHistory: boolean = true
    ): Promise<boolean> => {
      if (!uriPath) return false;
      // an unconfirmed parameter insert must not survive a model switch or a reload
      const currentUriPath = editorRef.current?.getModel()?.uri.path;
      if (hasPendingEdit && (currentUriPath !== uriPath || forceReload)) {
        rejectPendingEdit();
      }
      setNotificationDescription({ message: "Getting file from provider...", messageSeverity: "info" });
      // If model does not exist, try to fetch it
      const result: TModelResult = await monacoCtx.getModel(editorId, uriPath, forceReload);
      setNotificationDescription(undefined);
      setCurrentFileState({ name: getFileName(uriPath), requesting: false, path: uriPath });

      // get model from path if exists
      if (!result.model) {
        logCtx.error(`Could not get model for file: ${uriPath}`, "");
        setNotificationDescription({
          message: result?.error || `Could not get model for file: ${uriPath}`,
          messageSeverity: "error",
        });
        mEditor.setCurrentModel(null);
        return false;
      }
      mEditor.setCurrentModel(result.model);

      // set package name
      updatePackageName(result.model.uri.path);

      // set range if available
      if (range) {
        setSelectionRange(range);
      }
      setCurrentLaunchArgs(launchArgs);
      if (appendToHistory) {
        setHistoryModel({ uriPath: result.model.uri.path, range: range, launchArgs: launchArgs });
      }

      // only apply the initial parameter request once, never on reloads
      // the insert itself is done by the effect below, after react committed the model
      if (selectParameter && !selectParameterAppliedRef.current) {
        selectParameterAppliedRef.current = true;
        setParameterRequest(selectParameter);
      }

      return true;
    },
    [mEditor, monacoCtx, editorId, logCtx, selectParameter, hasPendingEdit, rejectPendingEdit]
  );

  const reloadCurrentFile = useCallback(async () => {
    if (!mEditor.activeModel?.uri.path) return;
    const path = mEditor.activeModel.uri.path;
    // drop the pending insert before the model gets disposed by forceReload
    if (hasPendingEdit) rejectPendingEdit();
    const result = await setEditorModel(path, selectionRange, currentLaunchArgs, true, false);
    if (result) {
      logCtx.success(`File reloaded [${getFileName(path)}]`, "", `${getFileName(path)} reloaded`);
    }
  }, [mEditor, selectionRange, currentLaunchArgs, logCtx, setEditorModel, hasPendingEdit, rejectPendingEdit]);

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
        // the dirty event may have fired before react committed the active model
        // mEditor.refreshDirtyState();
        return;
      }
      setNotificationDescription({
        message: result.error || `Parameter [${request.paramName}] could not be inserted (read-only file)`,
        messageSeverity: "warning",
      });
    },
    [provider, monacoCtx, startPendingEdit]
  );

  // apply a queued parameter request as soon as the model is active in the editor
  useEffect(() => {
    if (!parameterRequest) return;
    const model = mEditor.activeModel;
    if (!model) return;
    // the editor must already show this model, otherwise the insert hits the wrong buffer
    if (editorRef.current?.getModel()?.uri.path !== model.uri.path) return;
    setParameterRequest(null);
    applyParameterRequest(parameterRequest);
  }, [parameterRequest, mEditor.activeModel, applyParameterRequest]);

  /** select node definition on event. */
  useCustomEventListener(EVENT_EDITOR_SELECT_RANGE, async (data: TEventEditorSelectRange) => {
    if (data.editorId !== editorId) return;
    const ok = await setEditorModel(data.filePath, data.fileRange, data.launchArgs);
    if (ok && data.selectParameter) {
      selectParameterAppliedRef.current = true;
      setParameterRequest(data.selectParameter);
    }
  });

  useCustomEventListener(
    EVENT_PROVIDER_LAUNCH_LOADED,
    (data: EventProviderLaunchLoaded) => {
      // reload included files to update provided parameters
      if (data.provider.id === provider.id) {
        if (data.launchFile === rootFilePath) {
          loadFiles(mEditor.activeModel ? fileFromUriPath(mEditor.activeModel.uri.path) : currentFilePath);
        }
      }
    },
    [rootFilePath, provider, mEditor.activeModel, currentFilePath]
  );

  /** Handle events caused by changed files. */
  useCustomEventListener(
    EVENT_PROVIDER_PATH_EVENT,
    async (data: EventProviderPathEvent) => {
      if (data.provider.id !== provider.id) {
        // ignore event from other provider
        return;
      }
      const changedUri: string = createUriPath(provider.id, data.path.srcPath);
      if (ownUriPaths.has(changedUri)) {
        // ignore if we saved the file
        if (savedFiles.includes(changedUri)) {
          setSavedFiles(savedFiles.filter((uri) => uri !== changedUri));
          // TODO: reload file content
        }
        if (!editorRef.current) return;
        const currentModelUri = editorRef.current.getModel()?.uri.path;
        const result = await provider.getFileContent(data.path.srcPath);
        if (result.error) {
          console.error(`Could not open file: [${result.file.fileName}]: ${result.error}`);
          setNotificationDescription({
            message: `Could not open file: [${result.file.fileName}]: ${result.error}`,
            messageSeverity: "warning",
          });
        }
        const model = monacoCtx.createModel(editorId, result.file);
        if (!model) {
          console.error(`Could not create model for: [${result.file.fileName}]`);
          setNotificationDescription({
            message: `Could not create model for: [${result.file.fileName}]`,
            messageSeverity: "warning",
          });
          return;
        }
        if (monacoCtx.dirtyManager()?.isDirty(model)) {
          setNotificationDescription({
            message: `${result.file.fileName} was changed on remote host! Save your file or reload manually!`,
            messageSeverity: "warning",
          });
        }
        if (currentModelUri === model.uri.path) {
          mEditor.setCurrentModel(model);
        }
      }
    },
    [provider.id, ownUriPaths, editorId, monacoCtx, mEditor]
  );

  const saveModel = useCallback(
    async (editorModel: editor.ITextModel): Promise<void> => {
      // update saved file to avoid reload question of the current editing file
      if (hasPendingEdit) clearPendingState();
      if (!savedFiles.includes(editorModel.uri.path)) {
        setSavedFiles((prev) => [...prev, editorModel.uri.path]);
      }
      const saveResult = await monacoCtx.saveFile(editorModel);
      if (saveResult.result) {
        // update model state
        mEditor.setCurrentModel(editorModel);
      } else {
        setSavedFiles((prev) => prev.filter((f) => f !== editorModel.uri.path));
        setNotificationDescription({
          message: `Could not save file: ${saveResult.message}`,
          messageSeverity: "warning",
        });
        logCtx.error("Could not save file", saveResult.message, "save failed");
      }
    },
    [savedFiles, rootFilePath, hasPendingEdit, clearPendingState, monacoCtx, mEditor, logCtx]
  );

  const debouncedWidthUpdate = useDebounceCallback((newWidth) => {
    setEditorWidth(newWidth);
  }, 50);

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(panelRef.current.getBoundingClientRect().width - sideBarWidth);
    }
  }, [sideBarWidth]);

  const updatePackageName: (uriPath: string, forcePackageReload?: boolean) => void = useCallback(
    async (uriPath, forcePackageReload = false) => {
      if (forcePackageReload) {
        await provider.getPackageList(false);
      }
      const filePath = fileFromUriPath(uriPath);
      const packageName = provider.getPackageName(filePath);
      if (!packageName && !forcePackageReload) {
        updatePackageName(uriPath, true);
        return;
      }
      setPackageName(packageName || "");
    },
    [provider, mEditor]
  );

  function onKeyDown(event: React.KeyboardEvent): void {
    setKeyboardEvent(event);
  }

  // Most important function:
  //  when the component is mounted, this callback will execute following steps:
  //  - get the content of [currentFilePath]
  //  - create a monaco model (file in editor) based on [currentFilePath]
  //  - check if include files are available (for xml and launch files for example)
  //  -   if available, download all include files and create their corresponding models
  // We download the models per request. On recursive text search all files will be downloaded
  async function loadFiles(filePath: string): Promise<void> {
    if (!editorRef.current) {
      return;
    }
    if (!monacoCtx.monaco) {
      // monaco is not yet available
      setNotificationDescription({ message: "monaco is not yet available", messageSeverity: "error" });
      return;
    }
    if (!currentFilePath || currentFilePath.length === 0) {
      setNotificationDescription({ message: "[currentFilePath] Invalid file path", messageSeverity: "warning" });
      return;
    }
    if (!rootFilePath || rootFilePath.length === 0) {
      setNotificationDescription({ message: "[rootFilePath] Invalid file path", messageSeverity: "warning" });
      return;
    }
    // search host based on selected provider
    if (!provider) {
      setNotificationDescription({
        message: "Provider not available",
        messageSeverity: "warning",
      });
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
    // get file content from provider and create monaco model
    async function getFileAndIncludesAsync(filePath: string): Promise<void> {
      setCurrentFileState({ name: getFileName(filePath), requesting: true, path: filePath });
      const resultFetchIncludes = await includeResolver.fetchIncludedFiles();
      if (!resultFetchIncludes.result) {
        setNotificationDescription({ message: resultFetchIncludes.error, messageSeverity: "warning" });
        return;
      }
      const result: TModelResult = await monacoCtx.getModel(editorId, filePath, false);
      if (!result.model && !result.file) {
        setNotificationDescription({
          message: result.error || `Could not get file: [${filePath}]`,
          messageSeverity: "warning",
        });
        return;
      }
      setCurrentFileState({ name: getFileName(filePath), requesting: false, path: filePath });
      if (!result.model && result.file) {
        console.error(`Could not create model for: [${result.file.fileName}]`);
        setNotificationDescription({
          message: `Could not create model for: [${result.file.fileName}]`,
          messageSeverity: "warning",
        });
        return;
      }
      if (result.model) {
        await setEditorModel(result.model.uri.path, filePath === currentFilePath ? fileRange : null, launchArgs);
      }
      // Ignore "non-launch" files
      if (result.file && !["launch", "xml", "xacro", "py"].includes(result.file.extension)) {
        console.log(`wrong extension: ${result.file.extension} of ${result.file}`);
        includeResolver.clearIncludedFiles();
        setNotificationDescription(undefined);
        return;
      }
    }
    getFileAndIncludesAsync(filePath);
  }

  function handleEditorDidMount(editor: editor.IStandaloneCodeEditor): void {
    editorRef.current = editor;
  }

  useEffect(() => {
    if (monacoInitCtx.initialized && mEditor.initialized) {
      loadFiles(currentFilePath);
    }
  }, [monacoInitCtx.initialized, mEditor.initialized]);

  useEffect(() => {
    if (monacoInitCtx.initialized && monacoInitCtx.monacoCtx.monaco) {
      monacoInitCtx.monacoCtx.monaco.editor.setTheme(isDarkMode ? "vs-ros-dark" : "vs-ros-light");
    }
  }, [monacoInitCtx.initialized, isDarkMode]);

  // report dirty state of the active model to the external editor window
  useEffect(() => {
    const model = mEditor.activeModel;
    if (!model) return;
    window.editorManager?.changed(
      createEditorId(rootFilePath, provider.id),
      fileFromUriPath(model.uri.path),
      mEditor.activeModelDirty
    );
  }, [mEditor.activeModel, mEditor.activeModelDirty, rootFilePath, provider.id]);

  const handleEditorChange = useCallback(
    async (_value: string | undefined, event: editor.IModelContentChangedEvent): Promise<void> => {
      // use the editor model directly - activeModel may still be stale on the first change
      const model = editorRef.current?.getModel();
      if (!model || model.isDisposed()) return;
      cleanUpXmlComment(event.changes, model);
      // refresh dirty state (toolbar, sidebar, external window)
      mEditor.setCurrentModel(model);
    },
    [mEditor]
  );

  const onStateChange = useCallback(
    (collapsed: boolean) => {
      if (collapsed) {
        setSideBarWidth(sideBarMinSize);
      } else if (sideBarWidth <= sideBarMinSize) {
        setSideBarWidth(savedSideBarUserWidth);
      }
    },
    [sideBarMinSize, sideBarWidth, savedSideBarUserWidth]
  );

  return (
    <Stack
      direction="row"
      height="100%"
      width="100%"
      onKeyDown={(event) => onKeyDown(event)}
      onMouseDown={(event) => {
        setEventButton(event);
      }}
      ref={panelRef as ForwardedRef<HTMLDivElement>}
      overflow="auto"
    >
      <SplitPane
        sizes={[sideBarWidth]}
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
            includedFiles={includeResolver.includedFiles}
            selectedFile={{ uriPath: mEditor.activeModel?.uri.path || "", launchArgs: currentLaunchArgs }}
            modifiedUriPaths={mEditor.modifiedFiles}
            sideBarWidth={sideBarWidth}
            keyboardEvent={keyboardEvent}
            panelRef={panelRef}
            onStateChange={onStateChange}
          />
        </Pane>
        <Stack
          sx={{
            flex: 1,
            margin: 0,
          }}
          overflow="none"
        >
          <EditorToolbar
            refEl={toolbarRef as ForwardedRef<HTMLDivElement>}
            providerId={provider.id}
            providerName={providerName}
            packageName={packageName}
            rootFilePath={rootFilePath}
            currentFileState={currentFileState}
            activeModel={mEditor.activeModel}
            activeModelDirty={mEditor.activeModelDirty}
            historyModel={historyModel}
            includedFiles={includeResolver.includedFiles}
            modifiedFiles={mEditor.modifiedFiles}
            eventButton={eventButton}
            setEditorModel={setEditorModel}
            saveModel={saveModel}
            reloadCurrentFile={() => {
              reloadCurrentFile();
            }}
          />
          <PendingEditStyles />
          {/* portal target of the pending edit buttons - must stay mounted */}
          {/* the span wrapper avoids the prop-types warning of mui containers */}
          <span style={{ display: "contents" }}>{pendingEditWidget}</span>
          <AlertsBar
            refEl={alertRef as ForwardedRef<HTMLDivElement>}
            activeModel={mEditor.activeModel}
            message={notificationDescription?.message}
            messageSeverity={notificationDescription?.messageSeverity}
            onClose={() => setNotificationDescription(undefined)}
          />
          <Monaco.Editor
            key="editor"
            height={editorHeight}
            width={editorWidth}
            theme={isDarkMode ? "vs-ros-dark" : "vs-ros-light"}
            onMount={(editor: editor.IStandaloneCodeEditor) => handleEditorDidMount(editor)}
            onChange={(value: string | undefined, ev: editor.IModelContentChangedEvent) =>
              handleEditorChange(value, ev)
            }
            options={{
              // to check the all possible options check this - https://github.com/microsoft/monacoRef.current-editor/blob/a5298e1/website/typedoc/monacoRef.current.d.ts#L3017
              // TODO: make global config for this parameters
              readOnly: mEditor.activeModel ? monacoCtx.isReadOnly(mEditor.activeModel) : false,
              colorDecorators: true,
              mouseWheelZoom: true,
              scrollBeyondLastLine: false,
              smoothScrolling: false,
              wordWrap: "off",
              fontSize: fontSize,
              minimap: { enabled: true },
              selectOnLineNumbers: true,
              guides: {
                bracketPairs: true,
              },
              definitionLinkOpensInPeek: false,
              comments: {
                ignoreEmptyLines: false,
                insertSpace: true,
              },
            }}
          />
        </Stack>
      </SplitPane>
    </Stack>
  );
}
