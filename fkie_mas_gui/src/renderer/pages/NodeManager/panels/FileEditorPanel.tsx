import { useMonacoEditor } from "@/renderer/hooks/editor/useMonacoEditor";
import * as Monaco from "@monaco-editor/react";
import { Stack } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { editor } from "monaco-editor";
import { ForwardedRef, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import SplitPane, { Pane, SashContent } from "split-pane-react";
import "split-pane-react/esm/themes/default.css";

import { AlertsBar, EditorSidebar, EditorToolbar, THistoryModel } from "@/renderer/components/FileEditorPanel";
import { useEditorKeyboard } from "@/renderer/hooks/editor/useEditorKeyboard";
import { useEditorLayout } from "@/renderer/hooks/editor/useEditorLayout";
import { useIncludedFiles } from "@/renderer/hooks/useIncludedFiles";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { useMonacoInitContext } from "@/renderer/hooks/useMonacoInitContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { getFileName } from "@/renderer/models";
import { cleanUpXmlComment } from "@/renderer/monaco/setup";
import { TModelResult } from "@/renderer/monaco/types";
import { createEditorId, createUriPath, fileFromUriPath } from "@/renderer/monaco/utils";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  TEventEditorSelectRange,
  eventCloseComponent,
} from "@/renderer/pages/NodeManager/layout/events";
import { Provider } from "@/renderer/providers";
import { EventProviderLaunchLoaded, EventProviderPathEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_LAUNCH_LOADED, EVENT_PROVIDER_PATH_EVENT } from "@/renderer/providers/eventTypes";
import { TFileRange, TLaunchArg } from "@/types";
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
}

export default function FileEditorPanel(props: FileEditorPanelProps): JSX.Element {
  const { editorId, provider, rootFilePath, currentFilePath, fileRange, launchArgs } = props;
  const settingsCtx = useSettingsContext();
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
  const [isDarkMode, setIsDarkMode] = useState<boolean>(settingsCtx.get("useDarkMode") as boolean);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [historyModel, setHistoryModel] = useState<THistoryModel | undefined>();
  const [eventButton, setEventButton] = useState<React.MouseEvent<HTMLDivElement, MouseEvent> | undefined>(undefined);
  const [keyboardEvent, setKeyboardEvent] = useState<React.KeyboardEvent | undefined>();
  const [savedFiles, setSavedFiles] = useState<string[]>([]);

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

  const includeResolver = useIncludedFiles(provider, rootFilePath, launchArgs);
  monacoCtx.setResolver(editorId, includeResolver);

  const mEditor = useMonacoEditor({
    editorId: editorId,
    editorRef: editorRef,
    saveModel: (model) => {
      saveModel(model);
    },
  });

  const ownUriPaths: Set<string> = useMemo(() => {
    const result = new Set([
      rootFilePath,
      ...(includeResolver.includedFiles?.map((f) => createUriPath(provider.id, f.inc_path)) || []),
    ]);
    return result;
  }, [provider, rootFilePath, includeResolver.includedFiles]);

  useEditorKeyboard(() => {
    const id = createEditorId(rootFilePath, provider.id);
    emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
  });

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setIsDarkMode(settingsCtx.get("useDarkMode") as boolean);
  }, [settingsCtx.changed]);

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
      setNotificationDescription({ message: "Getting file from provider...", messageSeverity: "info" });
      // If model does not exist, try to fetch it
      const result: TModelResult = await monacoCtx.getModel(editorId, uriPath, forceReload);
      setNotificationDescription(undefined);
      setCurrentFileState({ name: getFileName(uriPath), requesting: false, path: uriPath });

      // get model from path if exists
      if (!result.model) {
        logCtx.error(`Could not get model for file: ${uriPath}`, "");
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

      return true;
    },
    [mEditor, monacoCtx.getModel]
  );

  const reloadCurrentFile = useCallback(async () => {
    if (mEditor.activeModel?.uri.path) {
      const path = mEditor.activeModel?.uri.path;
      const result = await setEditorModel(path, selectionRange, currentLaunchArgs, true, false);
      if (result) {
        logCtx.success(`File reloaded [${getFileName(path)}]`, "", `${getFileName(path)} reloaded`);
      }
    }
  }, [mEditor, selectionRange, currentLaunchArgs, logCtx, setEditorModel]);

  /** select node definition on event. */
  useCustomEventListener(EVENT_EDITOR_SELECT_RANGE, async (data: TEventEditorSelectRange) => {
    if (data.editorId === editorId) {
      setEditorModel(data.filePath, data.fileRange, data.launchArgs);
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
  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, async (data: EventProviderPathEvent) => {
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
        return;
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
        return;
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
        return;
      }
      if (currentModelUri === model.uri.path) {
        mEditor.setCurrentModel(model);
      }
    }
  });

  const saveModel = useCallback(
    async (editorModel: editor.ITextModel): Promise<void> => {
      // update saved file to avoid reload question of the current editing file
      if (!savedFiles.includes(editorModel.uri.path)) {
        setSavedFiles((prev) => [...prev, editorModel.uri.path]);
      }
      const saveResult = await monacoCtx.saveFile(editorModel);
      if (saveResult.result) {
        // update state of external window
        const path = fileFromUriPath(editorModel.uri.path);
        const id = createEditorId(rootFilePath, provider.id);
        window.editorManager?.changed(id, path, false);
        // update model state
        mEditor.setCurrentModel(editorModel);
      } else {
        setSavedFiles((prev) => prev.filter((f) => f === editorModel.uri.path));
        setNotificationDescription({
          message: `Could not save file: ${saveResult.message}`,
          messageSeverity: "warning",
        });
        logCtx.error("Could not save file", saveResult.message, "save failed");
      }
    },
    [savedFiles, rootFilePath]
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
    if (provider && !provider.host()) {
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
        setEditorModel(result.model.uri.path, filePath === currentFilePath ? fileRange : null, launchArgs);
      }
      // Ignore "non-launch" files
      // TODO: Add parameter Here
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

  const handleEditorChange = useCallback(
    async (_value: string | undefined, event: editor.IModelContentChangedEvent): Promise<void> => {
      if (mEditor.activeModel) {
        cleanUpXmlComment(event.changes, mEditor.activeModel);
      }
    },
    [mEditor.activeModel]
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
        // defaultSize={sideBarWidth}
        sizes={[sideBarWidth]}
        onChange={([size]) => {
          if (size !== sideBarMinSize && size >= sideBarMinSize) {
            setSavedSideBarUserWidth(size);
          }
          setSideBarWidth(size);
        }}
        split="vertical"
        resizerSize={6}
        sashRender={(_index, active) => (
          <SashContent className={`sash-wrap-line ${active ? "active" : "inactive"}`}>
            {/* <span className="line"/> */}
          </SashContent>
        )}
      >
        <Pane minSize={sideBarMinSize} style={{ backgroundColor: backgroundColor }}>
          <EditorSidebar
            editorId={editorId}
            provider={provider}
            rootFilePath={rootFilePath}
            includedFiles={includeResolver.includedFiles}
            selectedUriPath={mEditor.activeModel?.uri.path ? mEditor.activeModel?.uri.path : ""}
            launchArgs={currentLaunchArgs}
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
          <AlertsBar
            refEl={alertRef as ForwardedRef<HTMLDivElement>}
            message={notificationDescription?.message}
            messageSeverity={notificationDescription?.messageSeverity}
            activeModel={mEditor.activeModel}
            onClose={() => setNotificationDescription(undefined)}
          />
          <Monaco.Editor
            key="editor"
            height={editorHeight}
            width={editorWidth}
            theme={settingsCtx.get("useDarkMode") ? "vs-ros-dark" : "vs-ros-light"}
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
              // automaticLayout: true,
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
