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
import { useMonacoContext } from "@/renderer/hooks/useMonacoContext";
import { useRosContext } from "@/renderer/hooks/useRosContext";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { getFileName } from "@/renderer/models";
import { cleanUpXmlComment } from "@/renderer/monaco/setup";
import { TModelResult } from "@/renderer/monaco/types";
import { createEditorEditorId, createUriPath, fileFromUriPath } from "@/renderer/monaco/utils";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  TEventEditorSelectRange,
  eventCloseComponent,
} from "@/renderer/pages/NodeManager/layout/events";
import { EventProviderPathEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_PATH_EVENT } from "@/renderer/providers/eventTypes";
import { TFileRange, TLaunchArg } from "@/types";
import "./FileEditorPanel.css";

type TAlertNotification = {
  message?: string;
  messageSeverity?: "success" | "info" | "warning" | "error";
};

interface FileEditorPanelProps {
  editorId: string;
  providerId: string;
  rootFilePath: string;
  currentFilePath: string;
  fileRange: TFileRange | null;
  launchArgs: TLaunchArg[];
}

export default function FileEditorPanel(props: FileEditorPanelProps): JSX.Element {
  const { editorId, providerId, rootFilePath, currentFilePath, fileRange, launchArgs } = props;
  const settingsCtx = useSettingsContext();
  const logCtx = useLoggingContext();
  const rosCtx = useRosContext();
  const monacoCtx = useMonacoContext();

  const editorRef = useRef<editor.IStandaloneCodeEditor>();

  const [providerName, setProviderName] = useState<string>("");
  const [packageName, setPackageName] = useState<string>("");
  const [currentFileState, setCurrentFileState] = useState({ name: "", requesting: false, path: "" });
  const [ownUriToPackageDict] = useState({});

  const [selectionRange, setSelectionRange] = useState<TFileRange>();
  const [currentLaunchArgs, setCurrentLaunchArgs] = useState<TLaunchArg[]>(launchArgs);
  const [notificationDescription, setNotificationDescription] = useState<TAlertNotification | undefined>();
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

  const { includedFiles, fetchIncludedFiles, clearIncludedFiles } = useIncludedFiles(providerId, rootFilePath);
  const mEditor = useMonacoEditor({
    editorId: editorId,
    editorRef: editorRef,
    includedFiles: includedFiles,
    saveModel: (model) => {
      saveModel(model);
    },
  });

  const ownUriPaths: Set<string> = useMemo(() => {
    const result = new Set([rootFilePath, ...(includedFiles?.map((f) => createUriPath(providerId, f.inc_path)) || [])]);
    return result;
  }, [providerId, rootFilePath, includedFiles]);

  useEditorKeyboard(() => {
    const provider = rosCtx.getProviderById(providerId);
    if (provider) {
      const id = createEditorEditorId(rootFilePath, provider.id);
      emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
    }
  });

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
  }, [settingsCtx.changed]);

  useEffect(() => {
    return (): void => {
      editorRef.current?.setModel(null);
      mEditor.dispose();
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
      const modelPackageName = ownUriToPackageDict[result.model.uri.path];
      setPackageName(modelPackageName || "");

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
    [mEditor, providerId, ownUriToPackageDict, monacoCtx.getModel]
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

  /** Handle events caused by changed files. */
  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, async (data: EventProviderPathEvent) => {
    if (data.provider.id !== providerId) {
      // ignore event from other provider
      return;
    }
    const changedUri: string = createUriPath(providerId, data.path.srcPath);
    if (ownUriPaths.has(changedUri)) {
      // ignore if we saved the file
      if (savedFiles.includes(changedUri)) {
        setSavedFiles(savedFiles.filter((uri) => uri !== changedUri));
        // TODO: reload file content
        return;
      }
      if (!editorRef.current) return;

      const provider = rosCtx.getProviderById(providerId, true);
      if (provider) {
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
        const providerObj = rosCtx.getProviderById(providerId, true);
        if (providerObj) {
          const path = fileFromUriPath(editorModel.uri.path);
          const id = createEditorEditorId(rootFilePath, providerObj.id);
          window.editorManager?.changed(id, path, false);
        }
        mEditor.setCurrentModel(editorModel);
        const resultFetchIncludes = await fetchIncludedFiles();
        if (!resultFetchIncludes.result) {
          setNotificationDescription({ message: resultFetchIncludes.error, messageSeverity: "warning" });
        } else {
          setNotificationDescription(undefined);
        }
      } else {
        setSavedFiles((prev) => prev.filter((f) => f === editorModel.uri.path));
        setNotificationDescription({
          message: `Could not save file: ${saveResult.message}`,
          messageSeverity: "warning",
        });
        logCtx.error("Could not save file", saveResult.message, "save failed");
      }
    },
    [providerId, savedFiles, rootFilePath, rosCtx.getProviderById]
  );

  const debouncedWidthUpdate = useDebounceCallback((newWidth) => {
    setEditorWidth(newWidth);
  }, 50);

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(panelRef.current.getBoundingClientRect().width - sideBarWidth);
    }
  }, [sideBarWidth]);

  useEffect(() => {
    // update package names for own files
    const provider = rosCtx.getProviderById(providerId, true);
    if (!provider) return;

    const uriPath = createUriPath(providerId, rootFilePath);
    ownUriToPackageDict[uriPath] = provider.getPackageName(rootFilePath);
    for (const file of includedFiles) {
      const incUriPath = createUriPath(providerId, file.inc_path);
      ownUriToPackageDict[incUriPath] = provider.getPackageName(file.inc_path);
    }
  }, [includedFiles, rootFilePath, editorRef.current]);

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
  async function loadFiles(): Promise<void> {
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
    const provider = rosCtx.getProviderById(providerId);
    if (!provider) {
      setNotificationDescription({
        message: `Provider with id ${providerId} not available`,
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
    async function getFileAndIncludesAsync(): Promise<void> {
      setCurrentFileState({ name: getFileName(currentFilePath), requesting: true, path: currentFilePath });
      const resultFetchIncludes = await fetchIncludedFiles();
      if (!resultFetchIncludes.result) {
        setNotificationDescription({ message: resultFetchIncludes.error, messageSeverity: "warning" });
        return;
      }
      const result: TModelResult = await monacoCtx.getModel(editorId, currentFilePath, false);
      if (!result.model && !result.file) {
        setNotificationDescription({
          message: result.error || `Could not get file: [${currentFilePath}]`,
          messageSeverity: "warning",
        });
        return;
      }
      setCurrentFileState({ name: getFileName(currentFilePath), requesting: false, path: currentFilePath });
      if (!result.model && result.file) {
        console.error(`Could not create model for: [${result.file.fileName}]`);
        setNotificationDescription({
          message: `Could not create model for: [${result.file.fileName}]`,
          messageSeverity: "warning",
        });
        return;
      }
      if (result.model) {
        setEditorModel(result.model.uri.path, fileRange, launchArgs);
      }
      // Ignore "non-launch" files
      // TODO: Add parameter Here
      if (result.file && !["launch", "xml", "xacro", "py"].includes(result.file.extension)) {
        console.log(`wrong extension: ${result.file.extension} of ${result.file}`);
        clearIncludedFiles();
        setNotificationDescription(undefined);
        return;
      }
    }
    getFileAndIncludesAsync();
  }

  // initialization of provider definitions
  useEffect(() => {
    if (!editorRef.current || !monacoCtx.monaco) return;
    mEditor.setupMonacoEditor();
    loadFiles();
  }, [editorRef.current]);

  function handleEditorDidMount(editor: editor.IStandaloneCodeEditor): void {
    editorRef.current = editor;
    mEditor.setupContextMenu();
  }

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
            providerId={providerId}
            rootFilePath={rootFilePath}
            includedFiles={includedFiles}
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
            providerName={providerName}
            packageName={packageName}
            rootFilePath={rootFilePath}
            currentFileState={currentFileState}
            activeModel={mEditor.activeModel}
            activeModelDirty={mEditor.activeModelDirty}
            historyModel={historyModel}
            includedFiles={includedFiles}
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
