import * as Monaco from "@monaco-editor/react";
import CloudSyncOutlinedIcon from "@mui/icons-material/CloudSyncOutlined";
import FolderCopyOutlinedIcon from "@mui/icons-material/FolderCopyOutlined";
import SaveAltOutlinedIcon from "@mui/icons-material/SaveAltOutlined";
import SearchOutlinedIcon from "@mui/icons-material/SearchOutlined";
import UpgradeIcon from "@mui/icons-material/Upgrade";
import { Alert, CircularProgress, IconButton, Link, Stack, ToggleButton, Tooltip, Typography } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import { CancellationToken, IDisposable, Uri, editor, languages } from "monaco-editor/esm/vs/editor/editor.api";
import { ForwardedRef, useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import SplitPane, { Pane, SashContent } from "split-pane-react";
import "split-pane-react/esm/themes/default.css";

import ExplorerTree, { equalLaunchArgs } from "@/renderer/components/MonacoEditor/ExplorerTree";
import { createPythonLaunchProposals } from "@/renderer/components/MonacoEditor/PythonLaunchProposals";
import SearchTree from "@/renderer/components/MonacoEditor/SearchTree";
import XmlBeautify from "@/renderer/components/MonacoEditor/XmlBeautify";
import { Ros2XmlLanguage } from "@/renderer/components/MonacoEditor/XmlLaunchHighlighterR2";
import {
  createDocumentSymbols,
  createXMLDependencyProposals,
} from "@/renderer/components/MonacoEditor/XmlLaunchProposals";
import {
  createDocumentSymbolsR2,
  createXMLDependencyProposalsR2,
} from "@/renderer/components/MonacoEditor/XmlLaunchProposalsR2";
import { OverflowMenu } from "@/renderer/components/UI";
import { colorFromHostname } from "@/renderer/components/UI/Colors";
import SearchBar from "@/renderer/components/UI/SearchBar";
import { LoggingContext } from "@/renderer/context/LoggingContext";
import { MonacoContext, TModelResult } from "@/renderer/context/MonacoContext";
import RosContext from "@/renderer/context/RosContext";
import { SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import {
  FileItem,
  LaunchIncludedFile,
  LaunchIncludedFilesRequest,
  RosPackage,
  getFileAbb,
  getFileName,
} from "@/renderer/models";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  TEventEditorSelectRange,
  eventCloseComponent,
  eventEditorSelectRange,
} from "@/renderer/pages/NodeManager/layout/events";
import { Provider } from "@/renderer/providers";
import { EventProviderPathEvent } from "@/renderer/providers/events";
import { EVENT_PROVIDER_PATH_EVENT } from "@/renderer/providers/eventTypes";
import { TFileRange, TLaunchArg } from "@/types";
import "./FileEditorPanel.css";
import { Ros1XmlLanguage } from "@/renderer/components/MonacoEditor/XmlLaunchHighlighter";

type TActiveModel = {
  path: string;
  modified: boolean;
  model: editor.ITextModel;
};

type TModelVersion = {
  path: string;
  version: number;
};

type TFileItem = {
  path: string;
  file: FileItem;
};

interface FileEditorPanelProps {
  tabId: string;
  providerId: string;
  rootFilePath: string;
  currentFilePath: string;
  fileRange: TFileRange | null;
  launchArgs: TLaunchArg[];
}

export class IncludesProvider implements languages.LinkProvider {
  public modelLinks: { [id: string]: languages.ILink[] } = {};

  public provideLinks: (
    model: editor.ITextModel,
    token: CancellationToken
  ) => languages.ProviderResult<languages.ILinksList> = (model) => {
    return { links: this.modelLinks[model.uri.path] };
  };
  // public resolveLink: (link: languages.ILink, token: CancellationToken) => languages.ProviderResult<languages.ILink> = (
  //   link,
  //   token
  // ) => {
  //   return link;
  // };
}

export default function FileEditorPanel(props: FileEditorPanelProps): JSX.Element {
  const { tabId, providerId, rootFilePath, currentFilePath, fileRange, launchArgs } = props;
  const monaco = Monaco.useMonaco();
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const monacoCtx = useContext(MonacoContext);

  // ----- size handling
  const editorRef = useRef<editor.IStandaloneCodeEditor>();
  const panelRef = useRef<HTMLDivElement>();
  const infoRef = useRef<HTMLDivElement>();
  const alertRef = useRef<HTMLDivElement>();
  const resizeObserver = useRef<ResizeObserver>();
  const componentWillUnmount = useRef(false);
  const [fontSize, setFontSize] = useState<number>(settingsCtx.get("fontSize") as number);
  const [savedSideBarUserWidth, setSavedSideBarUserWidth] = useLocalStorage<number>(
    "Editor:sideBarWidth",
    fontSize * 20
  );
  const [savedExplorerBarHight, setSavedExplorerBarHight] = useLocalStorage<number>(
    "Editor:explorerBarHight",
    fontSize * 20
  );
  const [sideBarMinSize, setSideBarMinSize] = useState<number>(fontSize * 2);
  const [sideBarWidth, setSideBarWidth] = useState<number>(fontSize * 2);
  const [explorerBarMinSize, setExplorerBarMinSize] = useState<number>(fontSize * 2);
  const [explorerBarHeight, setExplorerBarHeight] = useState<number>(fontSize * 2);
  const [editorHeight, setEditorHeight] = useState<number>(20);
  const [editorWidth, setEditorWidth] = useState<number>(20);
  const [panelHeight, setPanelHeight] = useState<number>(0);
  const [panelSize, setPanelSize] = useState<DOMRect>();
  // ----- size handling end

  const [providerName, setProviderName] = useState<string>("");
  const [providerHost, setProviderHost] = useState<string>("");
  const [packageName, setPackageName] = useState<string>("");
  const [initialized, setInitialized] = useState<boolean>(false);
  const [activeModel, setActiveModel] = useState<TActiveModel>();
  const [currentFile, setCurrentFile] = useState({ name: "", requesting: false });
  const [openFiles, setOpenFiles] = useState<TFileItem[]>([]);
  const [ownUriPaths, setOwnUriPaths] = useState<string[]>([]);
  const [ownUriToPackageDict] = useState({});
  const [monacoDisposables, setMonacoDisposables] = useState<IDisposable[]>([]);
  const [monacoViewStates] = useState(new Map());

  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [enableExplorer, setEnableExplorer] = useState(false);
  const [globalSearchTerm, setGlobalSearchTerm] = useState("");

  const [selectionRange, setSelectionRange] = useState<TFileRange>();
  const [currentLaunchArgs, setCurrentLaunchArgs] = useState<TLaunchArg[]>(launchArgs);
  const [installPathsWarn, setInstallPathsWarn] = useState<string[]>([]);
  const [modifiedFiles, setModifiedFiles] = useState<string[]>([]);
  const [savedModelVersions, setSavedModelVersions] = useState<TModelVersion[]>([]);
  const [includedFiles, setIncludedFiles] = useState<LaunchIncludedFile[]>([]);
  const [notificationDescription, setNotificationDescription] = useState("");
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [selectParentFiles, setSelectParentFiles] = useState<LaunchIncludedFile[]>([]);

  const [includesProvider] = useState<IncludesProvider>(new IncludesProvider());

  useEffect(() => {
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx.changed]);

  const [savedFiles, setSavedFiles] = useState<string[]>([]);

  let escapePressCount = 0;

  useEffect(() => {
    if (selectionRange) {
      if (editorRef.current) {
        editorRef.current.revealRangeInCenter({
          startLineNumber: selectionRange.startLineNumber,
          endLineNumber: selectionRange.endLineNumber,
          startColumn: selectionRange.startColumn,
          endColumn: selectionRange.endColumn,
        });

        editorRef.current.setPosition({
          lineNumber: selectionRange.startLineNumber,
          column: selectionRange.startColumn,
        });

        let endLineNumber = selectionRange.endLineNumber;
        if (
          selectionRange.startColumn === selectionRange.endColumn &&
          selectionRange.startLineNumber === selectionRange.endLineNumber
        ) {
          endLineNumber += 1;
        }
        editorRef.current.setSelection({
          startLineNumber: selectionRange.startLineNumber,
          endLineNumber: endLineNumber,
          startColumn: selectionRange.startColumn,
          endColumn: selectionRange.endColumn,
        });

        editorRef.current.focus();
      }
    }
  }, [selectionRange]);

  const handleChangeExplorer = useCallback(
    function (isExpanded: boolean): void {
      setEnableExplorer(isExpanded);
    },
    [setEnableExplorer]
  );

  const handleChangeSearch = useCallback(
    function (isExpanded: boolean): void {
      setEnableGlobalSearch(isExpanded);
    },
    [setEnableGlobalSearch]
  );

  const isReadOnly = useCallback(
    function (path: string): boolean {
      const files = openFiles.filter((item) => item.path === path);
      if (files.length > 0) {
        return files[0].file.readonly;
      }
      return false;
    },
    [openFiles]
  );

  const addMonacoDisposable = useCallback(
    function (disposable: IDisposable): void {
      setMonacoDisposables((prev) => [...prev, disposable]);
    },
    [setMonacoDisposables]
  );

  // update modified files in this panel and context
  function updateModifiedFiles(): void {
    if (!monaco) return;
    const newModifiedFiles = monaco.editor
      .getModels()
      .filter((model: editor.ITextModel) => {
        const result = isModified(model) && ownUriPaths.includes(model.uri.path);
        return result;
      })
      .map((model) => {
        return model.uri.path;
      });
    setModifiedFiles(newModifiedFiles);
    monacoCtx.updateModifiedFiles(tabId, providerId, newModifiedFiles);
  }

  // update decorations for included files
  const updateIncludeDecorations = useCallback(function (
    model: editor.ITextModel | null,
    includedFilesList: LaunchIncludedFile[]
  ): void {
    if (!model) return;
    // filter files
    const newLinks: languages.ILink[] = [];
    if (includedFilesList) {
      includedFilesList.forEach((f: LaunchIncludedFile) => {
        const path = model.uri.path.split(":")[1];
        if (path === f.path && path !== f.inc_path) {
          const matches = model.findMatches(f.raw_inc_path, false, false, false, null, true);
          if (matches.length > 0) {
            matches.forEach((match) => {
              newLinks.push({
                range: match.range,
                url: `${model.uri.path.split(":")[0]}:${f.inc_path}`,
              });
            });
          }
        }
      });
    }
    includesProvider.modelLinks[model.uri.path] = newLinks;
  }, []);

  const isModified = useCallback(
    function (model: editor.ITextModel): boolean {
      const item: TModelVersion | undefined = savedModelVersions.find((item) => item.path === model.uri.path);
      if (item) {
        return item.version !== model.getAlternativeVersionId();
      }
      return model.getAlternativeVersionId() > 1;
    },
    [savedModelVersions]
  );

  function updateOpenFiles(result: TModelResult): void {
    if (result && result.file !== null && result.model && result.model.uri.path) {
      const newFileItem: TFileItem = { path: result.model.uri.path, file: result.file };
      setOpenFiles((prev) => [...prev.filter((item) => item.path !== result.model?.uri.path), newFileItem]);
    }
  }

  useEffect(() => {
    // update parent files
    const pathSplits = activeModel?.model.uri.path.split(":");
    if (pathSplits && pathSplits.length > 1) {
      const path = pathSplits[1];
      const parentPaths = includedFiles.filter((item) => {
        if (path === item.inc_path) {
          // check args to select the correct file, if the same file included twice
          return equalLaunchArgs(currentLaunchArgs, item.args || []);
        }
        return false;
      });
      setSelectParentFiles(parentPaths);
    }
  }, [activeModel, includedFiles, currentLaunchArgs]);

  // set the current model to the editor based on [uriPath], and update its decorations
  const setEditorModel = useCallback(
    async function (
      uriPath: string,
      range: TFileRange | null = null,
      launchArgs: TLaunchArg[] = [],
      forceReload: boolean = false
    ): Promise<boolean> {
      if (!uriPath) return false;
      setCurrentFile({ name: getFileName(uriPath), requesting: true });
      setNotificationDescription("Getting file from provider...");
      // If model does not exist, try to fetch it
      const result: TModelResult = await monacoCtx.getModel(tabId, providerId, uriPath, forceReload);
      setCurrentFile({ name: getFileName(uriPath), requesting: false });
      setNotificationDescription("");

      // get model from path if exists
      if (!result.model) {
        logCtx.error(`Could not get model for file: ${uriPath}`, "");
        return false;
      }

      if (result.file && result.model) {
        updateInstallPathsWarn(result.file, result.model.uri.path);
      }
      updateOpenFiles(result);
      // save current view state, in case user wants to open the file again
      // view state contains the cursor position, folding, selections etc...
      if (editorRef.current) {
        const currentModel: editor.ITextModel | null | undefined = editorRef.current.getModel();
        if (currentModel) {
          monacoViewStates.set(currentModel.uri.path, editorRef.current.saveViewState());
        }

        editorRef.current.setModel(result.model);
        // restore view state for current file
        const viewState = monacoViewStates.get(result.model.uri.path);
        if (viewState) {
          editorRef.current.restoreViewState(viewState);
        }
      }
      updateIncludeDecorations(result.model, includedFiles);
      setActiveModel({
        path: result.model.uri.path,
        modified: isModified(result.model),
        model: result.model,
      });
      // update modified files for the user info in the info bar
      updateModifiedFiles();

      // set package name
      const modelPackageName = ownUriToPackageDict[result.model.uri.path];
      setPackageName(modelPackageName || "");
      // set range is available
      if (range) {
        setSelectionRange(range);
      }
      setCurrentLaunchArgs(launchArgs);
      return true;
    },
    [
      monaco,
      monacoViewStates,
      includedFiles,
      providerId,
      providerHost,
      monacoCtx.getModel,
      setActiveModel,
      updateModifiedFiles,
    ]
  );

  /** select node definition on event. */
  useCustomEventListener(EVENT_EDITOR_SELECT_RANGE, async (data: TEventEditorSelectRange) => {
    if (data.tabId === tabId) {
      setEditorModel(data.filePath, data.fileRange, data.launchArgs);
    }
  });

  function updateInstallPathsWarn(file: FileItem, modelUriPath: string): void {
    const filePath = file.realpath && file.realpath.length > 0 ? file.realpath : file.path;
    // check if the file is located in install folder
    if (filePath.search("/install/") !== -1) {
      setInstallPathsWarn((prev) => [modelUriPath, ...prev.filter((item) => item !== modelUriPath)]);
    }
  }

  /** Handle events caused by changed files. */
  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, async (data: EventProviderPathEvent) => {
    if (data.provider.id !== providerId) {
      // ignore event from other provider
      return;
    }
    const changedUri: string = monacoCtx.createUriPath(tabId, data.path.srcPath);
    if (ownUriPaths.includes(changedUri)) {
      // ignore if we saved the file
      if (savedFiles.includes(changedUri)) {
        setSavedFiles(savedFiles.filter((uri) => uri !== changedUri));
        // TODO: reload file content
        return;
      }
      if (!editorRef.current) return;
      const provider = rosCtx.getProviderById(providerId, true);
      if (provider) {
        const currentModel = editorRef.current.getModel();
        const result = await provider.getFileContent(data.path.srcPath);
        if (result.error) {
          console.error(`Could not open file: [${result.file.fileName}]: ${result.error}`);
          setNotificationDescription(`Could not open file: [${result.file.fileName}]: ${result.error}`);
          return;
        }
        if (currentModel) {
          monacoViewStates.set(currentModel.uri.path, editorRef.current.saveViewState());
        }
        const model = monacoCtx.createModel(tabId, result.file);
        if (!model) {
          console.error(`Could not create model for: [${result.file.fileName}]`);
          setNotificationDescription(`Could not create model for: [${result.file.fileName}]`);
          return;
        }
        updateInstallPathsWarn(result.file, model.uri.path);
        if (currentModel && currentModel.uri.path === model.uri.path) {
          updateOpenFiles({ model: model, file: result.file } as TModelResult);
          editorRef.current.setModel(model);
          // restore view state for current file
          const viewState = monacoViewStates.get(model.uri.path);
          if (viewState) {
            editorRef.current.restoreViewState(viewState);
          }
        }
      }
    }
  });

  useEffect(() => {
    updateModifiedFiles();
  }, [savedModelVersions]);

  async function saveCurrentFile(editorModel: editor.ITextModel): Promise<void> {
    const path = editorModel.uri.path.split(":")[1];
    // TODO change encoding if the file is encoded as HEX
    const fileToSave = new FileItem("", path, "", "", false, editorModel.getValue());
    const providerObj = rosCtx.getProviderById(providerId, true);
    if (providerObj) {
      const saveResult = await providerObj.saveFileContent(fileToSave);
      if (saveResult.bytesWritten > 0) {
        const id = `editor-${providerObj.connection.host}-${providerObj.connection.port}-${rootFilePath}`;
        window.editorManager?.changed(id, path, false);
        if (!savedFiles.includes(editorModel.uri.path)) {
          setSavedFiles([...savedFiles, editorModel.uri.path]);
        }
        logCtx.success(`Successfully saved file`, `path: ${path}`);
        setSavedModelVersions((prev) => [
          ...prev.filter((item) => {
            return item.path !== editorModel.uri.path;
          }),
          { path: editorModel.uri.path, version: editorModel.getAlternativeVersionId() } as TModelVersion,
        ]);
        setActiveModel({ path: editorModel.uri.path, modified: false, model: editorModel });
      } else {
        logCtx.error(`Error while save file ${path}`, `${saveResult.error}`);
      }
    } else {
      logCtx.error(`Provider ${providerId} not found`, `can not save file: ${path}`);
    }
  }

  const debouncedWidthUpdate = useDebounceCallback((newWidth) => {
    setEditorWidth(newWidth);
  }, 50);

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(panelRef.current.getBoundingClientRect().width - sideBarWidth);
    }
  }, [debouncedWidthUpdate, sideBarWidth]);

  function cleanUpXmlComment(changes, model): void {
    // replace all '--' by '- - ' in XML comments
    if (!model) return;
    if (changes.length != 2) return;
    let addedComment = false;
    if (
      changes.filter((entry) => {
        addedComment = entry.text.length > 0;
        return !["", "<!-- ", " -->"].includes(entry.text);
      }).length > 0
    )
      return;
    // get range
    const range = {
      endColumn: changes[0].range.startColumn + (addedComment ? 2 : 0),
      endLineNumber: changes[0].range.startLineNumber,
      startColumn: changes[1].range.endColumn + (addedComment ? 4 : 0),
      startLineNumber: changes[1].range.endLineNumber,
    };
    const matches = model.findMatches(addedComment ? "--" : "- - ", range);
    matches.reverse().map((match) => {
      model.pushEditOperations(null, [
        { forceMoveMarkers: false, range: match.range, text: addedComment ? "- - " : "--" },
      ]);
    }, true);
  }

  const handleEditorChange = useCallback(
    async function (_value: string | undefined, event: editor.IModelContentChangedEvent): Promise<void> {
      // update activeModel modified flag only once
      cleanUpXmlComment(event.changes, activeModel?.model);
      if (activeModel) {
        if (!activeModel.modified) {
          setCurrentFile({ name: getFileName(activeModel.path), requesting: true });
          setNotificationDescription("Getting file from provider...");
          const result: TModelResult = await monacoCtx.getModel(tabId, providerId, activeModel.path, false);
          setCurrentFile({ name: getFileName(activeModel.path), requesting: false });
          setNotificationDescription("");
          if (result.model) {
            setActiveModel({ path: result.model.uri.path, modified: true, model: result.model });
          }
          if (result.file && result.model) {
            updateInstallPathsWarn(result.file, result.model.uri.path);
          }
          updateModifiedFiles();
          const provider = rosCtx.getProviderById(providerId, true);
          if (provider) {
            const id = `editor-${provider.connection.host}-${provider.connection.port}-${rootFilePath}`;
            window.editorManager?.changed(id, activeModel.path, true);
          }
        }
      }
    },
    [activeModel, setActiveModel, monacoCtx.getModel, updateModifiedFiles]
  );

  useEffect(() => {
    // update height and width of the split panel on the left side
    if (!(enableExplorer || enableGlobalSearch)) {
      setSideBarWidth(sideBarMinSize);
    } else if (sideBarWidth <= sideBarMinSize) {
      setSideBarWidth(savedSideBarUserWidth);
    }
    if (enableExplorer) {
      if (enableGlobalSearch) {
        setExplorerBarHeight(savedExplorerBarHight);
      } else {
        setExplorerBarHeight(panelHeight - explorerBarMinSize);
      }
    } else {
      setExplorerBarHeight(explorerBarMinSize);
    }
  }, [enableExplorer, enableGlobalSearch, panelHeight, setSideBarWidth, setExplorerBarHeight]);

  useEffect(() => {
    const newFontSize = settingsCtx.get("fontSize") as number;
    setFontSize(newFontSize);
    setSideBarMinSize(newFontSize * 2 + 2);
    setSideBarWidth(newFontSize * 2 + 2);
    setExplorerBarMinSize(newFontSize * 2 + 2);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // update ownUriPaths and package names (ownUriToPackageDict)
    const provider = rosCtx.getProviderById(providerId, true);
    if (!provider?.host()) return;
    if (editorRef.current) {
      updateIncludeDecorations(editorRef.current.getModel(), includedFiles);
    }
    const uriPath = monacoCtx.createUriPath(tabId, rootFilePath);
    const newOwnUris = [uriPath];
    ownUriToPackageDict[uriPath] = provider.getPackageName(rootFilePath);
    includedFiles.forEach((file) => {
      const incUriPath = monacoCtx.createUriPath(tabId, file.inc_path);
      newOwnUris.push(incUriPath);
      ownUriToPackageDict[incUriPath] = provider.getPackageName(file.inc_path);
    });
    setOwnUriPaths(newOwnUris);
  }, [includedFiles, rootFilePath, editorRef.current]);

  useEffect(() => {
    if (!panelSize) return;
    const infoHeight: number = infoRef.current ? infoRef.current?.getBoundingClientRect().height : 0;
    const alertHeight: number = alertRef.current ? alertRef.current?.getBoundingClientRect().height : 0;
    setEditorHeight(panelSize.height - infoHeight - alertHeight);
    setEditorWidth(panelSize.width - sideBarWidth);
    setPanelHeight(panelSize.height);
  }, [sideBarWidth, panelSize, activeModel]);

  useEffect(() => {
    // add resize observer to update size of monaco editor ad side bars
    if (panelRef.current) {
      resizeObserver.current = new ResizeObserver(() => {
        setPanelSize(panelRef.current?.getBoundingClientRect());
      });
      resizeObserver.current.observe(panelRef.current);
    }
    return (): void => {
      // update state to unmount and disconnect from resize observer
      componentWillUnmount.current = true;
      if (resizeObserver.current) {
        resizeObserver.current.disconnect();
      }
    };
  }, []);

  useEffect(() => {
    return (): void => {
      // This line only evaluates to true after the componentWillUnmount happens
      if (componentWillUnmount.current) {
        // clear monaco disposables:
        //    disposable objects includes autocomplete, code definition and editor actions
        monacoDisposables.forEach((d) => {
          d?.dispose();
        });
        // dispose all own models
        if (monaco) {
          ownUriPaths.forEach((uriPath) => {
            const model = monaco.editor.getModel(monaco.Uri.file(uriPath));
            if (model) {
              model.dispose();
            }
          });
          // remove undefined models
          monaco.editor.getModels().forEach((model) => {
            if (model.getValue().length === 0 && model.uri.path.indexOf(":") === -1) {
              model.dispose();
            }
          });
        }
        // remove modified files from context
        monacoCtx.updateModifiedFiles(tabId, "", []);
      }
    };
  }, [monacoDisposables, monaco, ownUriPaths, monacoCtx, tabId]);

  function onKeyDown(event: React.KeyboardEvent): void {
    if (event.ctrlKey && event.shiftKey && event.key === "E") {
      setEnableExplorer(!enableExplorer);
    }
    if (event.ctrlKey && event.shiftKey && event.key === "F") {
      setEnableGlobalSearch(!enableGlobalSearch);
    }
  }

  const getHostStyle = useCallback(
    function getHostStyle(): object {
      if (providerName && settingsCtx.get("colorizeHosts")) {
        return {
          flexGrow: 1,
          borderBottomStyle: "solid",
          borderBottomColor: colorFromHostname(providerName),
          borderBottomWidth: "0.3em",
        };
      }
      return { flexGrow: 1 };
    },
    [providerName, settingsCtx.changed]
  );

  // Most important function:
  //  when the component is mounted, this callback will execute following steps:
  //  - get the content of [currentFilePath]
  //  - create a monaco model (file in editor) based on [currentFilePath]
  //  - check if include files are available (for xml and launch files for example)
  //  -   if available, download all include files and create their corresponding models
  // We download the models per request. On recursive text search all files will be downloaded
  function loadFiles(): void {
    if (!editorRef.current) {
      return;
    }
    if (!monaco) {
      // monaco is not yet available
      setNotificationDescription("monaco is not yet available");
      return;
    }
    if (!currentFilePath || currentFilePath.length === 0) {
      setNotificationDescription("[currentFilePath] Invalid file path");
      return;
    }
    if (!rootFilePath || rootFilePath.length === 0) {
      setNotificationDescription("[rootFilePath] Invalid file path");
      return;
    }
    // search host based on selected provider
    const provider = rosCtx.getProviderById(providerId);
    if (!provider) {
      setNotificationDescription(`Provider with id ${providerId} not available`);
      return;
    }
    if (provider && !provider.host()) {
      logCtx.error("The provider does not have configured any host.", "Please check your provider configuration");
      setNotificationDescription("The provider does not have configured any host.");
      return;
    }
    setProviderHost(provider.host());
    setProviderName(provider.name());
    setNotificationDescription("Getting file from provider...");
    // get file content from provider and create monaco model
    async function getFileAndIncludesAsync(provider: Provider): Promise<void> {
      setCurrentFile({ name: getFileName(currentFilePath), requesting: true });
      const result: TModelResult = await monacoCtx.getModel(tabId, providerId, currentFilePath, false);
      setCurrentFile({ name: getFileName(currentFilePath), requesting: false });
      if (!result.model && result.file) {
        console.error(`Could not create model for: [${result.file.fileName}]`);
        setNotificationDescription(`Could not create model for: [${result.file.fileName}]`);
        return;
      }
      if (result.model) {
        updateOpenFiles(result);
        setEditorModel(result.model.uri.path, fileRange, launchArgs);
      }
      if (result.file && result.model) {
        updateInstallPathsWarn(result.file, result.model.uri.path);
      }

      // Ignore "non-launch" files
      // TODO: Add parameter Here
      if (result.file && !["launch", "xml", "xacro", "py"].includes(result.file.extension)) {
        console.log(`wrong extension: ${result.file.extension} of ${result.file}`);
        setIncludedFiles([]);
        setNotificationDescription("");
        return;
      }

      // if file is a launch or XML, try to fetch included files
      const request = new LaunchIncludedFilesRequest();
      request.path = rootFilePath;
      request.unique = false;
      request.recursive = true;
      const includedFilesLocal = await provider.launchGetIncludedFiles(request);
      if (includedFilesLocal) {
        // filter unique file names (in case multiple imports)
        const uniqueIncludedFiles = [rootFilePath];
        includedFilesLocal.forEach((f) => {
          if (!uniqueIncludedFiles.includes(f.inc_path)) uniqueIncludedFiles.push(f.inc_path);
        });

        // get file content and create corresponding monaco models on demand

        setIncludedFiles(includedFilesLocal);
      }
      setNotificationDescription("");
    }
    getFileAndIncludesAsync(provider);
  }

  // get text from clipboard for suggestions
  let clipTextSuggest = "";
  let clipTextReadyForSuggest = false;
  async function getClipboardTextForSuggest(): Promise<void> {
    await navigator.clipboard.readText().then((cbText) => {
      clipTextSuggest = cbText;
      clipTextReadyForSuggest = true;
      editorRef.current?.trigger("anything", "editor.action.triggerSuggest", {
        suggestions: [],
      });
      return true;
    });
  }

  function formatXml(xml: string, tab = 2): string {
    const xmlResult = new XmlBeautify().beautify(xml, tab);
    return xmlResult;
  }

  function configureMonacoEditor(): void {
    if (!monaco) return;

    monaco.languages.register({ id: "ros2xml" });
    monaco.languages.setMonarchTokensProvider("ros2xml", Ros2XmlLanguage);
    monaco.languages.setLanguageConfiguration("ros2xml", { comments: { blockComment: ["<!--", "-->"] } });

    monaco.languages.register({ id: "ros1xml" });
    monaco.languages.setMonarchTokensProvider("ros1xml", Ros1XmlLanguage);
    monaco.languages.setLanguageConfiguration("ros1xml", { comments: { blockComment: ["<!--", "-->"] } });

    // monaco.editor.setTheme("ros2xml");
    let packages: RosPackage[] = [];
    const provider = rosCtx.getProviderById(providerId, true);
    if (provider && provider.packages) {
      packages = provider.packages;
    }

    addMonacoDisposable(
      monaco.editor.registerLinkOpener({
        open(resource: Uri): boolean | Promise<boolean> {
          emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, resource.path.split(":")[1], null));
          return true;
        },
      })
    );

    ["ros2xml", "ros1xml", "launch", "python"].forEach((e) => {
      addMonacoDisposable(monaco.languages.registerLinkProvider(e, includesProvider));
    });
    const isRos2 = provider?.rosVersion === "2";
    // personalize launch file objects
    ["ros2xml", "ros1xml", "launch", "python"].forEach((e) => {
      // Add Completion provider for XML and launch files
      addMonacoDisposable(
        monaco.languages.registerCompletionItemProvider(e, {
          provideCompletionItems: (model, position) => {
            const word = model.getWordUntilPosition(position);
            const range = {
              startLineNumber: position.lineNumber,
              endLineNumber: position.lineNumber,
              startColumn: word.startColumn,
              endColumn: word.endColumn,
            };

            const lineContent = model.getLineContent(position.lineNumber);

            if (clipTextReadyForSuggest) {
              clipTextReadyForSuggest = false;

              switch(e) {
                case "python":
                  return { suggestions: createPythonLaunchProposals(monaco, range, clipTextSuggest, model.getValue(), packages) };
                case "ros1xml":
                  return { suggestions: createXMLDependencyProposals(monaco, range, lineContent, clipTextSuggest, packages) };
                case "ros2xml":
                  return { suggestions: createXMLDependencyProposalsR2(monaco, range, clipTextSuggest, packages) };
              }
            }
            getClipboardTextForSuggest();
            return {
              suggestions: [],
            };
          },
        })
      );

      // Add symbols XML and launch files
      if (e !== "python") {
        addMonacoDisposable(
          monaco.languages.registerDocumentSymbolProvider(e, {
            displayName: "ROS Symbols",
            provideDocumentSymbols: (model: editor.ITextModel /*, token: CancellationToken */) => {
              return isRos2 ? createDocumentSymbolsR2(model) : createDocumentSymbols(model);
            },
          })
        );
        addMonacoDisposable(
          monaco.languages.registerDocumentFormattingEditProvider("ros1xml", {
            async provideDocumentFormattingEdits(
              model: editor.ITextModel /*, options: languages.FormattingOptions, token: CancellationToken */
            ) {
              return [
                {
                  range: model.getFullModelRange(),
                  text: formatXml(model.getValue()),
                },
              ];
            },
          })
        );
        addMonacoDisposable(
          monaco.languages.registerDocumentFormattingEditProvider("ros2xml", {
            async provideDocumentFormattingEdits(
              model: editor.ITextModel /*, options: languages.FormattingOptions, token: CancellationToken */
            ) {
              return [
                {
                  range: model.getFullModelRange(),
                  text: formatXml(model.getValue()),
                },
              ];
            },
          })
        );
      }
    });

    if (editorRef.current) {
      addMonacoDisposable(
        editorRef.current?.addAction({
          id: "save_action",
          label: "Save",
          keybindings: [
            // eslint-disable-next-line no-bitwise
            monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyS,
          ],
          precondition: undefined,
          keybindingContext: undefined,
          contextMenuGroupId: "navigation",
          contextMenuOrder: 1.0,
          run: async (editorInstance: editor.ICodeEditor) => {
            const model = editorInstance.getModel();
            if (model) {
              saveCurrentFile(model);
            }
          },
        } as editor.IActionDescriptor)
      );
      addMonacoDisposable(
        editorRef.current.addAction({
          id: "command_palette",
          label: "Command Palette",
          keybindings: [
            // eslint-disable-next-line no-bitwise
            monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.KeyP,
          ],
          precondition: undefined,
          keybindingContext: undefined,
          // contextMenuGroupId: "none",
          // contextMenuOrder: 1.0,
          run: async (editorInstance: editor.ICodeEditor) => {
            editorInstance.trigger("open command palette", "editor.action.quickCommand", {});
          },
        })
      );
      addMonacoDisposable(
        editorRef.current.addAction({
          id: "toggle line comment",
          label: "Toggle line comment",
          keybindings: [
            // eslint-disable-next-line no-bitwise
            monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.Digit7,
          ],
          precondition: undefined,
          keybindingContext: undefined,
          // contextMenuGroupId: "none",
          // contextMenuOrder: 1.0,
          run: async (editorInstance: editor.ICodeEditor) => {
            editorInstance.trigger("toggle line comment", "editor.action.commentLine", {});
          },
        })
      );
    }
  }

  // initialization of provider definitions
  useEffect(() => {
    if (initialized || !editorRef.current || !monaco) return;
    editorRef.current.onMouseDown((event) => {
      // handle CTRL+click to open included files
      if (event.event.ctrlKey) {
        // setClickRequest(event.target.position);
      }
    });
    configureMonacoEditor();
    setInitialized(true);
    loadFiles();
    // uncomment to show supported actions
    // editorRef.current.getSupportedActions().forEach((value) => {
    //   console.log(value);
    // });
  }, [initialized, monaco, loadFiles]);

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      escapePressCount++;
      if (escapePressCount === 2) {
        const provider = rosCtx.getProviderById(providerId);
        if (provider) {
          const id = `editor-${provider.connection.host}-${provider.connection.port}-${rootFilePath}`;
          emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(id));
        }
      }
      // Reset after 500 ms
      setTimeout(() => {
        escapePressCount = 0;
      }, 500);
    }
  });

  function handleEditorDidMount(editor: editor.IStandaloneCodeEditor): void {
    editorRef.current = editor;
  }

  return (
    <Stack
      direction="row"
      height="100%"
      width="100%"
      onKeyDown={(event) => onKeyDown(event)}
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
          <SplitPane
            // defaultSize={sideBarWidth}
            sizes={[explorerBarHeight]}
            onChange={([size]) => {
              if (size !== explorerBarHeight && size >= explorerBarMinSize) {
                setSavedExplorerBarHight(size);
              }
              setExplorerBarHeight(size);
            }}
            split="horizontal"
            resizerSize={6}
            sashRender={(_index, active) => (
              <SashContent className={`sash-wrap-line ${active ? "active" : "inactive"}`}>
                {/* <span className="line"/> */}
              </SashContent>
            )}
          >
            <Pane minSize={explorerBarMinSize}>
              <Stack>
                <Stack direction="row" alignItems="center" spacing={1}>
                  <Tooltip title="Explorer (Ctrl+Shift+E)" placement="right" disableInteractive>
                    <ToggleButton
                      size="small"
                      value="showExplorer"
                      selected={enableExplorer}
                      onChange={() => {
                        handleChangeExplorer(!enableExplorer);
                      }}
                    >
                      <FolderCopyOutlinedIcon sx={{ fontSize: "inherit" }} />
                    </ToggleButton>
                  </Tooltip>
                  {enableExplorer && sideBarWidth > fontSize * 7 && <Typography fontSize="0.8em">Explorer</Typography>}
                </Stack>
                {enableExplorer && (
                  <Stack
                    overflow="auto"
                    direction="column"
                    height={explorerBarHeight - fontSize * 2 - 2}
                    width={sideBarWidth}
                  >
                    <ExplorerTree
                      tabId={tabId}
                      providerId={providerId}
                      rootFilePath={rootFilePath}
                      includedFiles={includedFiles}
                      selectedUriPath={activeModel?.path ? activeModel?.path : ""}
                      launchArgs={currentLaunchArgs}
                      modifiedUriPaths={modifiedFiles}
                    />
                  </Stack>
                )}
              </Stack>
            </Pane>
            <Stack paddingTop="2px">
              <Stack direction="row" alignItems="center" spacing={1}>
                <Tooltip title="Search (Ctrl+Shift+F)" placement="right" disableInteractive>
                  <ToggleButton
                    size="small"
                    value="showSearch"
                    selected={enableGlobalSearch}
                    onChange={() => handleChangeSearch(!enableGlobalSearch)}
                  >
                    <SearchOutlinedIcon sx={{ fontSize: "inherit" }} />
                  </ToggleButton>
                </Tooltip>
                {enableGlobalSearch && (
                  <SearchBar
                    onSearch={(value) => {
                      setGlobalSearchTerm(value);
                    }}
                    placeholder="Search in all included files..."
                    defaultValue={globalSearchTerm}
                    searchIcon={false}
                  />
                )}
              </Stack>
              {enableGlobalSearch && (
                <Stack
                  direction="column"
                  height={(panelSize?.height ? panelSize?.height : 100) - explorerBarHeight - fontSize * 2 - 8}
                  overflow="auto"
                >
                  <SearchTree
                    tabId={tabId}
                    providerId={providerId}
                    ownUriPaths={ownUriPaths}
                    searchTerm={globalSearchTerm}
                  />
                </Stack>
              )}
            </Stack>
          </SplitPane>
        </Pane>
        <Stack
          sx={{
            flex: 1,
            margin: 0,
          }}
          overflow="none"
        >
          <Stack
            direction="row"
            spacing={0.5}
            alignItems="center"
            ref={infoRef as ForwardedRef<HTMLDivElement>}
            sx={getHostStyle()}
          >
            <Tooltip title="Save File" disableInteractive>
              <span>
                <IconButton
                  edge="end"
                  disabled={!activeModel?.modified}
                  aria-label="Save File"
                  onClick={() => {
                    const model = editorRef.current?.getModel();
                    if (model) {
                      saveCurrentFile(model);
                    }
                  }}
                >
                  <SaveAltOutlinedIcon style={{ fontSize: "0.8em" }} />
                </IconButton>
              </span>
            </Tooltip>

            <OverflowMenu
              icon={<UpgradeIcon style={{ fontSize: "0.8em" }} />}
              tooltip="Open parent file"
              autoClick={true}
              options={selectParentFiles.map((item) => {
                return {
                  name: getFileName(item.path),
                  tooltip: item.path,
                  key: item.path,
                  onClick: (): void => {
                    setEditorModel(item.path, {
                      startLineNumber: item.line_number,
                      endLineNumber: item.line_number,
                      startColumn: 0,
                      endColumn: 0,
                    });
                  },
                };
              })}
              id="path-options"
            />

            <Tooltip title="Drop unsaved changes and reload current file from host" disableInteractive>
              <IconButton
                edge="end"
                aria-label="Reload file"
                onClick={async () => {
                  if (activeModel?.path) {
                    const path = activeModel?.path;
                    const result = await setEditorModel(path, selectionRange, currentLaunchArgs, true);
                    if (result) {
                      logCtx.success(`File reloaded [${getFileName(path)}]`);
                    }
                  }
                }}
              >
                <CloudSyncOutlinedIcon style={{ fontSize: "0.8em" }} />
              </IconButton>
            </Tooltip>
            <Stack direction="row" width="100%" spacing={0.4} alignItems="center">
              {currentFile.requesting && <CircularProgress size="0.8em" />}
              <Tooltip title={activeModel?.path?.split(":")[1]} disableInteractive>
                <Typography
                  noWrap
                  style={{
                    padding: "0.1em",
                    fontWeight: "normal",
                    fontSize: "0.8em",
                  }}
                >
                  {activeModel?.modified ? "*" : ""}
                  {currentFile.name}
                </Typography>
              </Tooltip>
              <Stack direction="row" spacing={0.2}>
                {modifiedFiles
                  .filter((path) => path !== activeModel?.path)
                  .map((path) => {
                    return (
                      <Tooltip
                        key={path}
                        title={`changed ${getFileName(path)}`}
                        enterDelay={tooltipDelay}
                        enterNextDelay={tooltipDelay}
                        disableInteractive
                      >
                        <Link
                          noWrap
                          aria-label={`modified ${path}`}
                          href="#"
                          // underline="none"
                          color="inherit"
                          onClick={() => {
                            setEditorModel(path);
                          }}
                        >
                          <Typography variant="body2" padding="0.4em" fontSize="0.6em">
                            {`[*${getFileAbb(path)}]`}
                          </Typography>
                        </Link>
                      </Tooltip>
                    );
                  })}
              </Stack>
              <Typography flexGrow={1} />
              <Typography
                noWrap
                style={{
                  padding: "0.4em",
                  fontWeight: 100,
                  fontSize: "0.6em",
                }}
              >
                {packageName} &#8226; {providerName}
              </Typography>
            </Stack>
          </Stack>
          {notificationDescription.length > 0 && (
            <Alert
              severity="warning"
              style={{ minWidth: 0 }}
              onClose={() => {
                setNotificationDescription("");
              }}
            >
              {notificationDescription}
            </Alert>
          )}
          {activeModel && isReadOnly(activeModel.path) ? (
            <Alert ref={alertRef as ForwardedRef<HTMLDivElement>} severity="info" style={{ minWidth: 0 }}>
              {`no write permissions for ${activeModel.path.split(":")[1]}`}
            </Alert>
          ) : (
            activeModel?.path &&
            installPathsWarn.includes(activeModel?.path) && (
              <Alert ref={alertRef as ForwardedRef<HTMLDivElement>} severity="warning" style={{ minWidth: 0 }}>
                {`${activeModel?.path.split(":")[1]} is located in 'install' path. The changes could be lost after rebuilding packages. You can build your packages with '--symlink-install' to edit your launch files in your sources.`}
              </Alert>
            )
          )}
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
              readOnly: activeModel?.model ? isReadOnly(activeModel.model.uri.path) : false,
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
