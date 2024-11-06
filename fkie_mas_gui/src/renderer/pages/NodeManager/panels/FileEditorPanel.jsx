import RosContext from "@/renderer/context/RosContext";
import * as Monaco from "@monaco-editor/react";
import CloudSyncOutlinedIcon from "@mui/icons-material/CloudSyncOutlined";
import FolderCopyOutlinedIcon from "@mui/icons-material/FolderCopyOutlined";
import SaveAltOutlinedIcon from "@mui/icons-material/SaveAltOutlined";
import SearchOutlinedIcon from "@mui/icons-material/SearchOutlined";
import UpgradeIcon from "@mui/icons-material/Upgrade";
import { Alert, CircularProgress, IconButton, Link, Stack, ToggleButton, Tooltip, Typography } from "@mui/material";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";
import SplitPane, { Pane } from "split-pane-react";
import "split-pane-react/esm/themes/default.css";
import ExplorerTree, { equalLaunchArgs } from "../../../components/MonacoEditor/ExplorerTree";
import { createDocumentSymbols, createXMLDependencyProposals } from "../../../components/MonacoEditor/MonacoTools";
import SearchTree from "../../../components/MonacoEditor/SearchTree";
import XmlBeautify from "../../../components/MonacoEditor/XmlBeautify";
import { colorFromHostname } from "../../../components/UI/Colors";
import SearchBar from "../../../components/UI/SearchBar";
import { LoggingContext } from "../../../context/LoggingContext";
import { MonacoContext } from "../../../context/MonacoContext";
import { SettingsContext } from "../../../context/SettingsContext";
import useLocalStorage from "../../../hooks/useLocalStorage";
import { FileItem, LaunchIncludedFilesRequest, getFileAbb, getFileName } from "../../../models";
import { EVENT_PROVIDER_PATH_EVENT } from "../../../providers/eventTypes";
import { EVENT_CLOSE_COMPONENT, EVENT_EDITOR_SELECT_RANGE, eventCloseComponent } from "../layout/events";

function FileEditorPanel({ tabId, providerId, rootFilePath, currentFilePath, fileRange, launchArgs }) {
  const monaco = Monaco.useMonaco();
  const settingsCtx = useContext(SettingsContext);
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const monacoCtx = useContext(MonacoContext);

  // ----- size handling
  const editorRef = useRef(null);
  const panelRef = useRef(null);
  const infoRef = useRef(null);
  const resizeObserver = useRef(null);
  const componentWillUnmount = useRef(false);
  const [fontSize, setFontSize] = useState(settingsCtx.get("fontSize"));
  const [savedSideBarUserWidth, setSavedSideBarUserWidth] = useLocalStorage("Editor:sideBarWidth", fontSize * 20);
  const [savedExplorerBarHight, setSavedExplorerBarHight] = useLocalStorage("Editor:explorerBarHight", fontSize * 20);
  const [sideBarMinSize, setSideBarMinSize] = useState(fontSize * 2);
  const [sideBarWidth, setSideBarWidth] = useState(fontSize * 2);
  const [explorerBarMinSize, setExplorerBarMinSize] = useState(fontSize * 2);
  const [explorerBarHeight, setExplorerBarHeight] = useState(fontSize * 2);
  const [editorHeight, setEditorHeight] = useState(20);
  const [editorWidth, setEditorWidth] = useState(20);
  const [panelHeight, setPanelHeight] = useState(0);
  const [panelSize, setPanelSize] = useState(null);
  // ----- size handling end

  const [providerName, setProviderName] = useState("");
  const [providerHost, setProviderHost] = useState("");
  const [packageName, setPackageName] = useState("");
  const [initialized, setInitialized] = useState(false);
  const [activeModel, setActiveModel] = useState(null);
  const [currentFile, setCurrentFile] = useState({ name: "", requesting: false });
  const [ownUriPaths, setOwnUriPaths] = useState([]);
  const [ownUriToPackageDict] = useState({});
  const [monacoDisposables, setMonacoDisposables] = useState([]);
  const [monacoViewStates] = useState(new Map("", ""));
  const [clickRequest, setClickRequest] = useState(null);

  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [enableExplorer, setEnableExplorer] = useState(false);
  const [globalSearchTerm, setGlobalSearchTerm] = useState("");

  const [selectionRange, setSelectionRange] = useState(null);
  const [currentLaunchArgs, setCurrentLaunchArgs] = useState(launchArgs);
  const [modifiedFiles, setModifiedFiles] = useState([]);
  const [includedFiles, setIncludedFiles] = useState([]);
  const [includeDecorations, setIncludeDecorations] = useState([]);
  const [notificationDescription, setNotificationDescription] = useState("");
  const tooltipDelay = settingsCtx.get("tooltipEnterDelay");

  const [savedFiles, setSavedFiles] = useState([]);

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
    (isExpanded) => {
      setEnableExplorer(isExpanded);
    },
    [setEnableExplorer]
  );

  const handleChangeSearch = useCallback(
    (isExpanded) => {
      setEnableGlobalSearch(isExpanded);
    },
    [setEnableGlobalSearch]
  );

  /**
   * Returns include path if position is on the include declaration, overwise undefined.
   * @param {any} position - Mouse cursor position
   */
  const getIncludeResource = useCallback(
    (position) => {
      const declarations = includeDecorations.filter((item) => {
        return (
          item.range.startLineNumber <= position.lineNumber &&
          position.lineNumber <= item.range.endLineNumber &&
          item.range.startColumn <= position.column &&
          position.column <= item.range.endColumn
        );
      });
      if (declarations.length > 0) {
        return declarations[0].resource;
      }
      return undefined;
    },
    [includeDecorations]
  );

  const addMonacoDisposable = useCallback(
    (disposable) => {
      setMonacoDisposables((prev) => [...prev, disposable]);
    },
    [setMonacoDisposables]
  );

  // update modified files in this panel and context
  const updateModifiedFiles = useCallback(() => {
    const newModifiedFiles = monaco.editor
      .getModels()
      .filter((model) => {
        const result = model.modified && ownUriPaths.includes(model.uri.path);
        return result;
      })
      .map((model) => {
        return model.uri.path;
      });
    setModifiedFiles(newModifiedFiles);
    monacoCtx.updateModifiedFiles(tabId, providerId, newModifiedFiles);
  }, [monacoCtx, ownUriPaths, providerId, tabId]);

  // update decorations for included files
  const updateIncludeDecorations = (model, includedFilesList) => {
    if (!model) return;
    // prepare file decorations
    const newIncludeDecorations = [];
    const newDecorators = [];
    if (includedFilesList) {
      includedFilesList.forEach((f) => {
        const path = model.uri.path.split(":")[1];
        if (path !== f.inc_path) {
          const matches = model.findMatches(f.raw_inc_path);
          if (matches.length > 0) {
            matches.forEach((match) => {
              // Add a different style to "clickable" definitions
              newDecorators.push({
                range: match.range,
                options: { inlineClassName: "filePathDecoration" },
              });
              newIncludeDecorations.push({
                resource: f.inc_path,
                range: match.range,
              });
            });
          }
        }
      });
    }
    model.deltaDecorations([], newDecorators);
    setIncludeDecorations(newIncludeDecorations);
  };

  // set the current model to the editor based on [uriPath], and update its decorations
  const setEditorModel = useCallback(
    async (uriPath, range = null, launchArgs = null, forceReload = false) => {
      if (!uriPath) return false;
      setCurrentFile({ name: getFileName(uriPath), requesting: true });
      setNotificationDescription("Getting file from provider...");
      // If model does not exist, try to fetch it
      let result = await monacoCtx.getModel(tabId, providerId, uriPath, forceReload);
      setCurrentFile({ name: getFileName(uriPath), requesting: false });
      setNotificationDescription("");

      // get model from path if exists
      if (!result.model) {
        logCtx.error(`Could not get model for file: ${uriPath}`, "");
        return false;
      }

      // save current view state, in case user wants to open the file again
      // view state contains the cursor position, folding, selections etc...
      const currentModel = editorRef.current.getModel();
      if (currentModel) {
        monacoViewStates.set(currentModel.uri.path, editorRef.current.saveViewState());
      }

      editorRef.current.setModel(result.model);
      // restore view state for current file
      const viewState = monacoViewStates.get(result.model.uri.path);
      if (viewState) {
        editorRef.current.restoreViewState(viewState);
      }
      updateIncludeDecorations(result.model, includedFiles);
      setActiveModel({ path: result.model.uri.path, modified: result.model.modified, model: result.model });

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
    // eslint-disable-next-line react-hooks/exhaustive-deps
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
  useCustomEventListener(EVENT_EDITOR_SELECT_RANGE, async (data) => {
    if (data.tabId === tabId) {
      setEditorModel(data.filePath, data.fileRange, data.launchArgs);
    }
  });

  /** Handle events caused by changed files. */
  useCustomEventListener(EVENT_PROVIDER_PATH_EVENT, async (data) => {
    if (data.provider.id !== providerId) {
      // ignore event from other provider
      return;
    }
    const changedUri = monacoCtx.createUriPath(tabId, data.path.srcPath);
    if (ownUriPaths.includes(changedUri)) {
      // ignore if we saved the file
      if (savedFiles.includes(changedUri)) {
        setSavedFiles(savedFiles.filter((uri) => uri !== changedUri));
        // TODO: reload file content
        return;
      }
      const provider = rosCtx.getProviderById(providerId);
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
        if (currentModel.uri.path === model.uri.path) {
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
    // test on each click
    if (clickRequest) {
      const resource = getIncludeResource(clickRequest);
      if (resource) {
        setEditorModel(resource);
      }
      setClickRequest(null);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [clickRequest, getIncludeResource]);

  const saveCurrentFile = useCallback(
    async (editorModel) => {
      const path = editorModel.uri.path.split(":")[1];
      // TODO change encoding if the file is encoded as HEX
      const fileToSave = new FileItem("", path, "", "", editorModel.getValue());
      const providerObj = rosCtx.getProviderById(providerId);
      if (providerObj) {
        const saveResult = await providerObj.saveFileContent(fileToSave);
        if (saveResult.bytesWritten > 0) {
          const id = `editor-${providerObj.connection.host}-${providerObj.connection.port}-${rootFilePath}`;
          window.editorManager?.changed(id, path, false);
          if (!savedFiles.includes(editorModel.uri.path)) {
            setSavedFiles([...savedFiles, editorModel.uri.path]);
          }
          logCtx.success(`Successfully saved file`, `path: ${path}`);
          setActiveModel({ path: editorModel.uri.path, modified: false, model: editorModel });
          updateModifiedFiles();
        } else {
          logCtx.error(`Error while save file ${path}`, `${saveResult.error}`);
        }
      } else {
        logCtx.error(`Provider ${providerId} not found`, `can not save file: ${path}`);
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.getProviderById, monaco, setActiveModel, monacoCtx.getModel]
  );

  const debouncedWidthUpdate = useDebounceCallback(
    (newWidth) => {
      setEditorWidth(newWidth);
    },
    [setEditorWidth],
    50
  );

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(panelRef.current.getBoundingClientRect().width - sideBarWidth);
    }
  }, [debouncedWidthUpdate, sideBarWidth]);

  const cleanUpXmlComment = (changes, model) => {
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
  };

  const handleEditorChange = useCallback(
    async (value, event) => {
      // update activeModel modified flag only once
      cleanUpXmlComment(event.changes, activeModel?.model);
      if (activeModel) {
        if (!activeModel.modified) {
          setCurrentFile({ name: getFileName(activeModel.path), requesting: true });
          setNotificationDescription("Getting file from provider...");
          const result = await monacoCtx.getModel(tabId, providerId, activeModel.path, false);
          setCurrentFile({ name: getFileName(activeModel.path), requesting: false });
          setNotificationDescription("");
          result.model.modified = true;
          setActiveModel({ path: result.model.uri.path, modified: result.model.modified, model: result.model });
          updateModifiedFiles();
          const provider = rosCtx.getProviderById(providerId);
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
    const newFontSize = settingsCtx.get("fontSize");
    setFontSize(newFontSize);
    setSideBarMinSize(newFontSize * 2 + 2);
    setSideBarWidth(newFontSize * 2 + 2);
    setExplorerBarMinSize(newFontSize * 2 + 2);
  }, [settingsCtx, settingsCtx.changed]);

  useEffect(() => {
    // update ownUriPaths and package names (ownUriToPackageDict)
    const provider = rosCtx.getProviderById(providerId);
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
    setEditorHeight(panelSize.height - infoRef.current?.getBoundingClientRect().height);
    setEditorWidth(panelSize.width - sideBarWidth);
    setPanelHeight(panelSize.height);
  }, [sideBarWidth, panelSize]);

  useEffect(() => {
    // add resize observer to update size of monaco editor ad side bars
    if (panelRef.current) {
      resizeObserver.current = new ResizeObserver(() => {
        setPanelSize(panelRef.current?.getBoundingClientRect());
      });
      resizeObserver.current.observe(panelRef.current);
    }
    return () => {
      // update state to unmount and disconnect from resize observer
      componentWillUnmount.current = true;
      if (resizeObserver.current) {
        resizeObserver.current.disconnect();
      }
    };
  }, []);

  useEffect(() => {
    return () => {
      // This line only evaluates to true after the componentWillUnmount happens
      if (componentWillUnmount.current) {
        // clear monaco disposables:
        //    disposable objects includes autocomplete, code definition and editor actions
        monacoDisposables.forEach((d) => {
          d?.dispose();
        });
        // dispose all own models
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
        // remove modified files from context
        monacoCtx.updateModifiedFiles(tabId, "", []);
      }
    };
  }, [monacoDisposables, monaco, ownUriPaths, monacoCtx, tabId]);

  const onKeyDown = (event) => {
    if (event.ctrlKey && event.shiftKey && event.key === "E") {
      setEnableExplorer(!enableExplorer);
    }
    if (event.ctrlKey && event.shiftKey && event.key === "F") {
      setEnableGlobalSearch(!enableGlobalSearch);
    }
  };

  const getHostStyle = () => {
    if (providerName && settingsCtx.get("colorizeHosts")) {
      return {
        flexGrow: 1,
        borderBottomStyle: "solid",
        borderBottomColor: colorFromHostname(providerName),
        borderBottomWidth: "0.3em",
      };
    }
    return { flexGrow: 1, alignItems: "center" };
  };

  // Most important function:
  //  when the component is mounted, this callback will execute following steps:
  //  - get the content of [currentFilePath]
  //  - create a monaco model (file in editor) based on [currentFilePath]
  //  - check if include files are available (for xml and launch files for example)
  //  -   if available, download all include files and create their corresponding models
  // We download the models per request. On recursive text search all files will be downloaded
  const loadFiles = useCallback(() => {
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
    const getFileAndIncludesAsync = async () => {
      setCurrentFile({ name: getFileName(currentFilePath), requesting: true });
      const result = await monacoCtx.getModel(tabId, providerId, currentFilePath, false);
      setCurrentFile({ name: getFileName(currentFilePath), requesting: false });
      if (!result.model && result.file) {
        console.error(`Could not create model for: [${result.file.fileName}]`);
        setNotificationDescription(`Could not create model for: [${result.file.fileName}]`);
        return;
      }
      // setActiveModel({ path: model.uri.path, modified: model.modified });
      setEditorModel(result.model.uri.path, fileRange, launchArgs);

      // Ignore "non-launch" files
      // TODO: Add parameter Here
      if (!["launch", "xml", "xacro"].includes(result.file?.extension)) {
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

      // filter unique file names (in case multiple imports)
      const uniqueIncludedFiles = [rootFilePath];
      includedFilesLocal.forEach((f) => {
        if (!uniqueIncludedFiles.includes(f.inc_path)) uniqueIncludedFiles.push(f.inc_path);
      });

      // get file content and create corresponding monaco models on demand

      setIncludedFiles(includedFilesLocal);
      setNotificationDescription("");
    };
    getFileAndIncludesAsync();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    monaco,
    editorRef.current,
    currentFilePath,
    providerId,
    setEditorModel,
    monacoCtx.getModel,
    setProviderHost,
    setProviderName,
    setNotificationDescription,
    setIncludedFiles,
  ]);

  // get text from clipboard for suggestions
  let clipTextSuggest = "";
  let clipTextReadyForSuggest = false;
  const getClipboardTextForSuggest = async () => {
    await navigator.clipboard.readText().then((cbText) => {
      clipTextSuggest = cbText;
      clipTextReadyForSuggest = true;
      editorRef.current.trigger("anything", "editor.action.triggerSuggest", {
        suggestions: [],
      });
      return true;
    });
  };

  function formatXml(xml, tab = 2) {
    const xmlResult = new XmlBeautify().beautify(xml, tab);
    return xmlResult;
  }

  const configureMonacoEditor = () => {
    // !=> the goto functionality is provided by clickRequest

    // personalize launch file objects
    ["xml", "launch"].forEach((e) => {
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

            if (clipTextReadyForSuggest) {
              clipTextReadyForSuggest = false;
              return {
                suggestions: createXMLDependencyProposals(monaco, range, clipTextSuggest),
              };
            }
            getClipboardTextForSuggest();
            return {
              suggestions: [],
            };
          },
        })
      );

      // register definition providers
      // this will add "Go To Definition" capability to the monacoCtx.monaco editor
      // for file extensions "xml" and "launch"
      // addMonacoDisposable(
      //   monaco.languages.registerDefinitionProvider(e, {
      //     provideDefinition: provideDefinitionFunction,
      //   }),
      // );
      // !=> the goto functionality is provided by clickRequest

      // Add symbols XML and launch files
      addMonacoDisposable(
        monaco.languages.registerDocumentSymbolProvider(e, {
          displayName: "ROS Symbols",
          provideDocumentSymbols: (model, token) => {
            return createDocumentSymbols(model, token);
          },
        })
      );
      addMonacoDisposable(
        monaco.languages.registerDocumentFormattingEditProvider("xml", {
          async provideDocumentFormattingEdits(model, options, token) {
            return [
              {
                range: model.getFullModelRange(),
                text: formatXml(model.getValue()),
              },
            ];
          },
        })
      );
    });

    addMonacoDisposable(
      editorRef.current.addAction({
        id: "save_action",
        label: "Save",
        keybindings: [
          // eslint-disable-next-line no-bitwise
          monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyS,
        ],
        precondition: null,
        keybindingContext: null,
        contextMenuGroupId: "navigation",
        contextMenuOrder: 1.0,
        run: async (editorInstance) => {
          saveCurrentFile(editorInstance.getModel());
        },
      }),
      editorRef.current.addAction({
        id: "command_palette",
        label: "Command Palette",
        keybindings: [
          // eslint-disable-next-line no-bitwise
          monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.KeyP,
        ],
        precondition: null,
        keybindingContext: null,
        // contextMenuGroupId: "none",
        // contextMenuOrder: 1.0,
        run: async (editorInstance) => {
          editorInstance.trigger("open command palette", "editor.action.quickCommand");
        },
      }),
      editorRef.current.addAction({
        id: "toggle line comment",
        label: "Toggle line comment",
        keybindings: [
          // eslint-disable-next-line no-bitwise
          monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.Digit7,
        ],
        precondition: null,
        keybindingContext: null,
        // contextMenuGroupId: "none",
        // contextMenuOrder: 1.0,
        run: async (editorInstance) => {
          editorInstance.trigger("toggle line comment", "editor.action.commentLine");
        },
      })
    );
  };

  // initialization of provider definitions
  useEffect(() => {
    if (initialized || !editorRef.current || !monaco) return;
    editorRef.current.onMouseDown((event) => {
      // handle CTRL+click to open included files
      if (event.event.ctrlKey) {
        setClickRequest(event.target.position);
      }
    });
    configureMonacoEditor();
    setInitialized(true);
    loadFiles();
    // uncomment to show supported actions
    // editorRef.current.getSupportedActions().forEach((value) => {
    //   console.log(value);
    // });
    // eslint-disable-next-line react-hooks/exhaustive-deps
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

  const handleEditorDidMount = (editor) => {
    editorRef.current = editor;
  };

  return (
    <Stack direction="row" height="100%" width="100%" onKeyDown={onKeyDown} ref={panelRef} overflow="auto">
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
      >
        <Pane minSize={sideBarMinSize}>
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
                      selectedUriPath={activeModel?.path}
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
                  height={panelRef.current.getBoundingClientRect().height - explorerBarHeight - fontSize * 2 - 8}
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
          <Stack direction="row" spacing={0.5} alignItems="center" ref={infoRef} style={getHostStyle()}>
            <Tooltip title="Save File" disableInteractive>
              <span>
                <IconButton
                  edge="end"
                  disabled={!activeModel?.modified}
                  aria-label="Save File"
                  onClick={() => {
                    saveCurrentFile(editorRef.current.getModel());
                  }}
                >
                  <SaveAltOutlinedIcon style={{ fontSize: "0.8em" }} />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip title="Open parent file" disableInteractive>
              <IconButton
                edge="end"
                aria-label="Open parent file"
                onClick={async () => {
                  const path = activeModel?.path.split(":")[1];
                  const parentPaths = includedFiles.filter((item) => {
                    if (path === item.inc_path) {
                      // check args to select the correct file, if the same file included twice
                      return equalLaunchArgs(currentLaunchArgs, item.args);
                    }
                    return false;
                  });
                  parentPaths.forEach(async (item) => {
                    const result = await setEditorModel(
                      `${activeModel?.path.split(":")[0]}:${item.path}`,
                      null,
                      null,
                      false
                    );
                    if (result) {
                      logCtx.success(`Parent file opened [${getFileName(path)}]`);
                      setEditorModel(item.path, {
                        startLineNumber: item.line_number,
                        endLineNumber: item.line_number,
                        startColumn: 0,
                        endColumn: 0,
                      });
                    }
                  });
                }}
              >
                <UpgradeIcon style={{ fontSize: "0.8em" }} />
              </IconButton>
            </Tooltip>
            <Tooltip title="Drop unsaved changes and reload current file from host" disableInteractive>
              <IconButton
                edge="end"
                aria-label="Reload file"
                onClick={async () => {
                  const path = activeModel?.path;
                  const result = await setEditorModel(path, selectionRange, currentLaunchArgs, true);
                  if (result) {
                    logCtx.success(`File reloaded [${getFileName(path)}]`);
                  }
                }}
              >
                <CloudSyncOutlinedIcon style={{ fontSize: "0.8em" }} />
              </IconButton>
            </Tooltip>
            <Stack direction="row" width="100%" spacing={0.4} alignItems="center">
              {currentFile.requesting && <CircularProgress size="0.8em" />}
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
          <Monaco.Editor
            key="editor"
            height={editorHeight}
            width={editorWidth}
            theme={settingsCtx.get("useDarkMode") ? "vs-dark" : "light"}
            onMount={handleEditorDidMount}
            onChange={handleEditorChange}
            options={{
              // to check the all possible options check this - https://github.com/microsoft/monacoRef.current-editor/blob/a5298e1/website/typedoc/monacoRef.current.d.ts#L3017
              // TODO: make global config for this parameters
              readOnly: false,
              colorDecorators: true,
              mouseWheelZoom: true,
              scrollBeyondLastLine: false,
              smoothScrolling: false,
              wordWrap: "off",
              fontSize: settingsCtx.get("fontSize"),
              minimap: { enabled: true },
              selectOnLineNumbers: true,
              "bracketPairColorization.enabled": true,
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

FileEditorPanel.propTypes = {
  tabId: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
  rootFilePath: PropTypes.string.isRequired,
  currentFilePath: PropTypes.string.isRequired,
  fileRange: PropTypes.any,
  launchArgs: PropTypes.any,
};

export default FileEditorPanel;
