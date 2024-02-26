import Editor from '@monaco-editor/react';
import { TreeView } from '@mui/x-tree-view';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useRef, useState } from 'react';
import SplitPane from 'react-split-pane';

import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import CloudSyncOutlinedIcon from '@mui/icons-material/CloudSyncOutlined';
import FolderCopyOutlinedIcon from '@mui/icons-material/FolderCopyOutlined';
import SaveAltOutlinedIcon from '@mui/icons-material/SaveAltOutlined';
import SearchOutlinedIcon from '@mui/icons-material/SearchOutlined';
import UpgradeIcon from '@mui/icons-material/Upgrade';
import {
  IconButton,
  Stack,
  ToggleButton,
  Tooltip,
  Typography,
} from '@mui/material';
import { useCustomEventListener } from 'react-custom-events';
import { FileItem, getFileName } from '../../models';
import {
  createDocumentSymbols,
  createXMLDependencyProposals,
} from './MonacoTools';

import { DEFAULT_BUG_TEXT, LoggingContext } from '../../context/LoggingContext';
import { MonacoContext } from '../../context/MonacoContext';
import { RosContext } from '../../context/RosContext';
import { SSHContext } from '../../context/SSHContext';
import { SettingsContext } from '../../context/SettingsContext';
import useLocalStorage from '../../hooks/useLocalStorage';
import { EVENT_EDITOR_SELECT_RANGE } from '../../utils/events';
import { colorFromHostname } from '../UI/Colors';
import CopyButton from '../UI/CopyButton';
import SearchBar from '../UI/SearchBar';
import { FileTreeItem } from './FileTreeItem';
import { SearchFileTreeItem, SearchResultTreeItem } from './SearchTreeItem';

function MonacoEditor({
  rootFilePath,
  file,
  fileRange,
  providerId,
  provideDefinitionFunction,
  includedFiles,
  onChangeFile,
  onSaveFile,
  addMonacoDisposable,
  onCloseComponent,
  setTitlePanel,
}) {
  const settingsCtx = useContext(SettingsContext);
  const rosCtx = useContext(RosContext);
  const SSHCtx = useContext(SSHContext);
  const logCtx = useContext(LoggingContext);
  const monacoCtx = useContext(MonacoContext);

  const editorRef = useRef(null);
  const panelRef = useRef(null);
  const infoRef = useRef(null);
  const resizeObserver = useRef(null);
  const [panelHeight, setPanelHeight] = useState(0);
  const [panelWidth, setPanelWidth] = useState(0);

  const [providerName, setProviderName] = useState('');
  const [initialized, setInitialized] = useState(false);
  const [activeModel, setActiveModel] = useState(null);
  const [expandedExplorerResults, setExpandedExplorerResults] = useState([]);
  const [expandedSearchResults, setExpandedSearchResults] = useState([]);
  const [savedSideBarUserWidth, setSavedSideBarUserWidth] = useLocalStorage(
    'Editor:sideBarWidth',
    settingsCtx.get('fontSize') * 20,
  );
  const [savedExplorerBarHight, setSavedExplorerBarHight] = useLocalStorage(
    'Editor:explorerBarHight',
    settingsCtx.get('fontSize') * 20,
  );
  const [fontSize, setFontSize] = useState(16);
  const [sideBarMinSize, setSideBarMinSize] = useState(
    settingsCtx.get('fontSize') * 2,
  );
  const [sideBarWidth, setSideBarWidth] = useState(
    settingsCtx.get('fontSize') * 2,
  );
  const [explorerBarMinSize, setExplorerBarMinSize] = useState(
    settingsCtx.get('fontSize') * 2,
  );
  const [explorerBarHeight, setExplorerBarHeight] = useState(
    settingsCtx.get('fontSize') * 2,
  );
  const [editorHeight, setEditorHeight] = useState(20);
  const [editorWidth, setEditorWidth] = useState(20);
  const [selectedTabIndex, setSelectedTabIndex] = useState(0);
  const [listTabs, setListTabs] = useState([]);
  const [monacoViewStates] = useState(new Map('', ''));
  const [currentMonacoInput, setCurrentMonacoInput] = useState(null);
  const [clickRequest, setClickRequest] = useState(null);

  const textFieldRef = useRef(null);
  const [includeRoot, setIncludeRoot] = useState(null);
  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [enableExplorer, setEnableExplorer] = useState(false);
  const [globalSearchTerm, setGlobalSearchTerm] = useState('');
  const [globalSearchTree, setGlobalSearchTree] = useState({});

  const [selectionRange, setSelectionRange] = useState(fileRange);

  /** close tabs on signals from tab itself (e.g. ctrl+d) */
  useCustomEventListener(EVENT_EDITOR_SELECT_RANGE, (data) => {
    if (
      data.filePath === rootFilePath ||
      includedFiles.filter((f) => f.inc_path === data.filePath).length > 0
    ) {
      // open only if the file is in included files
      const uriPath = monacoCtx.monaco.Uri.file(
        `${data.fileHost}:${data.filePath}`,
      ).path;
      const model = monacoCtx.getModelFromPath(uriPath);
      if (model) {
        setEditorModel(uriPath, data.fileRange);
      }
    }
  });

  const handleChangeExplorer = useCallback(
    (isExpanded) => {
      setEnableExplorer(isExpanded);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [setEnableExplorer],
  );

  const handleChangeSearch = useCallback(
    (isExpanded) => {
      setEnableGlobalSearch(isExpanded);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [setEnableGlobalSearch],
  );

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

        editorRef.current.setSelection({
          startLineNumber: selectionRange.startLineNumber,
          endLineNumber: selectionRange.endLineNumber,
          startColumn: selectionRange.startColumn,
          endColumn: selectionRange.endColumn,
        });

        editorRef.current.focus();
      }
    }
  }, [selectionRange, editorRef.current]);

  function a11yProps(index) {
    return {
      id: `simple-tab-${index}`,
      key: `simple-tab-${index}`,
      'aria-controls': `simple-tabpanel-${index}`,
    };
  }
  const openCodeEditor = useCallback(
    (input) => {
      if (!input) return;

      const fileName = getFileName(input.uri.path);
      // check if tab exists
      const indexTab = listTabs.findIndex((t) => {
        return t.name === fileName;
      });

      const model = monacoCtx.getModelFromPath(input.uri.path);
      if (model) {
        if (indexTab === -1) {
          setListTabs((prev) => [
            ...prev,
            { name: fileName, path: input.uri.path },
          ]);
          setSelectedTabIndex(listTabs.length > 0 ? listTabs.length : 0);
        } else {
          setSelectedTabIndex(indexTab);
        }
        setActiveModel({ path: model.uri.path, modified: model.modified });
        setCurrentMonacoInput(input);
      } else {
        logCtx.error(`not supported file type ${fileName}`);
      }
    },
    [listTabs, setListTabs, monacoCtx],
  );

  useEffect(() => {
    if (clickRequest) {
      const resource = provideDefinitionFunction(
        editorRef.current.getModel(),
        clickRequest,
      );
      if (resource.length > 0 && resource[0].uri.scheme === 'file') {
        openCodeEditor(resource[0]);
      }

      setClickRequest(null);
    }
  }, [clickRequest]);

  // const openCodeEditorCallback = useCallback(
  //   (input, source) => {
  //     if (!input || !source) return;
  //     const fileName = getFileName(input.resource.path);

  //     // check if tab exists
  //     const indexTab = listTabs.findIndex((t) => t.name === fileName);

  //     if (indexTab === -1) {
  //       setListTabs((prev) => [
  //         ...prev,
  //         { name: fileName, path: input.resource.path },
  //       ]);
  //       setSelectedTabIndex(listTabs.length > 0 ? listTabs.length : 0);
  //     } else {
  //       setSelectedTabIndex(indexTab);
  //     }
  //     const model = monacoCtx.getModelFromPath(input.resource.path);
  //     setActiveModel({ path: model.uri.path, modified: model.modified });
  //     setCurrentMonacoInput(input);
  //   },
  //   [listTabs, setListTabs, monacoCtx],
  // );

  const onCloseTab = useCallback(
    (indexTab) => {
      if (indexTab === selectedTabIndex) {
        onChangeTab(selectedTabIndex - 1);
      }
      if (indexTab > -1) {
        setListTabs((prev) => {
          prev.splice(indexTab, 1);
          return prev;
        });
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [setListTabs, monacoCtx.monaco, selectedTabIndex],
  );

  // set the current model to the editor based on [uriPath], and update its decorations
  const setEditorModel = useCallback(
    async (uriPath, range = null, tabIndex = null, forceReload = false) => {
      if (!uriPath) return false;
      const provider = rosCtx.getProviderById(providerId);

      let modelUri = uriPath;
      if (modelUri.indexOf(':') === -1) {
        // create uriPath
        modelUri = monacoCtx.monaco.Uri.file(
          `${provider.host()}:${uriPath}`,
        ).path;
      }
      // If model does not exist, try to fetch it
      const modelExist = monacoCtx.existModelFromPath(modelUri);
      if (!modelExist || forceReload) {
        const filePath = modelUri.split(':')[1];
        if (provider) {
          const result = await provider.getFileContent(filePath);

          if (result && result.file) {
            if (modelExist) {
              monacoCtx.updateModel(result.file);
            } else {
              monacoCtx.createModel(result.file);
            }
          } else {
            logCtx.error(
              `Could not get file: ${filePath}`,
              `Provider: ${provider.name()}`,
            );
            return false;
          }
        }
      }

      // get model from path if exists
      const m = monacoCtx.getModelFromPath(modelUri);
      if (!m) {
        logCtx.error(
          `Could not get model for file: ${modelUri}`,
          `Provider: ${provider.name()}`,
        );
        return false;
      }

      // save current view state, in case user wants to open the file again
      // view state contains the cursor position, folding, selections etc...
      const currentModel = editorRef.current.getModel();
      if (currentModel) {
        monacoViewStates.set(
          currentModel.uri.path,
          editorRef.current.saveViewState(),
        );
      }

      editorRef.current.setModel(m);

      // restore view state for current file
      if (monacoViewStates.has(modelUri)) {
        editorRef.current.restoreViewState(monacoViewStates.get(modelUri));
      }

      // prepare file decorations
      const newDecorators = [];
      if (includedFiles) {
        includedFiles.forEach((f) => {
          const matches = m.findMatches(f.raw_inc_path);

          if (matches.length > 0) {
            matches.forEach((match) => {
              // Add a different style to "clickable" definitions
              newDecorators.push({
                range: match.range,
                options: { inlineClassName: 'filePathDecoration' },
              });
            });
          }
        });
      }
      m.deltaDecorations([], newDecorators);

      setActiveModel({ path: m.uri.path, modified: m.modified });

      // search tab index and set it up
      const foundIndex = listTabs.findIndex(
        (t) => t.name === getFileName(modelUri),
      );

      if (foundIndex !== -1) {
        setSelectedTabIndex(foundIndex);
      } else {
        // tab not found, create a new one
        setListTabs(() => [
          ...listTabs,
          { name: getFileName(modelUri), path: modelUri },
        ]);
        setSelectedTabIndex(listTabs.length);
      }
      // }

      // set range is available
      if (range) {
        setSelectionRange(range);
      }
      return true;
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      monacoCtx,
      monacoViewStates,
      includedFiles,
      setTitlePanel,
      rosCtx,
      providerId,
      SSHCtx,
      listTabs,
      setListTabs,
    ],
  );

  const onChangeTab = useCallback(
    async (selectedIndex) => {
      const index = Math.min(selectedIndex, listTabs.length - 1);
      const t = listTabs[index];

      if (listTabs.length === 0 || !t) {
        // no tabs available, close component
        if (onCloseComponent) onCloseComponent();
        return;
      }

      const success = await setEditorModel(t.path, null, index);
      if (!success) {
        logCtx.error('Could not find model', DEFAULT_BUG_TEXT);
      } else {
        setSelectedTabIndex(index);
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [listTabs, monacoCtx.monaco],
  );

  // get text from clipboard for suggestions
  let clipTextSuggest = '';
  let clipTextReadyForSuggest = false;
  const getClipboardTextForSuggest = async () => {
    await navigator.clipboard.readText().then((cbText) => {
      clipTextSuggest = cbText;
      clipTextReadyForSuggest = true;
      editorRef.current.trigger('anything', 'editor.action.triggerSuggest', {
        suggestions: [],
      });
      return true;
    });
  };

  const saveCurrentFile = useCallback(
    async (editorModel) => {
      const path = editorModel.uri.path.split(':')[1];
      // TODO change encoding if the file is encoded as HEX
      const fileToSave = new FileItem('', path, '', '', editorModel.getValue());
      const providerObj = rosCtx.getProviderById(providerId);
      if (providerObj) {
        const result = await providerObj.saveFileContent(fileToSave);
        if (result.bytesWritten > 0) {
          logCtx.success(`Successfully saved file`, `path: ${path}`);
          // unset modified flag of the current model
          setActiveModel((prevModel) => {
            const model = monacoCtx.getModelFromPath(prevModel.path);
            model.modified = false;
            return { path: model.uri.path, modified: model.modified };
          });
          if (onSaveFile) {
            onSaveFile();
          }
          // TODO: Important: Update definition provider function!
        } else {
          logCtx.error(`Error while save file ${path}`, `${result.error}`);
        }
      } else {
        logCtx.error(
          `Provider ${providerId} not found`,
          `can not save file: ${path}`,
        );
      }
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [listTabs, monacoCtx.monaco],
  );

  const configureMonacoEditor = (provider) => {
    // Add definition provider definitions (Enable "Go To Definition" option)
    // eslint-disable-next-line no-underscore-dangle
    // const editorServiceLocal = editorRef.current._codeEditorService;
    // editorServiceLocal.openCodeEditor = openCodeEditorCallback;

    editorRef.current.onMouseDown((event) => {
      // handle CTRL+click to open included files
      if (monacoCtx.monaco.KeyMod.CtrlCmd) {
        setClickRequest(event.target.position);
      }
    });

    // personalize launch file objects
    ['xml', 'launch'].forEach((e) => {
      // Add Completion provider for XML and launch files
      addMonacoDisposable(
        monacoCtx.monaco.languages.registerCompletionItemProvider(e, {
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
                suggestions: createXMLDependencyProposals(
                  monacoCtx.monaco,
                  range,
                  clipTextSuggest,
                ),
              };
            }
            getClipboardTextForSuggest();
            return {
              suggestions: [],
            };
          },
        }),
      );

      // Add symbols XML and launch files
      addMonacoDisposable(
        monacoCtx.monaco.languages.registerDocumentSymbolProvider(e, {
          displayName: 'ROS Symbols',
          provideDocumentSymbols: (model, token) => {
            return createDocumentSymbols(model, token);
          },
        }),
      );
    });

    addMonacoDisposable(
      editorRef.current.addAction({
        id: 'save_action',
        label: 'Save',
        keybindings: [
          // eslint-disable-next-line no-bitwise
          monacoCtx.monaco.KeyMod.CtrlCmd | monacoCtx.monaco.KeyCode.KeyS,
        ],
        precondition: null,
        keybindingContext: null,
        contextMenuGroupId: 'navigation',
        contextMenuOrder: 1.0,
        run: async (editorInstance) => {
          saveCurrentFile(editorInstance.getModel());
        },
      }),
    );
  };

  // initialization of tabs, activeModel and provider definitions
  useEffect(() => {
    if (initialized) return;

    if (!file || (file && !file.host) || !provideDefinitionFunction) {
      // clear list of tabs, required if using local storage
      setListTabs([]);
      return;
    }

    const model = monacoCtx.getModelFromPath(`${file.host}:${file.path}`);
    setActiveModel({ path: model?.uri.path, modified: model.modified });
    setListTabs([{ name: file.fileName, path: model.uri.path }]);

    // register definition providers
    // this will add "Go To Definition" capability to the monacoCtx.monaco editor
    // for file extensions "xml" and "launch"
    ['xml', 'launch'].forEach((e) => {
      addMonacoDisposable(
        monacoCtx.monaco.languages.registerDefinitionProvider(e, {
          provideDefinition: provideDefinitionFunction,
        }),
      );
    });
    setInitialized(true);
  }, [
    initialized,
    file,
    monacoCtx,
    setListTabs,
    provideDefinitionFunction,
    addMonacoDisposable,
  ]);

  // set a new editor model when [currentMonacoInput] changes
  //    usually due to new file opened or current tab changed
  useEffect(() => {
    if (!currentMonacoInput || !monacoCtx.monaco) return;

    const asyncWrap = async () => {
      // Set model and editor properties
      const success = await setEditorModel(
        currentMonacoInput.resource.path,
        null, // currentMonacoInput.options.selection,
        null,
      );
      if (!success) {
        logCtx.error(
          `Invalid model for path: [${currentMonacoInput.resource.path}]`,
          DEFAULT_BUG_TEXT,
        );
      }
    };
    asyncWrap();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [monacoCtx.monaco, currentMonacoInput]);

  const handleEditorDidMount = (editor) => {
    editorRef.current = editor;
    const provider = rosCtx.getProviderById(providerId);
    if (!provider.host()) return;
    // Set model and editor properties
    const uriPath = monacoCtx.monaco.Uri.file(`${file.host}:${file.path}`).path;
    console.log(`OPEN PATH: ${uriPath} ${JSON.stringify(fileRange)}`);
    setEditorModel(uriPath, fileRange);
    configureMonacoEditor(provider);
    setProviderName(provider.name());
  };

  // update the reference to the updated function [openCodeEditorCallback]
  // required to update list of tabs
  // useEffect(() => {
  //   if (!editorRef.current) return;

  //   // eslint-disable-next-line no-underscore-dangle
  //   // const editorServiceLocal = editorRef.current._codeEditorService;
  //   // editorServiceLocal.openCodeEditor = openCodeEditorCallback;
  // }, [openCodeEditorCallback]);

  const handleEditorChange = (value) => {
    file.value = value;

    // update activeModel modified flag
    setActiveModel((prevModel) => {
      const model = monacoCtx.getModelFromPath(prevModel.path);
      model.modified = true;
      return { path: model.uri.path, modified: model.modified };
    });

    if (onChangeFile) {
      onChangeFile();
    }
  };

  const debouncedFindAllMatches = useDebounceCallback((searchText) => {
    const searchResult = monacoCtx.findAllTextMatches(searchText);
    const newSearchTree = {};
    searchResult.forEach((item) => {
      const entry = newSearchTree[item.file];
      if (!entry) {
        newSearchTree[item.file] = [item];
      } else {
        newSearchTree[item.file].push(item);
      }
    });
    setGlobalSearchTree(newSearchTree);
    setExpandedSearchResults(Object.keys(newSearchTree));
  }, 50);

  const selectSearchResult = useCallback(
    (entry) => {
      const m = monacoCtx.getModelFromPath(entry.file);
      if (!m) {
        logCtx.error(`Could not get model for file: ${entry.file}`, '');
        return false;
      }
      editorRef.current.setModel(m);
      // restore view state for current file
      if (monacoViewStates.has(entry.file)) {
        editorRef.current.restoreViewState(monacoViewStates.get(entry.file));
      }
      setEditorModel(m.uri.path, entry.range);
      // setSelectionRange(entry.range);
      setActiveModel({ path: m.uri.path, modified: m.modified });
    },
    [editorRef],
  );

  const debouncedWidthUpdate = useDebounceCallback((newWidth) => {
    setEditorWidth(newWidth);
  }, 50);

  useEffect(() => {
    if (panelRef.current) {
      debouncedWidthUpdate(
        panelRef.current.getBoundingClientRect().width - sideBarWidth,
      );
    }
  }, [sideBarWidth]);

  useEffect(() => {
    debouncedFindAllMatches(globalSearchTerm);
  }, [globalSearchTerm]);

  // update FileEditorPanel's title when the active model changes
  // useEffect(() => {
  //   if (!activeModel) {
  //     return;
  //   }
  //   setTitlePanel(
  //     `${activeModel.modified ? '*' : ''}(${activeModel.path
  //       .split(':')[0]
  //       .slice(1)}) ${getFileName(activeModel.path)}`,
  //   );
  // }, [activeModel, setTitlePanel]);

  useEffect(() => {
    // update height and width of the split panel on the left side
    if (!(enableExplorer || enableGlobalSearch)) {
      setSideBarWidth(sideBarMinSize);
    } else {
      if (sideBarWidth <= sideBarMinSize) {
        setSideBarWidth(savedSideBarUserWidth);
      }
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
  }, [enableExplorer, enableGlobalSearch, panelHeight]);

  useEffect(() => {
    const newFontSize = settingsCtx.get('fontSize');
    setFontSize(newFontSize);
    setSideBarMinSize(newFontSize * 2 + 2);
    setSideBarWidth(newFontSize * 2 + 2);
    setExplorerBarMinSize(newFontSize * 2 + 2);
  }, [settingsCtx.changed]);

  useEffect(() => {
    const provider = rosCtx.getProviderById(providerId);
    if (!provider.host()) return;
    const uriPath = monacoCtx.monaco.Uri.file(
      `/${provider.host()}:${rootFilePath}`,
    ).path;
    const rootItem = {
      inc_path: uriPath,
      exists: true,
      rec_depth: -1,
      line_number: -1,
      children: [],
    };
    let currentFile = rootItem;
    includedFiles.forEach((file) => {
      file.children = [];
      if (file.rec_depth - 1 === currentFile.rec_depth) {
        currentFile.children.push(file);
      } else if (file.rec_depth - 1 > currentFile.rec_depth) {
        currentFile = currentFile.children.slice(-1)[0];
        currentFile.children.push(file);
      } else {
        currentFile = rootItem;
        while (file.rec_depth - 1 > currentFile.rec_depth) {
          currentFile = currentFile.children.slice(-1)[0];
        }
        currentFile.children.push(file);
      }
    });
    setExpandedExplorerResults([
      `${rootItem.inc_path}-${rootItem.line_number}`,
      ...includedFiles.map((item) => {
        return `${item.inc_path}-${item.line_number}`;
      }),
    ]);
    setIncludeRoot(rootItem);
  }, [includedFiles]);

  useEffect(() => {
    if (panelRef.current) {
      resizeObserver.current = new ResizeObserver(() => {
        setEditorHeight(
          panelRef.current?.getBoundingClientRect().height -
            infoRef.current?.getBoundingClientRect().height,
        );
        setEditorWidth(
          panelRef.current?.getBoundingClientRect().width - sideBarWidth,
        );
        setPanelHeight(panelRef.current?.getBoundingClientRect().height);
      });
      resizeObserver.current.observe(panelRef.current);
    }
    return () => {
      if (resizeObserver.current) {
        resizeObserver.current.disconnect();
      }
    };
  }, []);

  /** Create from TreeView from given root item */
  const includeFilesToTree = (file) => {
    if (!file) return;
    return (
      <FileTreeItem
        key={`${file.inc_path}-${file.line_number}`}
        nodeId={`${file.inc_path}-${file.line_number}`}
        labelText={`${getFileName(file.inc_path)}`}
        labelLine={file.line_number}
        textColor={!file.exists ? 'red' : ''}
        onLabelClick={(event) => {
          setEditorModel(file.inc_path);
          event.stopPropagation();
        }}
        onLinenumberClick={(event) => {
          setEditorModel(file.path, {
            startLineNumber: file.line_number,
            endLineNumber: file.line_number,
            startColumn: 0,
            endColumn: 0,
          });
          event.stopPropagation();
        }}
      >
        {file.children.map((child) => {
          return includeFilesToTree(child);
        })}
      </FileTreeItem>
    );
  };

  const onKeyDown = (event) => {
    if (event.ctrlKey && event.shiftKey && event.key === 'E') {
      setEnableExplorer(!enableExplorer);
    }
    if (event.ctrlKey && event.shiftKey && event.key === 'F') {
      setEnableGlobalSearch(!enableGlobalSearch);
    }
  };

  const getHostStyle = () => {
    if (providerName && settingsCtx.get('colorizeHosts')) {
      return {
        flexGrow: 1,
        borderBottomStyle: 'solid',
        borderBottomColor: colorFromHostname(providerName),
        borderBottomWidth: '0.3em',
      };
    }
    return { flexGrow: 1, alignItems: 'center' };
  };

  return (
    <Stack
      direction="row"
      height="100%"
      onKeyDown={onKeyDown}
      ref={panelRef}
      overflow="auto"
    >
      <SplitPane
        minSize={sideBarMinSize}
        // defaultSize={sideBarWidth}
        size={sideBarWidth}
        onChange={(size) => {
          if (size != sideBarMinSize) {
            setSavedSideBarUserWidth(size);
          }
          setSideBarWidth(size);
        }}
        split="vertical"
      >
        <SplitPane
          minSize={explorerBarMinSize}
          // defaultSize={sideBarWidth}
          size={explorerBarHeight}
          onChange={(size) => {
            if (size != explorerBarHeight) {
              setSavedExplorerBarHight(size);
            }
            setExplorerBarHeight(size);
          }}
          split="horizontal"
        >
          <Stack>
            <Stack direction="row" alignItems="center" spacing={1}>
              <Tooltip title="Explorer (Ctrl+Shift+E)" placement="right">
                <ToggleButton
                  size="small"
                  value="showExplorer"
                  selected={enableExplorer}
                  onChange={() => {
                    handleChangeExplorer(!enableExplorer);
                  }}
                >
                  <FolderCopyOutlinedIcon sx={{ fontSize: 'inherit' }} />
                </ToggleButton>
              </Tooltip>
              {enableExplorer && sideBarWidth > fontSize * 7 && (
                <Typography fontSize="0.8em">Explorer</Typography>
              )}
            </Stack>
            {enableExplorer && (
              <Stack
                overflow="auto"
                direction="column"
                height={explorerBarHeight}
                width={sideBarWidth}
              >
                <TreeView
                  aria-label="Explorer"
                  expanded={expandedExplorerResults}
                  defaultCollapseIcon={<ArrowDropDownIcon />}
                  defaultExpandIcon={<ArrowRightIcon />}
                  // defaultEndIcon={<div style={{ width: 24 }} />}
                  onNodeSelect={(event, nodeId) => {
                    const index = expandedExplorerResults.indexOf(nodeId);
                    const copyExpanded = [...expandedExplorerResults];
                    if (index === -1) {
                      copyExpanded.push(nodeId);
                    } else {
                      copyExpanded.splice(index, 1);
                    }
                    setExpandedExplorerResults(copyExpanded);
                  }}
                  sx={{ flexGrow: 1, overflow: 'auto' }}
                >
                  {includeFilesToTree(includeRoot)}
                </TreeView>
              </Stack>
            )}
          </Stack>
          <Stack paddingTop={'2px'}>
            <Stack direction="row" alignItems="center" spacing={1}>
              <Tooltip title="Search (Ctrl+Shift+F)" placement="right">
                <ToggleButton
                  size="small"
                  value="showSearch"
                  selected={enableGlobalSearch}
                  onChange={() => handleChangeSearch(!enableGlobalSearch)}
                >
                  <SearchOutlinedIcon sx={{ fontSize: 'inherit' }} />
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
                height={
                  panelRef.current.getBoundingClientRect().height -
                  explorerBarHeight -
                  fontSize * 2 -
                  8
                }
                overflow="auto"
              >
                <TreeView
                  aria-label="Search results"
                  expanded={expandedSearchResults}
                  defaultCollapseIcon={<ArrowDropDownIcon />}
                  defaultExpandIcon={<ArrowRightIcon />}
                  // defaultEndIcon={<div style={{ width: 24 }} />}
                  onNodeSelect={(event, nodeId) => {
                    const index = expandedSearchResults.indexOf(nodeId);
                    const copyExpanded = [...expandedSearchResults];
                    if (index === -1) {
                      copyExpanded.push(nodeId);
                    } else {
                      copyExpanded.splice(index, 1);
                    }
                    setExpandedSearchResults(copyExpanded);
                  }}
                  sx={{ flexGrow: 1, overflow: 'auto' }}
                >
                  {Object.keys(globalSearchTree).map((fileName) => {
                    const entries = globalSearchTree[fileName];
                    return (
                      <SearchFileTreeItem
                        fontSize="0.8em"
                        key={fileName}
                        nodeId={fileName}
                        labelText={`${getFileName(fileName)}`}
                        labelCount={entries.length}
                      >
                        {entries.map((entry) => {
                          return (
                            <SearchResultTreeItem
                              fontSize="0.8em"
                              key={`${fileName}-${entry.lineNumber}`}
                              nodeId={`${fileName}-${entry.lineNumber}`}
                              labelText={entry.lineNumber}
                              labelInfo={entry.text}
                              onClick={() => {
                                selectSearchResult(entry);
                              }}
                            ></SearchResultTreeItem>
                          );
                        })}
                      </SearchFileTreeItem>
                    );
                  })}
                </TreeView>
              </Stack>
            )}
          </Stack>
        </SplitPane>
        {file && (
          <Stack
            sx={{
              flex: 1,
              margin: 0,
            }}
            overflow="none"
          >
            {/* <AppBar position="static" color="transparent" elevation={0}> */}
            <Stack
              direction="row"
              spacing={0.5}
              alignItems="center"
              ref={infoRef}
              style={getHostStyle()}
            >
              <Tooltip title="Save File">
                <span>
                  <IconButton
                    edge="end"
                    disabled={!activeModel?.modified}
                    aria-label="Save File"
                    onClick={() => {
                      saveCurrentFile(editorRef.current.getModel());
                    }}
                  >
                    <SaveAltOutlinedIcon style={{ fontSize: '0.8em' }} />
                  </IconButton>
                </span>
              </Tooltip>
              <Tooltip title="Open parent file">
                <IconButton
                  edge="end"
                  aria-label="Open parent file"
                  onClick={async () => {
                    const path = activeModel?.path.split(':')[1];
                    const parentPaths = includedFiles.filter(
                      (item) => path === item.inc_path,
                    );
                    parentPaths.forEach(async (item) => {
                      const result = await setEditorModel(
                        `${activeModel?.path.split(':')[0]}:${item.path}`,
                        null,
                        null,
                        false,
                      );
                      if (result) {
                        logCtx.success(
                          `Parent file opened [${getFileName(path)}]`,
                        );
                      }
                    });
                  }}
                >
                  <UpgradeIcon style={{ fontSize: '0.8em' }} />
                </IconButton>
              </Tooltip>
              <Tooltip title="Reload current file from host">
                <IconButton
                  edge="end"
                  aria-label="Reload file"
                  onClick={async () => {
                    const path = activeModel?.path;
                    const result = await setEditorModel(path, null, null, true);
                    if (result) {
                      logCtx.success(`File reloaded [${getFileName(path)}]`);
                    }
                  }}
                >
                  <CloudSyncOutlinedIcon style={{ fontSize: '0.8em' }} />
                </IconButton>
              </Tooltip>
              <Stack direction="row" width="100%">
                <Typography
                  noWrap
                  style={{
                    padding: 2,
                    fontWeight: 'normal',
                    fontSize: '0.8em',
                  }}
                >
                  {getFileName(activeModel?.path)}
                  {activeModel?.modified ? '*' : ''}
                </Typography>
                <CopyButton
                  value={activeModel?.path?.split(':')[1]}
                  fontSize={'0.7em'}
                />
                <Typography flexGrow={1}></Typography>
                <Typography
                  noWrap
                  style={{
                    padding: 2,
                    fontWeight: 'normal',
                    fontSize: '0.8em',
                  }}
                >
                  {providerName}
                </Typography>
              </Stack>
            </Stack>
            <Editor
              key="editor"
              height={editorHeight}
              width={editorWidth}
              theme={settingsCtx.get('useDarkMode') ? 'vs-dark' : 'light'}
              onMount={handleEditorDidMount}
              onChange={handleEditorChange}
              // registerEditorOpener={(source, resource, selectionOrPosition) => {
              //   console.log(source, resource, selectionOrPosition);
              //   console.log(`Opened definition for ${resource}`);
              //   // if (resource.path === '/lib.dom.d.ts') {
              //   //   // simulate openening a new browser tab for our own type (open definition of alert)
              //   //   console.log(`Opened definition for ${resource}`);

              //   //   // alternatively set model directly in the editor if you have your own tab/navigation implementation
              //   //   // const model = monaco.editor.getModel(resource);
              //   //   // editor.setModel(model);
              //   //   // if (monaco.Range.isIRange(selectionOrPosition)) {
              //   //   // 	editor.revealRangeInCenterIfOutsideViewport(selectionOrPosition);
              //   //   // 	editor.setSelection(selectionOrPosition);
              //   //   // } else {
              //   //   // 	editor.revealPositionInCenterIfOutsideViewport(selectionOrPosition);
              //   //   // 	editor.setPosition(selectionOrPosition);
              //   //   // }

              //   //   return true;
              //   // }
              //   return false;
              // }}
              options={{
                // to check the all possible options check this - https://github.com/microsoft/monacoRef.current-editor/blob/a5298e1/website/typedoc/monacoRef.current.d.ts#L3017
                // TODO: make global config for this parameters
                readOnly: false,
                colorDecorators: true,
                mouseWheelZoom: true,
                scrollBeyondLastLine: false,
                smoothScrolling: false,
                wordWrap: 'off',
                fontSize: settingsCtx.get('fontSize'),
                minimap: { enabled: true },
                selectOnLineNumbers: true,
                'bracketPairColorization.enabled': true,
                guides: {
                  bracketPairs: true,
                },
                definitionLinkOpensInPeek: false,
                // automaticLayout: true,
              }}
            />
          </Stack>
        )}
      </SplitPane>
    </Stack>
  );
}

MonacoEditor.defaultProps = {
  rootFilePath: null,
  file: null,
  fileRange: null,
  provideDefinitionFunction: null,
  onChangeFile: null,
  onSaveFile: null,
  includedFiles: null,
  onCloseComponent: null,
  setTitlePanel: null,
};

MonacoEditor.propTypes = {
  rootFilePath: PropTypes.string,
  file: PropTypes.object,
  fileRange: PropTypes.any,
  providerId: PropTypes.string.isRequired,
  provideDefinitionFunction: PropTypes.func,
  includedFiles: PropTypes.any,
  onChangeFile: PropTypes.func,
  onSaveFile: PropTypes.func,
  addMonacoDisposable: PropTypes.func.isRequired,
  onCloseComponent: PropTypes.func,
  setTitlePanel: PropTypes.func,
};

export default MonacoEditor;
