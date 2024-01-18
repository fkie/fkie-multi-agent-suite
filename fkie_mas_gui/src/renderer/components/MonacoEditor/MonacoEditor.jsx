import Editor from '@monaco-editor/react';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useRef, useState } from 'react';

import {
  AppBar,
  Autocomplete,
  IconButton,
  Stack,
  Tab,
  Tabs,
  TextField,
  Toolbar,
  Tooltip,
} from '@mui/material';

import CloseOutlinedIcon from '@mui/icons-material/CloseOutlined';
import CloudSyncOutlinedIcon from '@mui/icons-material/CloudSyncOutlined';
import UpgradeIcon from '@mui/icons-material/Upgrade';

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
import CopyButton from '../UI/CopyButton';

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
  const [initialized, setInitialized] = useState(false);
  const [activeModel, setActiveModel] = useState(null);

  const [editorHeight, setEditorHeight] = useState(20);
  const [selectedTabIndex, setSelectedTabIndex] = useState(0);
  const [listTabs, setListTabs] = useState([]);
  const [monacoViewStates] = useState(new Map('', ''));
  const [currentMonacoInput, setCurrentMonacoInput] = useState(null);
  const [clickRequest, setClickRequest] = useState(null);

  const textFieldRef = useRef(null);
  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [globalSearchResults, setGlobalSearchResults] = useState([]);

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

      // If model does not exist, try to fetch it
      const modelExist = monacoCtx.existModelFromPath(uriPath);
      if (!modelExist || forceReload) {
        const filePath = uriPath.split(':')[1];
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
      const m = monacoCtx.getModelFromPath(uriPath);
      if (!m) {
        logCtx.error(
          `Could not get model for file: ${uriPath}`,
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
      if (monacoViewStates.has(uriPath)) {
        editorRef.current.restoreViewState(monacoViewStates.get(uriPath));
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
        (t) => t.name === getFileName(uriPath),
      );

      if (foundIndex !== -1) {
        setSelectedTabIndex(foundIndex);
      } else {
        // tab not found, create a new one
        setListTabs(() => [
          ...listTabs,
          { name: getFileName(uriPath), path: uriPath },
        ]);
        setSelectedTabIndex(listTabs.length);
      }
      // }

      // set range is available
      if (range) {
        editorRef.current.revealRangeInCenter({
          startLineNumber: range.startLineNumber,
          endLineNumber: range.endLineNumber,
          startColumn: range.startColumn,
          endColumn: range.endColumn,
        });

        editorRef.current.setPosition({
          lineNumber: range.startLineNumber,
          column: range.startColumn,
        });

        editorRef.current.setSelection({
          startLineNumber: range.startLineNumber,
          endLineNumber: range.endLineNumber,
          startColumn: range.startColumn,
          endColumn: range.endColumn,
        });

        editorRef.current.focus();
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
          const editorModel = editorInstance.getModel();
          const path = editorModel.uri.path.split(':')[1];
          // TODO change encoding if the file is encoded as HEX
          const fileToSave = new FileItem(
            '',
            path,
            '',
            '',
            editorModel.getValue(),
          );
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
      }),
    );

    addMonacoDisposable(
      editorRef.current.addAction({
        id: 'global_search',
        label: 'Search on included files',
        keybindings: [
          // eslint-disable-next-line no-bitwise
          monacoCtx.monaco.KeyMod.CtrlCmd |
            monacoCtx.monaco.KeyMod.Shift |
            monacoCtx.monaco.KeyCode.KeyF,
        ],
        precondition: null,
        keybindingContext: null,
        contextMenuGroupId: 'navigation',
        contextMenuOrder: 1.0,
        run: async () => {
          setEnableGlobalSearch(true);
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
    setEditorModel(uriPath, fileRange);
    configureMonacoEditor(provider);
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
    setGlobalSearchResults(searchResult);
  }, 50);

  // update FileEditorPanel's title when the active model changes
  useEffect(() => {
    if (!activeModel) {
      return;
    }
    setTitlePanel(
      `${activeModel.modified ? '*' : ''}(${activeModel.path
        .split(':')[0]
        .slice(1)}) ${getFileName(activeModel.path)}`,
    );
  }, [activeModel, setTitlePanel]);

  const panelRef = useRef(null);
  const resizeObserver = useRef(null);

  useEffect(() => {
    if (panelRef.current) {
      resizeObserver.current = new ResizeObserver(() => {
        setEditorHeight(panelRef.current.getBoundingClientRect().height - 62);
      });
      resizeObserver.current.observe(panelRef.current);
    }
    return () => {
      if (resizeObserver.current) {
        resizeObserver.current.disconnect();
      }
    };
  }, []);
  // <Stack height="100%" ref={panelRef}>

  return (
    <Stack height="100%" ref={panelRef}>
      {enableGlobalSearch && (
        <Stack>
          <AppBar position="static" color="transparent" elevation={0}>
            <Toolbar disableGutters={false} variant="dense">
              <Autocomplete
                id="autocomplete-global-search"
                size="sm"
                autoHighlight
                autoSelect
                clearOnEscape
                disableListWrap
                noOptionsText="Text was not found in files"
                options={globalSearchResults}
                groupBy={(option) => getFileName(option.file)}
                getOptionLabel={(option) => option.text}
                sx={{ flexGrow: 1, marginRight: 2 }}
                renderInput={(params) => (
                  <TextField
                    {...params}
                    ref={textFieldRef}
                    label="Search in all included files..."
                    // color=""
                    variant="filled"
                    size="small"
                    autoFocus
                    sx={{ fontSize: 'inherit' }}
                    onChange={(event) => {
                      debouncedFindAllMatches(event.target.value);
                    }}
                  />
                )}
                onChange={(event, searchResult) => {
                  if (searchResult) {
                    setEditorModel(searchResult.file, searchResult.range);
                  }
                }}
              />
              <IconButton
                edge="end"
                aria-label="close recursive search"
                onClick={async () => {
                  setEnableGlobalSearch(false);
                }}
              >
                <CloseOutlinedIcon style={{ fontSize: 'inherit' }} />
              </IconButton>
            </Toolbar>
          </AppBar>
        </Stack>
      )}
      {file && (
        <Stack
          sx={{
            flex: 1,
            margin: 0,
          }}
        >
          <Stack
            direction="row"
            justifyContent="space-between"
            paddingRight="10px"
          >
            <AppBar position="static" color="transparent" elevation={0}>
              <Tabs
                value={selectedTabIndex}
                style={{
                  padding: 0,
                  minHeight: '32px',
                  height: '32px',
                }}
                onChange={(event, newValue) => {
                  onChangeTab(newValue);
                }}
                aria-label="open-files"
                // contained
                variant="scrollable"
                scrollButtons="auto"
              >
                {listTabs.length > 0 && // Note: activeModel has to explicitly be used here to trigger an update of the tab names
                  listTabs.map((t, index) => {
                    const model =
                      activeModel?.path === t.path
                        ? activeModel
                        : monacoCtx.getModelFromPath(t.path);
                    const tabName = `${model?.modified ? '*' : ''}${t.name}`;
                    const paddingLeft = index > 0 ? 0 : 10;
                    return (
                      <Tab
                        style={{
                          paddingLeft,
                          paddingRight: 0,
                          minHeight: '32px',
                          height: '32px',
                          textTransform: 'none',
                        }}
                        value={index}
                        label={
                          <span>
                            {index > 0 && (
                              <IconButton
                                label="Close"
                                size="small"
                                component="span"
                                // sx={{ padding: 1 }}
                                onClick={() => {
                                  onCloseTab(index);
                                }}
                              >
                                <CloseOutlinedIcon
                                  style={{ fontSize: '12px' }}
                                  aria-label="Close"
                                />
                              </IconButton>
                            )}
                            {tabName}
                            {activeModel && (
                              <CopyButton
                                value={activeModel.path.split(':')[1]}
                              />
                            )}
                          </span>
                        }
                        {...a11yProps(index)}
                      />
                    );
                  })}
              </Tabs>
            </AppBar>
          </Stack>
          <Editor
            key="editor"
            height={editorHeight - (enableGlobalSearch ? 60 : 0)}
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
              fontSize: settingsCtx.fontSize,
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
          <Stack direction="row" spacing={0.5}>
            {/* <Tag color="default" text={activeModel?.path} wrap copyButton /> */}
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
                      logCtx.success(`File reloaded [${getFileName(path)}]`);
                    }
                  });
                }}
              >
                <UpgradeIcon style={{ fontSize: 'inherit' }} />
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
                <CloudSyncOutlinedIcon style={{ fontSize: 'inherit' }} />
              </IconButton>
            </Tooltip>
          </Stack>
        </Stack>
      )}
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
