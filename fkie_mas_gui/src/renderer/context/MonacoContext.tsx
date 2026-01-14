/* eslint-disable max-classes-per-file */
import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";
import { editor } from "monaco-editor/esm/vs/editor/editor.api";
import editorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import cssWorker from "monaco-editor/esm/vs/language/css/css.worker?worker";
import htmlWorker from "monaco-editor/esm/vs/language/html/html.worker?worker";
import jsonWorker from "monaco-editor/esm/vs/language/json/json.worker?worker";
import tsWorker from "monaco-editor/esm/vs/language/typescript/ts.worker?worker";
import React, { createContext, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

self.MonacoEnvironment = {
  getWorker(_, label): Worker {
    if (label === "json") {
      return new jsonWorker();
    }
    if (label === "css" || label === "scss" || label === "less") {
      return new cssWorker();
    }
    if (label === "html" || label === "handlebars" || label === "razor") {
      return new htmlWorker();
    }
    if (label === "typescript" || label === "javascript") {
      return new tsWorker();
    }
    return new editorWorker();
  },
};

// use local monaco editor sources
// see: https://github.com/suren-atoyan/monaco-react?tab=readme-ov-file#loader-config
MonacoReact.loader.config({ monaco });

import { FileItem, FileLanguageAssociations } from "@/renderer/models";
import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  eventCloseComponent,
  eventEditorSelectRange,
} from "@/renderer/pages/NodeManager/layout/events";
import { TFileRange, TLaunchArg } from "@/types";
import { PythonLanguage } from "../components/MonacoEditor/PythonLaunchHighlighter";
import XmlBeautify from "../components/MonacoEditor/XmlBeautify";
import { Ros1XmlLanguage } from "../components/MonacoEditor/XmlLaunchHighlighter";
import { Ros2XmlLanguage } from "../components/MonacoEditor/XmlLaunchHighlighterR2";
import { LoggingContext } from "./LoggingContext";
import { RosContext } from "./RosContext";

export type TModelResult = {
  model: monaco.editor.ITextModel | null;
  file: FileItem | null;
  error: string;
};

export type TModelVersion = {
  path: string;
  version: number;
};

export class ModifiedTabsInfo {
  tabId: string = "";

  providerId: string = "";

  uriPaths: string[] = [];

  constructor(tabId: string, providerId: string, uriPaths: string[]) {
    this.tabId = tabId;
    this.providerId = providerId;
    this.uriPaths = uriPaths;
  }
}

export class SaveResult {
  tabId: string = "";

  file: string = "";

  result: boolean = false;

  providerId: string = "";

  message: string = "";

  constructor(tabId: string, file: string, result: boolean, providerId: string, message: string) {
    this.tabId = tabId;
    this.file = file;
    this.result = result;
    this.providerId = providerId;
    this.message = message;
  }
}

export interface IMonacoContext {
  monaco: MonacoReact.Monaco | null;
  savedModelVersions: TModelVersion[];
  updateModifiedFiles: (tabId: string, providerId: string, uriPaths: string[]) => void;
  clearModifiedTabs: (tabIds: ModifiedTabsInfo[]) => void;
  getModifiedTabs: () => ModifiedTabsInfo[];
  getModifiedFilesByTab: (tabId: string) => ModifiedTabsInfo | undefined;
  saveModifiedFilesOfTabId: (tabId: string) => Promise<SaveResult[]>;
  createUriPath: (tabId: string, path: string) => string;
  getModel: (
    tabId: string,
    providerId: string,
    path: string,
    forceReload: boolean
  ) => Promise<{ model: monaco.editor.ITextModel | null; file: FileItem | null; error: string }>;
  createModel: (tabId: string, file: FileItem) => monaco.editor.ITextModel | null;
  saveFile(tabId: string, providerId: string, uriPath: string): Promise<SaveResult>;
  isModifiedModel: (model: editor.ITextModel) => boolean;
}

export interface IMonacoProvider {
  children: React.ReactNode;
}
export const DEFAULT_MONACO = {
  monaco: null,
  savedModelVersions: [],
  updateModifiedFiles: (): void => {},
  clearModifiedTabs: (): void => {},
  getModifiedTabs: (): ModifiedTabsInfo[] => [],
  getModifiedFilesByTab: (): ModifiedTabsInfo | undefined => undefined,
  saveModifiedFilesOfTabId: (): Promise<SaveResult[]> => {
    return Promise.resolve([]);
  },
  createUriPath: (): string => "",
  getModel: (): Promise<TModelResult> =>
    Promise.resolve({
      model: null,
      file: null,
      error: "",
    }),
  createModel: (): monaco.editor.ITextModel | null => null,
  saveFile: (): Promise<SaveResult> => {
    return Promise.resolve({ tabId: "", file: "", result: false, providerId: "", message: "" });
  },
  isModifiedModel: () => false,
};

export const MonacoContext = createContext<IMonacoContext>(DEFAULT_MONACO);

export function MonacoProvider({ children }: IMonacoProvider): ReturnType<React.FC<IMonacoProvider>> {
  const monaco = MonacoReact.useMonaco();
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);

  const [modifiedFiles, setModifiedFiles] = useState<ModifiedTabsInfo[]>([]);
  const [savedModelVersions, setSavedModelVersions] = useState<TModelVersion[]>([]);

  useEffect(() => {
    window.editorManager?.onFileRange(
      (tabId: string, filePath: string, fileRange: TFileRange | null, launchArgs: TLaunchArg[]) => {
        if (fileRange) {
          emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, filePath, fileRange, launchArgs));
        }
      }
    );
    window.editorManager?.onClose((tabId: string) => {
      emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(tabId));
    });
  }, []);

  function formatXml(xml: string, tab = 2): string {
    const xmlResult = new XmlBeautify().beautify(xml, tab);
    return xmlResult;
  }

  useEffect(() => {
    monaco?.languages.typescript.javascriptDefaults.setEagerModelSync(true);
    monaco?.languages.typescript.typescriptDefaults.setEagerModelSync(true);
    MonacoReact.loader.init().then((monaco) => {
      monaco.editor.defineTheme("vs-ros-light", {
        base: "vs",
        inherit: true,
        colors: {},
        rules: [
          { token: "delimiter.start", foreground: "#008000", fontStyle: "bold" },
          { token: "delimiter.end", foreground: "#008000", fontStyle: "bold" },
          { token: "tag", foreground: "#008000", fontStyle: "bold" },
          { token: "attribute.name", foreground: "#7D9029" },
          { token: "attribute.value", foreground: "#BA2121" },
          { token: "subst.key", foreground: "#009000", fontStyle: "bold" },
          { token: "subst.arg", foreground: "#BA2121", fontStyle: "bold" },
          { token: "comment", foreground: "#666666", fontStyle: "italic" },
          { token: "error-token", foreground: "#ff0000ff", fontStyle: "bold underline" },
        ],
      });
      monaco.editor.defineTheme("vs-ros-dark", {
        base: "vs-dark",
        inherit: true,
        colors: {},
        rules: [
          { token: "delimiter.start", foreground: "#008000", fontStyle: "bold" },
          { token: "delimiter.end", foreground: "#008000", fontStyle: "bold" },
          { token: "tag", foreground: "#008000", fontStyle: "bold" },
          { token: "attribute.name", foreground: "#7D9029" },
          // { token: "attribute.value", foreground: "#BA2121" },
          { token: "subst.key", foreground: "#009000", fontStyle: "bold" },
          { token: "subst.arg", foreground: "#996633", fontStyle: "bold" },
          { token: "comment", foreground: "#999999", fontStyle: "italic" },
          { token: "error-token", foreground: "#ff0000ff", fontStyle: "bold underline" },
        ],
      });
      monaco.languages.registerHoverProvider("ros2xml", {
        provideHover: (model, position) => {
          const wordInfo = model.getWordAtPosition(position);
          if (!wordInfo) return null;
          if (wordInfo.word === "false" || wordInfo.word === "true") {
            return {
              range: new monaco.Range(
                position.lineNumber,
                wordInfo.startColumn,
                position.lineNumber,
                wordInfo.endColumn
              ),
              contents: [{ value: "Use uppercase True/False, otherwise some eval statements may fail." }],
            };
          }
          return null;
        },
      });
      monaco.languages.register({ id: "ros2xml" });
      monaco.languages.setMonarchTokensProvider("ros2xml", Ros2XmlLanguage);
      monaco.languages.setLanguageConfiguration("ros2xml", {
        comments: { blockComment: ["<!--", "-->"] },
        autoClosingPairs: [
          { open: "<", close: ">" },
          { open: '"', close: '"' },
          { open: "'", close: "'" },
        ],
        brackets: [["<", ">"]],
        onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
      });
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
      });

      monaco.languages.register({ id: "ros1xml" });
      monaco.languages.setMonarchTokensProvider("ros1xml", Ros1XmlLanguage);
      monaco.languages.setLanguageConfiguration("ros1xml", {
        comments: { blockComment: ["<!--", "-->"] },
        autoClosingPairs: [
          { open: "<", close: ">" },
          { open: '"', close: '"' },
        ],
        brackets: [["<", ">"]],
        onEnterRules: [{ beforeText: />/, afterText: /<\//, action: { indentAction: 2 } }],
      });
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
      });

      monaco.languages.register({ id: "python" });
      monaco.languages.setMonarchTokensProvider("python", PythonLanguage);
    });
  }, [monaco]);

  const updateModifiedFiles = useCallback(
    (tabId: string, providerId: string, uriPaths: string[]): void => {
      if (uriPaths.length > 0) {
        // add to the list
        const newFilesInfo: ModifiedTabsInfo = new ModifiedTabsInfo(tabId, providerId, uriPaths);
        setModifiedFiles((prev) => {
          return [...prev.filter((item) => item.tabId !== tabId), newFilesInfo];
        });
      } else {
        // remove from the list
        setModifiedFiles((prev) => {
          return prev.filter((item) => item.tabId !== tabId);
        });
      }
    },
    [setModifiedFiles]
  );

  const clearModifiedTabs = useCallback(
    (tabIds?: ModifiedTabsInfo[]): void => {
      if (tabIds && tabIds.length > 0) {
        setModifiedFiles((prev) => {
          return prev.filter((item) => tabIds.filter((tabI) => tabI.tabId === item.tabId).length === 0);
        });
      } else {
        setModifiedFiles([]);
      }
    },
    [modifiedFiles]
  );

  const getModifiedTabs = useCallback((): ModifiedTabsInfo[] => modifiedFiles, [modifiedFiles]);

  const getModifiedFilesByTab = useCallback(
    (tabId: string): ModifiedTabsInfo | undefined => modifiedFiles.filter((item) => item.tabId === tabId)[0],
    [modifiedFiles]
  );

  const saveFile = useCallback(
    async (tabId: string, providerId: string, uriPath: string): Promise<SaveResult> => {
      const path = uriPath.split(":")[1];
      const saveItem: SaveResult = new SaveResult(tabId, path, false, providerId, "");
      if (!monaco) {
        saveItem.message = "monaco is invalid";
        return Promise.resolve(saveItem);
      }
      const editorModel = monaco.editor.getModel(monaco.Uri.file(uriPath));
      if (editorModel) {
        const path = editorModel.uri.path.split(":")[1];
        // TODO change encoding if the file is encoded as HEX
        const fileToSave = new FileItem("", path, "", "", false, editorModel.getValue());
        const providerObj = rosCtx.getProviderById(providerId, false);
        if (providerObj) {
          const saveResult = await providerObj.saveFileContent(fileToSave);
          if (saveResult.bytesWritten > 0) {
            setSavedModelVersions((prev) => [
              ...prev.filter((item) => {
                return item.path !== editorModel.uri.path;
              }),
              { path: editorModel.uri.path, version: editorModel.getAlternativeVersionId() } as TModelVersion,
            ]);
            logCtx.success("Successfully saved file", `path: ${path}`, "saved");
            saveItem.result = true;
          } else {
            saveItem.message = `Error while save file ${path}: ${saveResult.error}`;
            logCtx.error(`Error while save file ${path}`, `${saveResult.error}`, "not saved");
          }
        } else {
          saveItem.message = `Provider ${providerId} not found`;
          logCtx.error(`Provider ${providerId} not found`, `can not save file: ${path}`, "not saved, no provider");
        }
      } else {
        saveItem.message = `Model for ${path} not found`;
      }
      return Promise.resolve(saveItem);
    },
    [rosCtx]
  );

  const saveModifiedFilesOfTabId = useCallback(
    async (tabId: string): Promise<SaveResult[]> => {
      if (!monaco) return Promise.resolve([]);
      const result: SaveResult[] = [];
      const tabInfos: ModifiedTabsInfo[] = modifiedFiles.filter((item) => item.tabId === tabId);
      if (tabInfos.length === 0) return Promise.resolve([]);
      const tabInfo: ModifiedTabsInfo = tabInfos[0];
      await Promise.all(
        tabInfo.uriPaths.map(async (uriPath) => {
          const saveItem = await saveFile(tabId, tabInfo.providerId, uriPath);
          result.push(saveItem);
        })
      );
      return Promise.resolve(result);
    },
    [logCtx, modifiedFiles, monaco, rosCtx]
  );

  const isModifiedModel = useCallback(
    (model: editor.ITextModel): boolean => {
      const item: TModelVersion | undefined = savedModelVersions.find((item) => {
        return item.path === model.uri.path;
      });
      if (item) {
        return item.version !== model.getAlternativeVersionId();
      }
      return model.getAlternativeVersionId() > 1;
    },
    [savedModelVersions]
  );

  const createUriPath = useCallback((tabId: string, path: string): string => {
    if (path.indexOf(":") !== -1) {
      return path;
    }
    return `/${tabId}:${path}`;
  }, []);

  /**
   * Return a monaco model from a given path
   */
  function getModelFromPath(tabId: string, path: string): monaco.editor.ITextModel | null {
    if (!monaco) return null;
    if (!path || path.length === 0) return null;
    let modelUri = path;
    if (modelUri.indexOf(":") === -1) {
      // create uriPath
      modelUri = createUriPath(tabId, path);
    }
    return monaco.editor.getModel(monaco.Uri.file(modelUri));
  }

  /**
   * Create a new monaco model from a given file
   * @param file - Original file
   */
  const createModel = useCallback(
    (tabId: string, file: FileItem): monaco.editor.ITextModel | null => {
      if (!monaco) return null;
      const pathUri = monaco.Uri.file(createUriPath(tabId, file.path));
      // create monaco model, if it does not exists yet
      const model = monaco.editor.getModel(pathUri);
      if (model) {
        // if (model.modified) {
        //   // TODO: ask the user how to proceed
        //   return model;
        // }
        model.dispose();
      }
      return monaco.editor.createModel(file.value, FileLanguageAssociations[file.extension], pathUri);
    },
    [monaco]
  );

  const getModel = useCallback(
    async (
      tabId: string,
      providerId: string,
      path: string,
      forceReload: boolean
    ): Promise<{ model: monaco.editor.ITextModel | null; file: FileItem | null; error: string }> => {
      let model: monaco.editor.ITextModel | null = getModelFromPath(tabId, path);
      if (!model || forceReload) {
        const provider = rosCtx.getProviderById(providerId, false);
        if (!provider) {
          return Promise.resolve({ model: null, file: null, error: "" });
        }
        const filePath = path.indexOf(":") === -1 ? path : path.split(":")[1];
        const { file, error } = await provider.getFileContent(filePath);
        if (!error) {
          model = createModel(tabId, file);
          if (!model) {
            logCtx.error(
              `Could not create model for included file: [${file.fileName}]`,
              `Host: ${provider.host()}, root file: ${file.path}`,
              "file model error"
            );
          }
          return Promise.resolve({ model: model, file, error });
        }
        console.error(`Could not open included file: [${file.fileName}]: ${error}`);
      }
      return Promise.resolve({ model: model, file: null, error: "" });
    },
    [rosCtx]
  );

  const attributesMemo = useMemo(
    () => ({
      monaco,
      modifiedFiles,
      savedModelVersions,
      setModifiedFiles,
      getModifiedTabs,
      updateModifiedFiles,
      clearModifiedTabs,
      getModifiedFilesByTab,
      saveModifiedFilesOfTabId,
      createUriPath,
      getModel,
      createModel,
      saveFile,
      isModifiedModel,
    }),
    [
      monaco,
      rosCtx,
      logCtx,
      modifiedFiles,
      savedModelVersions,
    ]
  );

  return <MonacoContext.Provider value={attributesMemo}>{children}</MonacoContext.Provider>;
}
