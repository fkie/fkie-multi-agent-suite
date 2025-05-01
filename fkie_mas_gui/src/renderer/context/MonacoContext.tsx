/* eslint-disable max-classes-per-file */
import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";
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
import { LoggingContext } from "./LoggingContext";
import { RosContext } from "./RosContext";

export type TModelResult = {
  model: monaco.editor.ITextModel | null;
  file: FileItem | null;
  error: string;
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
}

export interface IMonacoProvider {
  children: React.ReactNode;
}
export const DEFAULT_MONACO = {
  monaco: null,
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
    } as TModelResult),
  createModel: (): monaco.editor.ITextModel | null => null,
};

export const MonacoContext = createContext<IMonacoContext>(DEFAULT_MONACO);

export function MonacoProvider({ children }: IMonacoProvider): ReturnType<React.FC<IMonacoProvider>> {
  const monaco = MonacoReact.useMonaco();
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);

  const [modifiedFiles, setModifiedFiles] = useState<ModifiedTabsInfo[]>([]);

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
        ],
      });
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

  const saveModifiedFilesOfTabId = useCallback(
    async (tabId: string): Promise<SaveResult[]> => {
      if (!monaco) return Promise.resolve([]);
      const result: SaveResult[] = [];
      const tabInfos: ModifiedTabsInfo[] = modifiedFiles.filter((item) => item.tabId === tabId);
      if (tabInfos.length === 0) return Promise.resolve([]);
      const tabInfo: ModifiedTabsInfo = tabInfos[0];
      await Promise.all(
        tabInfo.uriPaths.map(async (uriPath) => {
          const path = uriPath.split(":")[1];
          const saveItem: SaveResult = new SaveResult(tabId, path, false, tabInfo.providerId, "");
          const editorModel = monaco.editor.getModel(monaco.Uri.file(uriPath));
          if (editorModel) {
            const path = editorModel.uri.path.split(":")[1];
            // TODO change encoding if the file is encoded as HEX
            const fileToSave = new FileItem("", path, "", "", false, editorModel.getValue());
            const providerObj = rosCtx.getProviderById(tabInfo.providerId, false);
            if (providerObj) {
              const saveResult = await providerObj.saveFileContent(fileToSave);
              if (saveResult.bytesWritten > 0) {
                logCtx.success("Successfully saved file", `path: ${path}`);
                saveItem.result = true;
              } else {
                saveItem.message = `Error while save file ${path}: ${saveResult.error}`;
                logCtx.error(`Error while save file ${path}`, `${saveResult.error}`);
              }
            } else {
              saveItem.message = `Provider ${tabInfo.providerId} not found`;
              logCtx.error(`Provider ${tabInfo.providerId} not found`, `can not save file: ${path}`);
            }
          } else {
            saveItem.message = "Model not found";
          }
          result.push(saveItem);
        })
      );
      return Promise.resolve(result);
    },
    [logCtx, modifiedFiles, monaco, rosCtx]
  );

  function createUriPath(tabId: string, path: string): string {
    if (path.indexOf(":") !== -1) {
      return path;
    }
    return `/${tabId}:${path}`;
  }

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
  function createModel(tabId: string, file: FileItem): monaco.editor.ITextModel | null {
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
  }

  async function getModel(
    tabId: string,
    providerId: string,
    path: string,
    forceReload: boolean
  ): Promise<{ model: monaco.editor.ITextModel | null; file: FileItem | null; error: string }> {
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
            `Host: ${provider.host()}, root file: ${file.path}`
          );
        }
        return Promise.resolve({ model: model, file, error });
      }
      console.error(`Could not open included file: [${file.fileName}]: ${error}`);
    }
    return Promise.resolve({ model: model, file: null, error: "" });
  }

  const attributesMemo = useMemo(
    () => ({
      monaco,
      modifiedFiles,
      setModifiedFiles,
      getModifiedTabs,
      updateModifiedFiles,
      clearModifiedTabs,
      getModifiedFilesByTab,
      saveModifiedFilesOfTabId,
      createUriPath,
      getModel,
      createModel,
    }),
    [
      monaco,
      modifiedFiles,
      setModifiedFiles,
      getModifiedTabs,
      updateModifiedFiles,
      clearModifiedTabs,
      getModifiedFilesByTab,
      saveModifiedFilesOfTabId,
      createUriPath,
      getModel,
      createModel,
    ]
  );

  return <MonacoContext.Provider value={attributesMemo}>{children}</MonacoContext.Provider>;
}
