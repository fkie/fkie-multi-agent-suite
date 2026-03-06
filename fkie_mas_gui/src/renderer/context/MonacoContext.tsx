import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";
import { editor } from "monaco-editor/esm/vs/editor/editor.api";
import { createContext, useCallback, useContext, useEffect, useMemo, useRef } from "react";
import LoggingContext from "../context/LoggingContext";
import RosContext from "../context/RosContext";
import { FileItem, FileLanguageAssociations } from "../models";
import { initMonacoRuntime } from "../monaco/setup/configureMonaco";
import { SaveResult, TModelResult } from "../monaco/types";
import {
  createUriPath,
  createUriPathFromTab,
  fileFromUriPath,
  providerIdFromTabId,
  providerIdFromUriPath,
} from "../monaco/utils";
import { ModelRegistry } from "../monaco/workspace/ModelRegistry";
import { MonacoDirtyManager } from "../monaco/workspace/MonacoDirtyManager";
import { MonacoWorkspace } from "../monaco/workspace/MonacoWorkspace";
import { errorToMessage } from "../utils";

export interface IMonacoContext {
  monaco: MonacoReact.Monaco | null;
  dirtyManager: () => MonacoDirtyManager | undefined;
  modelRegistry: () => ModelRegistry | undefined;
  getModel: (tabId: string, providerId: string, path: string, forceReload?: boolean) => Promise<TModelResult>;
  createModel: (tabId: string, file: FileItem) => editor.ITextModel | null;
  saveFile: (model: editor.ITextModel) => Promise<SaveResult>;
  saveModifiedFilesOfTabId: (tabId: string) => Promise<SaveResult[]>;
  closeTabs: (tabIds?: string[]) => void;
  getModifiedFilesByTab: (tabId: string) => editor.ITextModel[];
  isModifiedModel: (model: editor.ITextModel) => boolean;
}

export const MonacoContext = createContext<IMonacoContext | null>(null);

// -------------------- MonacoProvider --------------------
export function MonacoProvider({ children }: { children: React.ReactNode }) {
  const monacoInstance = MonacoReact.useMonaco();
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const rosCtxRef = useRef(rosCtx);
  const logCtxRef = useRef(logCtx);

  // -------------------- Services --------------------
  const workspaceRef = useRef<MonacoWorkspace | null>(null);

  useEffect(() => {
    if (!monacoInstance) return;

    if (!workspaceRef.current) {
      initMonacoRuntime(monacoInstance);
      workspaceRef.current = new MonacoWorkspace(monacoInstance, rosCtxRef);
    }
  }, [monacoInstance]);

  useEffect(() => {
    rosCtxRef.current = rosCtx;
  }, [rosCtx]);

  useEffect(() => {
    logCtxRef.current = logCtx;
  }, [logCtx]);

  const dirtyManager = useCallback(() => {
    return workspaceRef.current?.dirty;
  }, []);

  const modelRegistry = useCallback(() => {
    return workspaceRef.current?.models;
  }, []);

  const closeTabs = useCallback((tabIds?: string[]): void => {
    if (tabIds === undefined) {
      workspaceRef.current?.models.closeAllModels();
      return;
    }
    for (const tabId of tabIds || []) {
      workspaceRef.current?.models.closeModelsByTabId(tabId);
    }
  }, []);

  const getModifiedFilesByTab = useCallback((tabId: string) => {
    if (!workspaceRef.current) {
      return [];
    }
    const dirtyModels: editor.ITextModel[] = workspaceRef.current.dirty.getDirtyModels();
    const tabModels = workspaceRef.current.models.getByTab(tabId);

    return dirtyModels.filter((m) => tabModels.has(m));
  }, []);

  const saveFile = useCallback(async (model: editor.ITextModel): Promise<SaveResult> => {
    const uriPath = model.uri.path;

    if (!workspaceRef.current) {
      return { uriPath, result: false, message: "Monaco not initialized yet" };
    }

    const tabIds: string[] = workspaceRef.current.models.getTabsByModels([model]);

    const providerId = providerIdFromUriPath(uriPath);
    if (!providerId) {
      return { tabIds, uriPath, result: false, message: `Invalid uri path ${uriPath}: no provider id found` };
    }

    try {
      // workspaceRef.current.saver.save sollte bereits korrekt typisiert sein:
      const result: { result: boolean; message: string } = await workspaceRef.current.saver.save(model, providerId);

      return { tabIds, uriPath, result: result.result, message: result.message };
    } catch (error) {
      return { tabIds, uriPath, result: false, message: errorToMessage(error, `Unknown error during save: ${error}`) };
    }
  }, []);

  const saveModifiedFilesOfTabId = useCallback(
    async (tabId: string): Promise<SaveResult[]> => {
      if (!workspaceRef.current) {
        return [{ uriPath: "", result: false, message: "Monaco not initialized yet" }];
      }

      const modelsToSave = getModifiedFilesByTab(tabId);
      if (modelsToSave.length === 0) return [];
      const result = await Promise.allSettled(modelsToSave.map((model) => saveFile(model)));
      return result.map((r, i) =>
        r.status === "fulfilled"
          ? r.value
          : { uriPath: modelsToSave[i].uri.path, result: false, message: r.reason?.toString() }
      );
    },
    [saveFile, getModifiedFilesByTab]
  );

  const isModifiedModel = useCallback((model: editor.ITextModel): boolean => {
    if (!workspaceRef.current) return false;
    return workspaceRef.current?.dirty.isDirty(model);
  }, []);

  /**
   * Create a new monaco model from a given file
   * @param file - Original file
   */
  const createModel = useCallback((tabId: string, file: FileItem): monaco.editor.ITextModel | null => {
    if (!workspaceRef.current) {
      return null;
    }
    const uriPath = createUriPathFromTab(tabId, file.path);
    const model = workspaceRef.current.models.get(uriPath);
    // create monaco model, if it does not exists yet
    if (model) {
      // if (model.modified) {
      //   // TODO: ask the user how to proceed
      //   return model;
      // }
      model.dispose();
    }
    const newModel = workspaceRef.current.models.create(
      tabId,
      uriPath,
      file.value,
      FileLanguageAssociations[file.extension] ?? "plaintext"
    );
    return newModel;
  }, []);

  const openFile = useCallback(async (tabId: string, path: string) => {
    if (!workspaceRef.current) {
      return { file: null, error: "Monaco not initialized yet" };
    }
    const providerId = providerIdFromTabId(tabId);
    if (!providerId) {
      return { file: null, error: `Invalid tabId ${tabId}: no provider id found` };
    }
    return workspaceRef.current.opener.open(providerId, path);
  }, []);

  const getModel = useCallback(
    async (tabId: string, providerId: string, path: string, forceReload = false): Promise<TModelResult> => {
      if (!workspaceRef.current) {
        return { model: null, file: null, error: "Monaco not initialized yet" };
      }
      const uriPath = createUriPath(providerId, path);
      // 1. Check if model already exists
      let model = workspaceRef.current.models.get(uriPath);

      // 2. If no model or forceReload, load file from provider
      if (!model || forceReload) {
        const filePath = fileFromUriPath(path);
        const { file, error } = await openFile(tabId, filePath).catch((error) => {
          return { model: null, file: null, error: error?.message ?? "Unknown error while opening file" };
        });

        if (!error && file) {
          model = workspaceRef.current.models.create(
            tabId,
            uriPath,
            file.value,
            FileLanguageAssociations[file.extension] ?? "plaintext"
          );
          if (!model) {
            logCtx.error(
              `Could not create model for included file: [${file.fileName}]`,
              `Host: ${providerId}, root file: ${file.path}`,
              "file model error"
            );
          }
          return { model, file, error: null };
        }

        return { model: null, file: null, error: error || "Failed to load file" };
      }

      // 3. Model exists and no reload needed
      return { model, file: null, error: null };
    },
    [logCtx, openFile]
  );

  // -------------------- Context Value --------------------
  const contextValue: IMonacoContext = useMemo(
    () => ({
      monaco: monacoInstance,
      dirtyManager,
      modelRegistry,
      getModel,
      createModel,
      saveFile,
      saveModifiedFilesOfTabId,
      closeTabs,
      getModifiedFilesByTab,
      isModifiedModel,
    }),
    [
      monacoInstance,
      dirtyManager,
      modelRegistry,
      getModel,
      createModel,
      saveFile,
      saveModifiedFilesOfTabId,
      closeTabs,
      getModifiedFilesByTab,
      isModifiedModel,
    ]
  );

  return <MonacoContext.Provider value={contextValue}>{children}</MonacoContext.Provider>;
}
