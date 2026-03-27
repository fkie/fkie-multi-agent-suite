import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";
import { editor } from "monaco-editor";
import { createContext, useCallback, useEffect, useMemo, useRef } from "react";

import { useAlwaysCurrentRef } from "../hooks/useAlwaysCurrentRef";
import { IncludeResolver } from "../hooks/useIncludedFiles";
import { useLoggingContext } from "../hooks/useLoggingContext";
import { useRosContext } from "../hooks/useRosContext";
import { FileItem, FileLanguageAssociations } from "../models";
import { SaveResult, TModelResult } from "../monaco/types";
import {
  createUriPath,
  createUriPathFromEditorId,
  fileFromUriPath,
  providerIdFromEditorId,
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
  getModel: (editorId: string, path: string, forceReload?: boolean) => Promise<TModelResult>;
  getModels: (editorId: string) => Set<editor.ITextModel>;
  createModel: (editorId: string, file: FileItem) => editor.ITextModel | null;
  saveFile: (model: editor.ITextModel) => Promise<SaveResult>;
  saveModifiedFilesOfEditorId: (editorId: string) => Promise<SaveResult[]>;
  closeEditors: (editorIds?: string[]) => void;
  getModifiedFilesByEditor: (editorId: string) => editor.ITextModel[];
  isModifiedModel: (model: editor.ITextModel) => boolean;
  isReadOnly: (model: editor.ITextModel) => boolean;
  isInstallPath: (model: editor.ITextModel) => boolean;
  setResolver: (editorId: string, resolver: IncludeResolver) => void;
  removeResolver: (editorId: string) => void;
  getResolver: (editorId: string) => IncludeResolver | undefined;
}

export const MonacoContext = createContext<IMonacoContext | null>(null);

// -------------------- MonacoProvider --------------------
export function MonacoProvider({ children }: { children: React.ReactNode }) {
  const monacoInstance = MonacoReact.useMonaco();
  const rosCtx = useRosContext();
  const logCtx = useLoggingContext();
  const rosCtxRef = useAlwaysCurrentRef(rosCtx);

  const files = useRef(new Map<string, FileItem>());
  const resolvers = useRef(new Map<string, IncludeResolver>());

  // -------------------- Services --------------------
  const workspaceRef = useRef<MonacoWorkspace | null>(null);

  useEffect(() => {
    if (!monacoInstance) return;

    if (!workspaceRef.current) {
      workspaceRef.current = new MonacoWorkspace(monacoInstance, rosCtxRef);
    }
  }, [monacoInstance]);

  const dirtyManager = useCallback(() => {
    return workspaceRef.current?.dirty;
  }, []);

  const modelRegistry = useCallback(() => {
    return workspaceRef.current?.models;
  }, []);

  const closeEditors = useCallback((editorIds?: string[]): void => {
    if (editorIds === undefined) {
      workspaceRef.current?.models.closeAllModels();
      return;
    }
    for (const editorId of editorIds || []) {
      workspaceRef.current?.models.closeModelsByEditorId(editorId);
      resolvers.current.delete(editorId);
    }
  }, []);

  const getModifiedFilesByEditor = useCallback((editorId: string) => {
    if (!workspaceRef.current) {
      return [];
    }
    const dirtyModels: editor.ITextModel[] = workspaceRef.current.dirty.getDirtyModels();
    const editorModels = workspaceRef.current.models.getByEditor(editorId);

    return dirtyModels.filter((m) => editorModels.has(m));
  }, []);

  const saveFile = useCallback(async (model: editor.ITextModel): Promise<SaveResult> => {
    const uriPath = model.uri.path;

    if (!workspaceRef.current) {
      return { uriPath, result: false, message: "Monaco not initialized yet" };
    }

    const editorIds: string[] = workspaceRef.current.models.getEditorsByModels([model]);

    const providerId = providerIdFromUriPath(uriPath);
    if (!providerId) {
      return { editorIds, uriPath, result: false, message: `Invalid uri path ${uriPath}: no provider id found` };
    }

    try {
      // workspaceRef.current.saver.save sollte bereits korrekt typisiert sein:
      const result: { result: boolean; message: string } = await workspaceRef.current.saver.save(model, providerId);

      return { editorIds, uriPath, result: result.result, message: result.message };
    } catch (error) {
      return {
        editorIds,
        uriPath,
        result: false,
        message: errorToMessage(error, `Unknown error during save: ${error}`),
      };
    }
  }, []);

  const saveModifiedFilesOfEditorId = useCallback(
    async (editorId: string): Promise<SaveResult[]> => {
      if (!workspaceRef.current) {
        return [{ uriPath: "", result: false, message: "Monaco not initialized yet" }];
      }

      const modelsToSave = getModifiedFilesByEditor(editorId);
      if (modelsToSave.length === 0) return [];
      const result = await Promise.allSettled(modelsToSave.map((model) => saveFile(model)));
      return result.map((r, i) =>
        r.status === "fulfilled"
          ? r.value
          : { uriPath: modelsToSave[i].uri.path, result: false, message: r.reason?.toString() }
      );
    },
    [saveFile, getModifiedFilesByEditor]
  );

  const isModifiedModel = useCallback((model: editor.ITextModel): boolean => {
    if (!workspaceRef.current) return false;
    return workspaceRef.current?.dirty.isDirty(model);
  }, []);

  /**
   * Create a new monaco model from a given file
   * @param file - Original file
   */
  const createModel = useCallback((editorId: string, file: FileItem): monaco.editor.ITextModel | null => {
    if (!workspaceRef.current) {
      return null;
    }
    const uriPath = createUriPathFromEditorId(editorId, file.path);
    const model = workspaceRef.current.models.get(uriPath);
    // create monaco model, if it does not exists yet
    if (model) {
      if (workspaceRef.current?.dirty.isDirty(model)) {
        return model;
      }
      model.dispose();
    }
    const newModel = workspaceRef.current.models.create(
      editorId,
      uriPath,
      file.value,
      FileLanguageAssociations[file.extension] ?? "plaintext"
    );
    return newModel;
  }, []);

  const openFile = useCallback(async (providerId: string, path: string) => {
    if (!workspaceRef.current) {
      return { file: null, error: "Monaco not initialized yet" };
    }
    return workspaceRef.current.opener.open(providerId, path);
  }, []);

  const getModel = useCallback(
    async (editorId: string, path: string, forceReload = false): Promise<TModelResult> => {
      if (!workspaceRef.current) {
        return { model: null, file: null, error: "Monaco not initialized yet" };
      }
      const providerId = providerIdFromEditorId(editorId);
      if (!providerId) {
        return { model: null, file: null, error: `Invalid editorId ${editorId}: no provider id found` };
      }
      const uriPath = createUriPath(providerId, path);
      // 1. Check if model already exists
      let model = workspaceRef.current.models.get(uriPath);

      // 2. If no model or forceReload, load file from provider
      if (!model || forceReload) {
        const filePath = fileFromUriPath(path);
        const { file, error } = await openFile(providerId, filePath).catch((error) => {
          return { model: null, file: null, error: error?.message ?? "Unknown error while opening file" };
        });

        if (!error && file) {
          // remove dirty flag for current model
          if (model) workspaceRef.current?.dirty.markSaved(model);
          model = workspaceRef.current.models.create(
            editorId,
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
            files.current.delete(uriPath);
          }
          files.current.set(uriPath, file);
          return { model, file, error: null };
        }

        return { model: null, file: null, error: error || "Failed to load file" };
      }
      // Ensure that the model is also entered in the registry for the editor
      workspaceRef.current.models.updateRegistry(editorId, model);

      // 3. Model exists and no reload needed
      return { model, file: null, error: null };
    },
    [logCtx, openFile]
  );

  const getModels: (editorId: string) => Set<editor.ITextModel> = (editorId) => {
    return workspaceRef.current?.models.getByEditor(editorId) || new Set();
  };

  const isReadOnly: (model: editor.ITextModel) => boolean = (model) => {
    return files.current.get(model.uri.path)?.readonly || false;
  };

  const isInstallPath: (model: editor.ITextModel) => boolean = (model) => {
    const file = files.current.get(model.uri.path);
    if (!file) return false;
    const filePath = file.realpath && file.realpath.length > 0 ? file.realpath : file.path;
    return filePath.search("/install/") !== -1;
  };

  const setResolver: (editorId: string, resolver: IncludeResolver) => void = (editorId, resolver) => {
    resolvers.current.set(editorId, resolver);
  };

  const removeResolver: (editorId: string) => void = (editorId) => {
    resolvers.current.delete(editorId);
  };

  const getResolver: (editorId: string) => IncludeResolver | undefined = (editorId) => {
    return resolvers.current.get(editorId);
  };

  // -------------------- Context Value --------------------
  const contextValue: IMonacoContext = useMemo(
    () => ({
      monaco: monacoInstance,
      dirtyManager,
      modelRegistry,
      getModel,
      getModels,
      createModel,
      saveFile,
      saveModifiedFilesOfEditorId,
      closeEditors,
      getModifiedFilesByEditor,
      isModifiedModel,
      isReadOnly,
      isInstallPath,
      setResolver,
      removeResolver,
      getResolver,
    }),
    [
      monacoInstance,
      dirtyManager,
      modelRegistry,
      getModel,
      createModel,
      saveFile,
      saveModifiedFilesOfEditorId,
      closeEditors,
      getModifiedFilesByEditor,
      isModifiedModel,
    ]
  );

  return <MonacoContext.Provider value={contextValue}>{children}</MonacoContext.Provider>;
}
