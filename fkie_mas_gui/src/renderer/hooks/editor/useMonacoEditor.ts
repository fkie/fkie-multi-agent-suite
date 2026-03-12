import { editor, IDisposable } from "monaco-editor";
import { useCallback, useEffect, useRef, useState } from "react";

import { LaunchIncludedFile, RosPackage } from "@/renderer/models";
import {
  configureContextMenu,
  configureMonacoEditor,
  registerLaunchDefinitionProvider,
  registerLaunchHoverProvider,
  registerLaunchLinkProvider,
} from "@/renderer/monaco/setup/configureMonacoEditor";
import { createIncludeResolver } from "@/renderer/monaco/setup/IncludeResolver";
import { providerIdFromTabId } from "../../monaco/utils";
import { useMonacoContext } from "../useMonacoContext";

type UseMonacoEditorOptions = {
  editorId: string;
  editorRef: React.MutableRefObject<editor.IStandaloneCodeEditor | undefined>;
  includedFiles: LaunchIncludedFile[];
  saveModel: (model: editor.ITextModel) => void;
};

export function useMonacoEditor({
  editorId,
  editorRef,
  includedFiles = [],
  saveModel = () => {},
}: UseMonacoEditorOptions) {
  const monacoCtx = useMonacoContext();

  const [activeModel, setActiveModel] = useState<editor.ITextModel | null>(null);
  const [activeModelDirty, setActiveModelDirty] = useState(false);
  const [modifiedFiles, setModifiedFiles] = useState<string[]>([]);

  const monacoViewStates = useRef(new Map<string, editor.ICodeEditorViewState | null>());

  const monacoDisposables = useRef<IDisposable[]>([]);
  const contextDisposables = useRef<IDisposable[]>([]);
  const linkDisposables = useRef<IDisposable[]>([]);

  // ---------------------------
  // setup context menu
  // ---------------------------

  const setupContextMenu = useCallback(() => {
    if (!monacoCtx.monaco || !editorRef.current) return;

    for (const d of contextDisposables.current) {
      d.dispose();
    }

    contextDisposables.current = configureContextMenu(monacoCtx.monaco, editorRef, saveModel);
  }, [monacoCtx.monaco, editorRef, saveModel]);

  // ---------------------------
  // setup monaco editor
  // ---------------------------

  const setupMonacoEditor = useCallback(
    (isRos2 = true, packages: RosPackage[] = []) => {
      if (!monacoCtx.monaco) return;

      for (const d of monacoDisposables.current) {
        d.dispose();
      }

      monacoDisposables.current = configureMonacoEditor(monacoCtx.monaco, editorId, isRos2, packages);
    },
    [editorId, monacoCtx.monaco]
  );

  // ---------------------------
  // set current model
  // ---------------------------

  const setCurrentModel = useCallback(
    (model: editor.ITextModel | null) => {
      const editorInstance = editorRef.current;
      if (!editorInstance) return;

      const currentModel = editorInstance.getModel();

      if (currentModel) {
        monacoViewStates.current.set(currentModel.uri.path, editorInstance.saveViewState());
      }

      editorInstance.setModel(model);

      if (model) {
        const viewState = monacoViewStates.current.get(model.uri.path);
        if (viewState) {
          editorInstance.restoreViewState(viewState);
        }

        const dirty = monacoCtx.isModifiedModel(model);
        setActiveModelDirty(dirty);
        setModifiedFiles((prev) => {
          if (dirty) {
            if (prev.includes(model.uri.path)) return prev;
            return [...prev, model.uri.path];
          }
          return prev.filter((m) => m !== model.uri.path);
        });
      }

      setActiveModel(model);
      editorInstance.focus();
    },
    [editorRef, monacoCtx]
  );

  // ---------------------------
  // dirty tracking
  // ---------------------------

  const handleDirtyChange = useCallback(
    (model: editor.ITextModel, dirty: boolean) => {
      if (activeModel === model) {
        setActiveModelDirty(dirty);
      }

      const registry = monacoCtx.modelRegistry()?.getByTab(editorId);
      if (!registry?.has(model)) return;

      setModifiedFiles((prev) => {
        if (dirty) {
          if (prev.includes(model.uri.path)) return prev;
          return [...prev, model.uri.path];
        }
        return prev.filter((m) => m !== model.uri.path);
      });
    },
    [activeModel, editorId, monacoCtx]
  );

  useEffect(() => {
    const dirtyManager = monacoCtx.dirtyManager?.();
    if (!dirtyManager) return;

    dirtyManager.onDirtyChange(editorId, handleDirtyChange);

    return () => {
      dirtyManager.removeDirtyListener(editorId);
    };
  }, [editorId, monacoCtx]);

  // ---------------------------
  // link provider (updated on changes of included files)
  // ---------------------------

  useEffect(() => {
    if (!monacoCtx.monaco) return;
    const providerId = providerIdFromTabId(editorId);
    if (!providerId) return;

    for (const d of linkDisposables.current) {
      d.dispose();
    }
    const resolver = createIncludeResolver(includedFiles);

    linkDisposables.current = [
      ...registerLaunchLinkProvider(monacoCtx.monaco, resolver, providerId),
      ...registerLaunchDefinitionProvider(monacoCtx, resolver, editorId),
      ...registerLaunchHoverProvider(monacoCtx.monaco, resolver),
    ];
  }, [activeModel, includedFiles]);

  // ---------------------------
  // dispose
  // ---------------------------

  const dispose = useCallback(() => {
    for (const d of monacoDisposables.current) {
      d.dispose();
    }
    for (const d of contextDisposables.current) {
      d.dispose();
    }
    for (const d of linkDisposables.current) {
      d.dispose();
    }

    monacoDisposables.current = [];
    contextDisposables.current = [];
    linkDisposables.current = [];
  }, []);

  return {
    activeModel,
    activeModelDirty,
    modifiedFiles,

    setCurrentModel,

    monacoViewStates: monacoViewStates.current,

    setupMonacoEditor,
    setupContextMenu,

    dispose,
  };
}
