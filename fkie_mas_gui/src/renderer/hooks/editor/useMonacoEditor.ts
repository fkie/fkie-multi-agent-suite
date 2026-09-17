import { editor, IDisposable } from "monaco-editor";
import { useCallback, useEffect, useRef, useState } from "react";

import { configureContextMenu, configureMonacoEditor } from "@/renderer/monaco/setup/configureMonacoEditor";
import { useMonacoContext } from "../useMonacoContext";

type UseMonacoEditorOptions = {
  editorId: string;
  editorRef: React.MutableRefObject<editor.IStandaloneCodeEditor | undefined>;
  saveModel: (model: editor.ITextModel) => void;
};

export function useMonacoEditor({ editorId, editorRef, saveModel = () => {} }: UseMonacoEditorOptions) {
  const monacoCtx = useMonacoContext();

  // read the manager as a value - the effect below must re-run once it exists
  const dirtyManager = monacoCtx.dirtyManager();

  const [activeModel, setActiveModel] = useState<editor.ITextModel | null>(null);
  const [activeModelDirty, setActiveModelDirty] = useState(false);
  const [initialized, setInitialized] = useState(false);
  const [initializedContextMenu, setInitializedContextMenu] = useState(false);
  const [initializedMonacoEditor, setInitializedMonacoEditor] = useState(false);
  const [modifiedFiles, setModifiedFiles] = useState<string[]>([]);

  const monacoViewStates = useRef(new Map<string, editor.ICodeEditorViewState | null>());

  const monacoDisposables = useRef<IDisposable[]>([]);
  const contextDisposables = useRef<IDisposable[]>([]);

  const activeModelRef = useRef<editor.ITextModel | null>(null);

  useEffect(() => {
    activeModelRef.current = activeModel;
  }, [activeModel]);

  // ---------------------------
  // setup context menu
  // ---------------------------

  const setupContextMenu = useCallback(() => {
    if (!monacoCtx.monaco || !editorRef.current) return;

    for (const d of contextDisposables.current) {
      d.dispose();
    }

    contextDisposables.current = configureContextMenu(monacoCtx.monaco, editorRef, saveModel);
    setInitializedContextMenu(true);
  }, [monacoCtx.monaco, editorRef, saveModel]);

  // ---------------------------
  // setup monaco editor
  // ---------------------------

  const setupMonacoEditor = useCallback(() => {
    if (!monacoCtx.monaco) return;

    for (const d of monacoDisposables.current) {
      d.dispose();
    }

    monacoDisposables.current = configureMonacoEditor(monacoCtx.monaco, editorId);
    setInitializedMonacoEditor(true);
  }, [editorId, monacoCtx.monaco]);

  useEffect(() => {
    setupContextMenu();
  }, [setupContextMenu]);

  useEffect(() => {
    setupMonacoEditor();
  }, [setupMonacoEditor]);

  useEffect(() => {
    setInitialized(initializedContextMenu && initializedMonacoEditor);
  }, [initializedContextMenu, initializedMonacoEditor]);

  // ---------------------------
  // set current model
  // ---------------------------

  const setCurrentModel = useCallback(
    (model: editor.ITextModel | null) => {
      const editorInstance = editorRef.current;
      if (!editorInstance) return;

      const currentModel = editorInstance.getModel();
      if (currentModel && !currentModel.isDisposed()) {
        monacoViewStates.current.set(currentModel.uri.path, editorInstance.saveViewState());
      }

      // keep the ref in sync synchronously - dirty events can arrive before react commits
      activeModelRef.current = model;
      editorInstance.setModel(model);

      if (model) {
        const viewState = monacoViewStates.current.get(model.uri.path);
        if (viewState) editorInstance.restoreViewState(viewState);

        const dirty = monacoCtx.isModifiedModel(model);
        setActiveModelDirty(dirty);
        setModifiedFiles((prev) => {
          if (dirty) return prev.includes(model.uri.path) ? prev : [...prev, model.uri.path];
          return prev.filter((m) => m !== model.uri.path);
        });
      } else {
        setActiveModelDirty(false);
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
      // fall back to the editor model - the ref may not be assigned yet on first load
      const current = activeModelRef.current ?? editorRef.current?.getModel() ?? null;
      if (current?.uri.path === model?.uri.path) {
        setActiveModelDirty(dirty);
      }

      const registry = monacoCtx.modelRegistry()?.getByEditor(editorId);
      if (!registry?.has(model)) return;

      setModifiedFiles((prev) => {
        if (dirty) return prev.includes(model.uri.path) ? prev : [...prev, model.uri.path];
        return prev.filter((m) => m !== model.uri.path);
      });
    },
    [editorId, editorRef, monacoCtx]
  );

  useEffect(() => {
    if (!dirtyManager) return;
    dirtyManager.onDirtyChange(editorId, handleDirtyChange);

    // events emitted before this listener existed are lost - sync once
    const model = editorRef.current?.getModel();
    if (model && !model.isDisposed()) {
      handleDirtyChange(model, dirtyManager.refresh(model));
    }

    return () => {
      dirtyManager.removeDirtyListener(editorId);
    };
  }, [editorId, dirtyManager, handleDirtyChange]);

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

    monacoDisposables.current = [];
    contextDisposables.current = [];
  }, []);

  useEffect(() => {
    return (): void => {
      dispose();
    };
  }, []);

  return {
    initialized,
    activeModel,
    activeModelDirty,
    modifiedFiles,

    setCurrentModel,

    monacoViewStates: monacoViewStates.current,
  };
}
