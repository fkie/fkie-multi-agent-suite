import { editor, IDisposable } from "monaco-editor";
import { useCallback, useEffect, useRef, useState } from "react";

import { LaunchIncludedFile } from "@/renderer/models";
import { configureContextMenu, configureMonacoEditor } from "@/renderer/monaco/setup/configureMonacoEditor";
import { providerIdFromEditorId } from "../../monaco/utils";
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
  }, [editorId, monacoCtx.monaco]);

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
      if (activeModelRef.current?.uri.path === model?.uri.path) {
        setActiveModelDirty(dirty);
      }

      const registry = monacoCtx.modelRegistry()?.getByEditor(editorId);
      if (!registry?.has(model)) return;

      setModifiedFiles((prev) => {
        if (dirty) {
          if (prev.includes(model.uri.path)) return prev;
          return [...prev, model.uri.path];
        }
        return prev.filter((m) => m !== model.uri.path);
      });
    },
    [editorId, monacoCtx]
  );

  useEffect(() => {
    const dirtyManager = monacoCtx.dirtyManager();
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
    const providerId = providerIdFromEditorId(editorId);
    if (!providerId) return;

    monacoCtx.updateResolver(editorId, includedFiles);
  }, [monacoCtx, editorId, includedFiles]);

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
