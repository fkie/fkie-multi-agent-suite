import * as MonacoReact from "@monaco-editor/react";
import { editor, IDisposable, Uri } from "monaco-editor";
import { emitCustomEvent } from "react-custom-events";

import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "@/renderer/pages/NodeManager/layout/events";
import { fileFromUriPath } from "../utils";

export function configureContextMenu(
  m: MonacoReact.Monaco,
  editorRef: React.MutableRefObject<editor.IStandaloneCodeEditor | undefined>,
  saveModel: (model: editor.ITextModel) => void
): IDisposable[] {
  if (!m) return [];
  if (!editorRef.current) return [];
  const newDisposables: IDisposable[] = [];
  newDisposables.push(
    editorRef.current?.addAction({
      id: "save_action",
      label: "Save",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyCode.KeyS],
      precondition: undefined,
      keybindingContext: undefined,
      contextMenuGroupId: "1_modification",
      contextMenuOrder: 1.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        const model = editorInstance.getModel();
        if (model) {
          saveModel(model);
        }
      },
    } as editor.IActionDescriptor)
  );
  newDisposables.push(
    editorRef.current.addAction({
      id: "toggle line comment",
      label: "Toggle line comment",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyMod.Shift | m.KeyCode.Digit7],
      precondition: undefined,
      keybindingContext: undefined,
      contextMenuGroupId: "1_modification",
      contextMenuOrder: 2.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        editorInstance.trigger("toggle line comment", "editor.action.commentLine", {});
      },
    })
  );
  newDisposables.push(
    editorRef.current.addAction({
      id: "command_palette",
      label: "Command Palette",
      keybindings: [m.KeyMod.CtrlCmd | m.KeyMod.Shift | m.KeyCode.KeyP],
      precondition: undefined,
      keybindingContext: undefined,
      // contextMenuGroupId: "none",
      // contextMenuOrder: 1.0,
      run: async (editorInstance: editor.ICodeEditor) => {
        editorInstance.trigger("open command palette", "editor.action.quickCommand", {});
      },
    })
  );
  return newDisposables;
}

export function configureMonacoEditor(m: MonacoReact.Monaco, editorId: string): IDisposable[] {
  if (!m) return [];
  const newDisposables: IDisposable[] = [];
  newDisposables.push(
    m.editor.registerLinkOpener({
      open(resource: Uri): boolean | Promise<boolean> {
        emitCustomEvent(
          EVENT_EDITOR_SELECT_RANGE,
          eventEditorSelectRange(editorId, fileFromUriPath(resource.path), null)
        );
        return true;
      },
    })
  );

  return newDisposables;
}
