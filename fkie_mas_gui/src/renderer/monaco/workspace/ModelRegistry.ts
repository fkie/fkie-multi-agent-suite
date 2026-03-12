import * as MonacoReact from "@monaco-editor/react";
import { editor } from "monaco-editor";

export class ModelRegistry {
  private models = new Map<string, editor.ITextModel>();
  private editorIndex = new Map<string, Set<editor.ITextModel>>();
  private uriIndex = new Map<editor.ITextModel, Set<string>>();

  constructor(private monaco: MonacoReact.Monaco) {
    this.monaco.editor.onWillDisposeModel((m) => {
      this.removeFromIndex(this.editorIndex, m);
      this.uriIndex.delete(m);
      for (const [uri, model] of this.models) {
        if (model === m) {
          this.models.delete(uri);
          break;
        }
      }
    });
  }

  get(uri: string): editor.ITextModel | undefined {
    const model = this.models.get(uri);
    if (model?.isDisposed()) {
      this.models.delete(uri);
      return undefined;
    }
    return model;
  }

  updateRegistry(editorId: string, model: editor.ITextModel): void {
    this.addToIndex(this.editorIndex, editorId, model);
    this.addToUriIndex(this.uriIndex, model, editorId);
  }

  create(editorId: string, uri: string, content: string, language: string): editor.ITextModel {
    let model = this.get(uri);

    if (!model) {
      model = this.monaco.editor.createModel(content, language, this.monaco.Uri.file(uri));
      this.models.set(uri, model);
    }
    this.updateRegistry(editorId, model);

    return model;
  }

  getByEditor(editorId: string): Set<editor.ITextModel> {
    return this.editorIndex.get(editorId) || new Set<editor.ITextModel>();
  }

  getByEditorIds(editorIds: string[]): Set<editor.ITextModel> {
    const result = new Set<editor.ITextModel>();

    for (const editorId of editorIds) {
      for (const model of this.getByEditor(editorId)) {
        result.add(model);
      }
    }

    return result;
  }

  getEditorsByModels(models: editor.ITextModel[]): string[] {
    // collect all Editor-IDs, remove duplicates
    const result = new Set(models.flatMap((model) => Array.from(this.uriIndex.get(model) || [])));

    return Array.from(result);
  }

  closeAllModels() {
    const models = this.monaco.editor.getModels();

    for (const model of models) {
      model.dispose();
    }
    this.editorIndex.clear();
    this.uriIndex.clear();
  }

  closeModelsByEditorId(editorId: string) {
    const models = this.editorIndex.get(editorId);
    if (!models) return;

    for (const model of models) {
      const editors = this.uriIndex.get(model);
      if (editors) {
        editors.delete(editorId);
        if (editors.size === 0) {
          model.dispose();
          this.uriIndex.delete(model);
        }
      }
    }
    this.editorIndex.delete(editorId);
    this.removeFromUriIndex(this.uriIndex, editorId);
  }

  /**
   * Utility to add a model to an index structure.
   */
  private addToIndex(index: Map<string, Set<editor.ITextModel>>, editorId: string, model: editor.ITextModel) {
    let set = index.get(editorId);

    if (!set) {
      set = new Set();
      index.set(editorId, set);
    }

    set.add(model);
  }

  /**
   * Utility to remove a model from an index structure.
   */
  private removeFromIndex(index: Map<string, Set<editor.ITextModel>>, model: editor.ITextModel) {
    for (const [key, set] of index.entries()) {
      if (!set) continue;

      set.delete(model);

      if (set.size === 0) {
        index.delete(key);
      }
    }
  }

  /**
   * Utility to add a editor id to an index structure.
   */
  private addToUriIndex(index: Map<editor.ITextModel, Set<string>>, model: editor.ITextModel, editorId: string) {
    let set = index.get(model);

    if (!set) {
      set = new Set();
      index.set(model, set);
    }

    set.add(editorId);
  }

  /**
   * Utility to remove a editor id from an index structure.
   */
  private removeFromUriIndex(index: Map<editor.ITextModel, Set<string>>, editorId: string) {
    for (const [key, set] of index.entries()) {
      if (!set) continue;
      set.delete(editorId);

      if (set.size === 0) {
        index.delete(key);
      }
    }
  }
}
