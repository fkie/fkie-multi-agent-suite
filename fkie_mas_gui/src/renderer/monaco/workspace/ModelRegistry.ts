import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";

export class ModelRegistry {
  private models = new Map<string, monaco.editor.ITextModel>();
  private tabIndex = new Map<string, Set<monaco.editor.ITextModel>>();
  private uriIndex = new Map<monaco.editor.ITextModel, Set<string>>();

  constructor(private monaco: MonacoReact.Monaco) {
    this.monaco.editor.onWillDisposeModel((m) => {
      this.removeFromIndex(this.tabIndex, m);
      this.uriIndex.delete(m);
      for (const [uri, model] of this.models) {
        if (model === m) {
          this.models.delete(uri);
          break;
        }
      }
    });
  }

  get(uri: string): monaco.editor.ITextModel | undefined {
    const model = this.models.get(uri);
    if (model?.isDisposed()) {
      this.models.delete(uri);
      return undefined;
    }
    return model;
  }

  create(tabId: string, uri: string, content: string, language: string): monaco.editor.ITextModel {
    let model = this.get(uri);

    if (!model) {
      model = this.monaco.editor.createModel(content, language, this.monaco.Uri.file(uri));
      this.models.set(uri, model);
    }
    this.addToIndex(this.tabIndex, tabId, model);
    this.addToUriIndex(this.uriIndex, model, tabId);

    return model;
  }

  getByTab(tabId: string): Set<monaco.editor.ITextModel> {
    return this.tabIndex.get(tabId) || new Set<monaco.editor.ITextModel>();
  }

  getTabsByModels(models: monaco.editor.ITextModel[]): string[] {
    // collect all Tab-IDs, remove duplicates
    const result = new Set(models.flatMap((model) => Array.from(this.uriIndex.get(model) || [])));

    return Array.from(result);
  }

  closeAllModels() {
    const models = this.monaco.editor.getModels();

    for (const model of models) {
      model.dispose();
    }
    this.tabIndex.clear();
    this.uriIndex.clear();
  }

  closeModelsByTabId(tabId: string) {
    const models = this.tabIndex.get(tabId);
    if (!models) return;

    for (const model of models) {
      const tabs = this.uriIndex.get(model);
      if (tabs) {
        tabs.delete(tabId);
        if (tabs.size === 0) {
          model.dispose();
          this.uriIndex.delete(model);
        }
      }
    }
    this.tabIndex.delete(tabId);
    this.removeFromUriIndex(this.uriIndex, tabId);
  }

  /**
   * Utility to add a model to an index structure.
   */
  private addToIndex(
    index: Map<string, Set<monaco.editor.ITextModel>>,
    tabId: string,
    model: monaco.editor.ITextModel
  ) {
    let set = index.get(tabId);

    if (!set) {
      set = new Set();
      index.set(tabId, set);
    }

    set.add(model);
  }

  /**
   * Utility to remove a model from an index structure.
   */
  private removeFromIndex(index: Map<string, Set<monaco.editor.ITextModel>>, model: monaco.editor.ITextModel) {
    for (const [key, set] of index.entries()) {
      if (!set) continue;

      set.delete(model);

      if (set.size === 0) {
        index.delete(key);
      }
    }
  }

  /**
   * Utility to add a tab id to an index structure.
   */
  private addToUriIndex(
    index: Map<monaco.editor.ITextModel, Set<string>>,
    model: monaco.editor.ITextModel,
    tabId: string
  ) {
    let set = index.get(model);

    if (!set) {
      set = new Set();
      index.set(model, set);
    }

    set.add(tabId);
  }

  /**
   * Utility to remove a tab id from an index structure.
   */
  private removeFromUriIndex(index: Map<monaco.editor.ITextModel, Set<string>>, tabId: string) {
    for (const [key, set] of index.entries()) {
      if (!set) continue;

      set.delete(tabId);

      if (set.size === 0) {
        index.delete(key);
      }
    }
  }
}
