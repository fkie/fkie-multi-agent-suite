import * as MonacoReact from "@monaco-editor/react";
import * as monaco from "monaco-editor";
import { DirtyChangeListener } from "../types";

/**
 * High performance dirty state manager for Monaco models.
 * This design scales well for large editors (hundreds or thousands of files).
 */
export class MonacoDirtyManager {
  /**
   * Stores the saved version for each model.
   *
   * We use `alternativeVersionId` because it returns to a previous value
   * when the content becomes identical again (e.g. via undo).
   *
   * WeakMap ensures models can be garbage collected automatically.
   */
  private savedVersions = new WeakMap<monaco.editor.ITextModel, number>();

  /**
   * Global set of currently dirty models.
   */
  private dirtyModels = new Set<monaco.editor.ITextModel>();

  /**
   * Keeps track of content change subscriptions per model.
   *
   * Stored in a WeakMap so listeners disappear automatically
   * when the model is garbage collected.
   */
  private disposables = new WeakMap<monaco.editor.ITextModel, monaco.IDisposable>();

  /**
   * Subscribers that want to react to dirty state changes.
   * Typically UI components (tab bar, save indicators, etc.).
   */
  private listeners = new Map<string, DirtyChangeListener>();

  constructor(private monaco: MonacoReact.Monaco) {
    // existing models
    for (const model of monaco.editor.getModels()) {
      this.trackModel(model);
    }
    this.monaco.editor.onDidCreateModel((m) => this.trackModel(m));
    this.monaco.editor.onWillDisposeModel((m) => this.untrackModel(m));
  }

  private trackModel(model: monaco.editor.ITextModel) {
    if (this.disposables.has(model)) return;

    this.savedVersions.set(model, model.getAlternativeVersionId());

    const disposable = model.onDidChangeContent(() => {
      this.updateDirtyState(model);
    });

    this.disposables.set(model, disposable);
  }

  /**
   * Stop tracking a model and clean up all references.
   */
  private untrackModel(model: monaco.editor.ITextModel) {
    this.disposables.get(model)?.dispose();
    this.disposables.delete(model);

    this.savedVersions.delete(model);
    this.dirtyModels.delete(model);
  }

  /**
   * Recalculate the dirty state of a model.
   *
   * A model is considered dirty when the current
   * alternativeVersionId differs from the saved version.
   */
  private updateDirtyState(model: monaco.editor.ITextModel) {
    const saved = this.savedVersions.get(model);
    const current = model.getAlternativeVersionId();

    const wasDirty = this.dirtyModels.has(model);
    const isDirty = saved !== current;

    if (isDirty) {
      this.dirtyModels.add(model);
    } else {
      this.dirtyModels.delete(model);
    }

    /**
     * Only emit events if the dirty state actually changed.
     */
    if (wasDirty !== isDirty) {
      this.emitDirtyChange(model, isDirty);
    }
  }

  /**
   * Mark a model as saved.
   *
   * This updates the saved version reference
   * and clears the dirty state.
   */
  markSaved(model: monaco.editor.ITextModel) {
    this.savedVersions.set(model, model.getAlternativeVersionId());

    const wasDirty = this.dirtyModels.delete(model);

    if (wasDirty) {
      this.emitDirtyChange(model, false);
    }
  }

  isDirty(model: monaco.editor.ITextModel) {
    return this.dirtyModels.has(model);
  }

  getDirtyModels(): monaco.editor.ITextModel[] {
    return Array.from(this.dirtyModels);
  }

  onDirtyChange(id: string, listener: DirtyChangeListener) {
    this.listeners.set(id, listener);
    return () => {
      this.listeners.delete(id);
    };
  }

  removeDirtyListener(id: string) {
    this.listeners.delete(id);
  }

  /**
   * Notify all listeners about a dirty state change.
   */
  private emitDirtyChange(model: monaco.editor.ITextModel, dirty: boolean) {
    for (const l of this.listeners.values()) {
      l(model, dirty);
    }
  }
}
