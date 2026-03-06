import * as monaco from "monaco-editor";
import { editor } from "monaco-editor/esm/vs/editor/editor.api";

import { FileItem } from "../models";

export type SaveResult = {
  uriPath: string;
  result: boolean;
  message: string;
  tabIds?: string[];
};

export type TModelResult = {
  model: editor.ITextModel | null;
  file: FileItem | null;
  error: string | null;
};

export type ModifiedTabsInfo = {
  tabId: string;
  uriPaths: string[];
};

/**
 * Listener signature for dirty state changes.
 *
 * model  -> the Monaco text model that changed
 * dirty  -> new dirty state
 */
export type DirtyChangeListener = (model: monaco.editor.ITextModel, dirty: boolean) => void;
