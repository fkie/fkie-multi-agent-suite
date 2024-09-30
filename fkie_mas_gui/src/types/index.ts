import {
  EditorCloseCallback,
  EditorManagerEvents,
  FileRangeCallback,
  IEditor,
  IEditorManager,
  TEditorManager,
} from "./EditorManager";
import { FileRange } from "./FileRange";
import JSONObject, { JSONValue } from "./JsonObject";
import { IShutdownManager, ShutdownManagerEvents, TerminateCallback, TShutdownManager } from "./ShutdownManager";

export { EditorManagerEvents, ShutdownManagerEvents };
export type {
  EditorCloseCallback,
  FileRange,
  FileRangeCallback,
  IEditor,
  IEditorManager,
  IShutdownManager,
  JSONObject,
  JSONValue,
  TEditorManager,
  TerminateCallback,
  TShutdownManager,
};
