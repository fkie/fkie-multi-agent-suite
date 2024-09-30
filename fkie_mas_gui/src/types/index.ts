import { DialogManagerEvents, IDialogManager, TDialogManager } from "./DialogManager";
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
import {
  ISubscriber,
  ISubscriberManager,
  SubscriberCloseCallback,
  SubscriberManagerEvents,
  TSubscriberManager,
} from "./SubscriberManager";
import {
  ITerminal,
  ITerminalManager,
  TerminalCloseCallback,
  TerminalManagerEvents,
  TTerminalManager,
} from "./TerminalManager";

export {
  DialogManagerEvents,
  EditorManagerEvents,
  ShutdownManagerEvents,
  SubscriberManagerEvents,
  TerminalManagerEvents,
};
export type {
  EditorCloseCallback,
  FileRange,
  FileRangeCallback,
  IDialogManager,
  IEditor,
  IEditorManager,
  IShutdownManager,
  ISubscriber,
  ISubscriberManager,
  ITerminal,
  ITerminalManager,
  JSONObject,
  JSONValue,
  SubscriberCloseCallback,
  TDialogManager,
  TEditorManager,
  TerminalCloseCallback,
  TerminateCallback,
  TShutdownManager,
  TSubscriberManager,
  TTerminalManager,
};
