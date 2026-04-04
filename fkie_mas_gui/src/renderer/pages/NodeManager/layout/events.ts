import { RosNode } from "@/renderer/models";
import { InfoStateLevel, TFileRange, TInfoState, TLaunchArg } from "@/types";
import { emitCustomEvent } from "react-custom-events";
import LayoutTabConfig from "./LayoutTabConfig";

export const EVENT_CLOSE_COMPONENT = "EVENT_CLOSE_COMPONENT" as const;
export const EVENT_OPEN_COMPONENT = "EVENT_OPEN_COMPONENT" as const;
export const EVENT_OPEN_SETTINGS = "EVENT_OPEN_SETTINGS" as const;
export const EVENT_EDITOR_SELECT_RANGE = "EVENT_EDITOR_SELECT_RANGE" as const;
export const EVENT_FILTER_NODES = "EVENT_FILTER_NODES" as const;
export const EVENT_FILTER_TOPICS = "EVENT_FILTER_TOPICS" as const;
export const EVENT_FILTER_SERVICES = "EVENT_FILTER_SERVICES" as const;
export const EVENT_INFO_STATE = "EVENT_INFO_STATE" as const;
export const EVENT_KILL_NODES = "EVENT_KILL_NODES" as const;
export const EVENT_SHOW_SCREENS = "EVENT_SHOW_SCREENS" as const;
export const EVENT_SELECT_TAB = "EVENT_SELECT_TAB" as const;

export type TEventId = {
  id: string;
};

export type TFilterText = {
  data: string;
};

export type TEventOpenComponent = {
  id: string;
  title: string;
  component: React.ReactNode | undefined;
  closable: boolean;
  panelGroup: string; // panel or tab id where to place the new tab
  config: LayoutTabConfig; // a place to hold json config for the hosted component
};

export type TEventEditorSelectRange = {
  editorId: string;
  filePath: string;
  fileRange: TFileRange | null;
  launchArgs?: TLaunchArg[];
};

export type TEventInfoState = {
  level: InfoStateLevel;
  message: string;
};

export type TEventCollapsedState = {
  isCollapsed: boolean;
  key: string;
};

export type TEventKillNodes = {
  nodes: RosNode[];
};

export type TEventShowScreens = {
  nodes: RosNode[];
};


export function sendStateSuccess(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.SUCCESS, message: `✅ ${message}` } as TInfoState);
}

export function sendStateInfo(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.INFO, message: `ℹ️ ${message}` } as TInfoState);
}

export function sendStateWarn(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.WARN, message: `⚠️ ${message}` } as TInfoState);
}

export function sendStateError(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.ERROR, message: `❌ ${message}` } as TInfoState);
}


export function eventOpenComponent(
  id: string,
  title: string,
  component: React.ReactNode | undefined = undefined,
  closable: boolean = true,
  panelGroup: string = "", // panel or tab id where to place the new tab
  config: LayoutTabConfig = new LayoutTabConfig(false, panelGroup) // a place to hold json config for the hosted component
): TEventOpenComponent {
  return {
    id,
    title,
    closable,
    component,
    panelGroup,
    config,
  } as TEventOpenComponent;
}

export class SETTING extends String {
  static IDS = {
    INTERFACE: "interface",
    ABOUT: "about",
  };
}

export function eventOpenSettings(id: string): TEventId {
  return { id };
}

export function eventCloseComponent(id: string): TEventId {
  return { id };
}

export function eventEditorSelectRange(
  editorId: string,
  filePath: string,
  fileRange: TFileRange | null,
  launchArgs?: TLaunchArg[]
): TEventEditorSelectRange {
  return { editorId, filePath, fileRange, launchArgs: launchArgs ? launchArgs : [] };
}

export function eventFilterNodes(id: string): TEventId {
  return { id };
}

export function eventFilterTopics(data: string): TFilterText {
  return { data };
}

export function eventFilterServices(data: string): TFilterText {
  return { data };
}

export function emitKillNodes(event: TEventKillNodes) {
  emitCustomEvent(EVENT_KILL_NODES, event);
}


export function emitShowScreens(event: TEventShowScreens) {
  emitCustomEvent(EVENT_SHOW_SCREENS, event);
}
