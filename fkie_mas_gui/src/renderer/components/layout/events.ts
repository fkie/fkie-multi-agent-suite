import { TLayoutTabConfig } from "@/renderer/components/layout/LayoutTabConfig";
import { RosNode } from "@/renderer/models";
import { InfoStateLevel, TFileRange, TInfoState, TLaunchArg, TParameterRequest } from "@/types";
import { emitCustomEvent } from "react-custom-events";

export const EVENT_CLOSE_COMPONENT = "EVENT_CLOSE_COMPONENT" as const;
export const EVENT_OPEN_COMPONENT = "EVENT_OPEN_COMPONENT" as const;
export const EVENT_TOGGLE_COMPONENT = "EVENT_TOGGLE_COMPONENT" as const;
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

export type TEventSelectTab = {
  tabId: string;
  forSubLayoutOnly?: boolean;
};

export type TFilterText = {
  data: string;
};

export type TEventOpenComponent = {
  id: string;
  title: string;
  closable: boolean;
  toNodeId: string; // panel or tab id where to place the new tab
  component: string;
  config?: TLayoutTabConfig; // a place to hold json config for the hosted component
};

export type TEventEditorSelectRange = {
  editorId: string;
  filePath: string;
  fileRange: TFileRange | null;
  launchArgs?: TLaunchArg[];
  selectParameter?: TParameterRequest;
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

export function emitStateSuccess(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.SUCCESS, message: `✅ ${message}` } as TInfoState);
}

export function emitStateInfo(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.INFO, message: `ℹ️ ${message}` } as TInfoState);
}

export function emitStateWarn(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.WARN, message: `⚠️ ${message}` } as TInfoState);
}

export function emitStateError(message: string) {
  emitCustomEvent(EVENT_INFO_STATE, { level: InfoStateLevel.ERROR, message: `❌ ${message}` } as TInfoState);
}

export function emitOpenComponent(props: TEventOpenComponent) {
  emitCustomEvent(EVENT_OPEN_COMPONENT, props);
}
export function emitToggleComponent(props: TEventOpenComponent) {
  emitCustomEvent(EVENT_TOGGLE_COMPONENT, props);
}

export function emitCloseComponent(props: TEventId) {
  emitCustomEvent(EVENT_CLOSE_COMPONENT, props);
}

export function emitSelectTab(props: TEventSelectTab) {
  emitCustomEvent(EVENT_SELECT_TAB, props);
}

export function emitEditorSelectRange(props: TEventEditorSelectRange) {
  emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, props);
}

export function emitFilterNodes(props: TEventId) {
  emitCustomEvent(EVENT_FILTER_NODES, props);
}

export function emitFilterTopics(props: TFilterText) {
  emitCustomEvent(EVENT_FILTER_TOPICS, props);
}

export function emitFilterServices(props: TFilterText) {
  emitCustomEvent(EVENT_FILTER_SERVICES, props);
}

export function emitKillNodes(event: TEventKillNodes) {
  emitCustomEvent(EVENT_KILL_NODES, event);
}

export function emitShowScreens(event: TEventShowScreens) {
  emitCustomEvent(EVENT_SHOW_SCREENS, event);
}
