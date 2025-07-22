import { TFileRange, TLaunchArg } from "@/types";
import LayoutTabConfig from "./LayoutTabConfig";

export const EVENT_CLOSE_COMPONENT = "EVENT_CLOSE_COMPONENT" as const;
export const EVENT_OPEN_COMPONENT = "EVENT_OPEN_COMPONENT" as const;
export const EVENT_OPEN_SETTINGS = "EVENT_OPEN_SETTINGS" as const;
export const EVENT_OPEN_CONNECT = "EVENT_OPEN_CONNECT" as const;
export const EVENT_EDITOR_SELECT_RANGE = "EVENT_EDITOR_SELECT_RANGE" as const;
export const EVENT_FILTER_NODES = "EVENT_FILTER_NODES" as const;
export const EVENT_FILTER_TOPICS = "EVENT_FILTER_TOPICS" as const;
export const EVENT_FILTER_SERVICES = "EVENT_FILTER_SERVICES" as const;

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
  tabId: string;
  filePath: string;
  fileRange: TFileRange | null;
  launchArgs?: TLaunchArg[];
};

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
  tabId: string,
  filePath: string,
  fileRange: TFileRange | null,
  launchArgs?: TLaunchArg[]
): TEventEditorSelectRange {
  return { tabId, filePath, fileRange, launchArgs: launchArgs ? launchArgs : [] };
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
