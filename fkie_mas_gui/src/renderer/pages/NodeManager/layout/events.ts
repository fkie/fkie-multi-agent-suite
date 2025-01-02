import { TFileRange, TLaunchArg } from "@/types";
import LayoutTabConfig from "./LayoutTabConfig";

export const EVENT_CLOSE_COMPONENT = "EVENT_CLOSE_COMPONENT" as const;
export const EVENT_OPEN_COMPONENT = "EVENT_OPEN_COMPONENT" as const;
export const EVENT_OPEN_SETTINGS = "EVENT_OPEN_SETTINGS" as const;
export const EVENT_OPEN_CONNECT = "EVENT_OPEN_CONNECT" as const;
export const EVENT_EDITOR_SELECT_RANGE = "EVENT_EDITOR_SELECT_RANGE" as const;
export const EVENT_FILTER_NODES = "EVENT_FILTER_NODES" as const;

export function eventOpenComponent(
  id: string,
  title: string,
  component: object,
  closable: boolean = true,
  panelGroup: string = "", // panel or tab id where to place the new tab
  config: LayoutTabConfig = new LayoutTabConfig(false, panelGroup) // a place to hold json config for the hosted component
): {
  id: string;
  title: string;
  component: object;
  closable: boolean;
  panelGroup: string; // panel or tab id where to place the new tab
  config: LayoutTabConfig;
} {
  return {
    id,
    title,
    closable,
    component,
    panelGroup,
    config,
  };
}

export class SETTING extends String {
  static IDS = {
    INTERFACE: "interface",
    ABOUT: "about",
  };
}

export function eventOpenSettings(id: SETTING): { id: SETTING } {
  return { id };
}

export function eventCloseComponent(id: string): { id: string } {
  return { id };
}

export function eventEditorSelectRange(
  tabId: string,
  filePath: string,
  fileRange: TFileRange | null,
  launchArgs: TLaunchArg[]
): { tabId: string; filePath: string; fileRange: TFileRange | null; launchArgs: TLaunchArg[] } {
  return { tabId, filePath, fileRange, launchArgs };
}

export function eventFilterNodes(id: string): { id: string } {
  return { id };
}
