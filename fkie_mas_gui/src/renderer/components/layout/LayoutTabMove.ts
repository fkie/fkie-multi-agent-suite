import { Actions, BorderNode, Model, TabNode } from "flexlayout-react";

import { LAYOUT_TAB_SETS, LAYOUT_TABS } from "@/renderer/components/layout";
import { TContentId, TLayoutTabConfig } from "@/renderer/components/layout/LayoutTabConfig";
import { collapseBorderOnLastTab } from "@/renderer/components/layout/LayoutTargets";
import { DOMAIN_LAYOUT_COMPONENTS } from "./LayoutDefines";

export type TMovableTab = {
  id: string;
  name: string;
  component: string;
  closable: boolean;
  config: TLayoutTabConfig;
  toNodeId: string;
};

/** Tabs that are structural or bound to a domain layout cannot be moved. */
const NON_MOVABLE_COMPONENTS: string[] = [
  ...DOMAIN_LAYOUT_COMPONENTS,
  LAYOUT_TABS.DOMAIN,
  LAYOUT_TABS.NO_RUNNING_DAEMONS,
  LAYOUT_TABS.HOSTS,
  LAYOUT_TABS.DETAILS,
  LAYOUT_TABS.PACKAGES,
  LAYOUT_TABS.LOGGING,
  LAYOUT_TABS.SETTINGS,
  LAYOUT_TABS.ABOUT,
];

export function isMovableTab(node: TabNode): boolean {
  if (node.getType() !== "tab") return false;
  if (NON_MOVABLE_COMPONENTS.includes(node.getComponent() ?? "")) return false;
  return node.isEnableClose();
}

/** The domain a tab originally belongs to, if any. */
export function originContentId(node: TabNode): TContentId | undefined {
  return (node.getConfig() as TLayoutTabConfig)?.contentId;
}

function preferredTarget(node: TabNode): string {
  const parent = node.getParent();
  if (parent?.getType() === "border") {
    return (parent as BorderNode).getLocation().getName() === "right"
      ? LAYOUT_TAB_SETS.BORDER_RIGHT
      : LAYOUT_TAB_SETS.BORDER_BOTTOM;
  }
  return LAYOUT_TAB_SETS.CENTER;
}

/**
 * Serialize a tab and remove it silently.
 * doAction() bypasses onAction(), so no dirty guard and no panel destruction happens.
 */
export function takeOutTab(model: Model, tabId: string): TMovableTab | undefined {
  const node = model.getNodeById(tabId) as TabNode | undefined;
  if (!node || node.getType() !== "tab") return undefined;

  const tab: TMovableTab = {
    id: node.getId(),
    name: node.getName(),
    component: node.getComponent() ?? "",
    closable: node.isEnableClose(),
    config: (node.getConfig() ?? {}) as TLayoutTabConfig,
    toNodeId: preferredTarget(node),
  };
  collapseBorderOnLastTab(model, tabId);
  model.doAction(Actions.deleteTab(tabId));
  return tab;
}
