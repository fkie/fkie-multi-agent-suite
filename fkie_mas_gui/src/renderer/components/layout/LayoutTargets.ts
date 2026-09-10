import { Actions, BorderNode, DockLocation, Model, TabNode } from "flexlayout-react";

import { LAYOUT_TAB_SETS } from "./index";

export type TDockTarget = {
  /** id of the tabset or border node to dock into */
  id: string;
  isBorder: boolean;
  location: DockLocation;
};

/** Border nodes always use fixed ids of the form "border_<location>". */
export function findBorderByLocation(model: Model, location: DockLocation): BorderNode | undefined {
  const node = model.getNodeById(`border_${location.getName()}`);
  return node instanceof BorderNode ? node : undefined;
}

/** Maps a logical target id (e.g. LAYOUT_TAB_SETS.BORDER_BOTTOM) to a dock location. */
function borderLocationFor(toNodeId: string): DockLocation | undefined {
  switch (toNodeId) {
    case LAYOUT_TAB_SETS.BORDER_TOP:
      return DockLocation.TOP;
    case LAYOUT_TAB_SETS.BORDER_BOTTOM:
      return DockLocation.BOTTOM;
    case LAYOUT_TAB_SETS.BORDER_RIGHT:
      return DockLocation.RIGHT;
    default:
      return undefined;
  }
}

/**
 * Resolves the node a new tab should be added to.
 * Works for the main layout as well as for a domain sub-layout.
 */
export function resolveDockTarget(model: Model, toNodeId?: string, fallbackTabId?: string): TDockTarget | undefined {
  // 1) border target
  const location = toNodeId ? borderLocationFor(toNodeId) : undefined;
  if (location) {
    const border = findBorderByLocation(model, location);
    if (border) return { id: border.getId(), isBorder: true, location };
  }

  // 2) explicit target node inside this model
  if (toNodeId) {
    const target = model.getNodeById(toNodeId);
    if (target?.getType() === "tabset") return { id: target.getId(), isBorder: false, location: DockLocation.CENTER };
    if (target?.getType() === "tab") {
      const parent = target.getParent();
      if (parent) {
        const isBorder = parent.getType() === "border";
        return {
          id: parent.getId(),
          isBorder,
          location: isBorder ? (parent as BorderNode).getLocation() : DockLocation.CENTER,
        };
      }
    }
  }

  // 3) tabset of a well known fallback tab (e.g. the "Nodes" tab of a domain layout)
  if (fallbackTabId) {
    const parent = model.getNodeById(fallbackTabId)?.getParent();
    if (parent?.getType() === "tabset") return { id: parent.getId(), isBorder: false, location: DockLocation.CENTER };
  }

  // 4) active tabset
  const active = model.getActiveTabset();
  return active ? { id: active.getId(), isBorder: false, location: DockLocation.CENTER } : undefined;
}

/**
 * Makes sure a newly added border tab becomes visible:
 * if no tab of that border is currently selected, select the last one.
 */
export function ensureBorderTabVisible(model: Model, location: DockLocation): void {
  const border = findBorderByLocation(model, location);
  if (!border) return;
  const hasVisible = border.getChildren().some((c) => (c as TabNode).isVisible());
  if (!hasVisible && border.getChildren().length > 0) {
    model.doAction(Actions.selectTab(border.getChildren().slice(-1)[0].getId()));
  }
}

/**
 * Hides a border when its last visible tab is closed
 * (selecting the tab before deleting collapses the border).
 */
export function collapseBorderOnLastTab(model: Model, tabId: string): void {
  const parent = model.getNodeById(tabId)?.getParent();
  if (parent?.getType() !== "border") return;
  const border = parent as BorderNode;
  if (border.getChildren().length === 2 && border.getSelectedNode()?.isVisible()) {
    model.doAction(Actions.selectTab(tabId));
  }
}
