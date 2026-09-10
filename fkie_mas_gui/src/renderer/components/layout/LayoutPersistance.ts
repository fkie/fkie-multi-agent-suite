import { IJsonBorderNode, IJsonModel, IJsonRowNode } from "flexlayout-react";

/** Minimal JSON node shape used to walk/modify a FlexLayout JSON model. */
export type TJsonNode = {
  id?: string;
  type?: string;
  component?: string;
  location?: string;
  selected?: number;
  children?: TJsonNode[];
};

/** Predicate deciding whether a tab node should be persisted. */
export type TKeepTabFn = (tab: TJsonNode) => boolean;

/** Removes/repairs duplicate ids in the layout JSON, keeping the first occurrence. */
export function sanitizeLayout(json: IJsonModel): IJsonModel {
  // deep clone first, so shared object references can no longer collide
  const cloned: IJsonModel = structuredClone(json);
  const seen = new Set<string>();

  const walk = (node: TJsonNode): void => {
    if (node.id) {
      if (seen.has(node.id)) {
        // duplicate: assign a new unique id
        node.id = `#dup-${crypto.randomUUID?.() ?? Math.random().toString(36).slice(2)}`;
      }
      seen.add(node.id);
    }
    for (const child of node.children ?? []) {
      walk(child);
    }
  };

  for (const border of cloned.borders ?? []) {
    walk(border as TJsonNode);
  }
  walk(cloned.layout as TJsonNode);
  return cloned;
}

/** Recursively removes all tabs which are not accepted by `keepTab`. */
export function removeGenericTabs<T extends TJsonNode>(parent: T, keepTab: TKeepTabFn): T {
  if (!parent.children) return parent;

  // if tabs are removed, the stored selection index may become invalid
  if (parent.selected !== undefined) {
    parent.selected = undefined;
  }

  parent.children = parent.children.filter((item) => {
    if (item.type === "tab") return keepTab(item);
    if (item.children) {
      removeGenericTabs(item, keepTab);
      return true;
    }
    return false;
  });

  return parent;
}

/** Recursively searches a node by id. */
export function findJsonNodeById(node: TJsonNode, id: string): TJsonNode | undefined {
  if (node.id === id) return node;
  for (const child of node.children ?? []) {
    const found = findJsonNodeById(child, id);
    if (found) return found;
  }
  return undefined;
}

/** Recursively checks whether a node with the given id (and optional type) exists. */
export function hasJsonNode(node: TJsonNode, id: string, type?: string): boolean {
  const found = findJsonNodeById(node, id);
  return found !== undefined && (type === undefined || found.type === type);
}

/** Convenience casts for the FlexLayout JSON types. */
export function asRowNode(node: TJsonNode): IJsonRowNode {
  return node as unknown as IJsonRowNode;
}

export function asBorderNode(node: TJsonNode): IJsonBorderNode {
  return node as unknown as IJsonBorderNode;
}
