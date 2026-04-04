import * as FlexLayout from "flexlayout-react";
import { useCallback, useEffect, useReducer, useRef, useState } from "react";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";
import { EVENT_SELECT_TAB } from "@/renderer/pages/NodeManager/layout/events";
import { Box } from "@mui/material";
import { useCustomEventListener } from "react-custom-events";

/**
 * Minimal JSON node shape used for manipulating the FlexLayout JSON model.
 * We intentionally keep this small and typed to avoid using `any`.

 */
type JsonNode = {
  type: string;
  children?: JsonNode[];
  name?: string;
  component?: string;
  enableClose?: boolean;
  enableMaximize?: boolean;
  config?: Record<string, unknown>;
  weight?: number;
};

/**
 * Options for the persistent FlexLayout hook.
 * TId is the identifier type used to distinguish tabs (e.g. domainId, workspaceId).

 */
export interface DomainFlexLayoutOptions<TId extends string | number> {
  /** Storage key used to persist the serialized layout JSON (e.g. localStorage) */
  storageKey: string;
  /** List of all ids that should have a corresponding tab in the layout */
  ids: TId[];
  /** Name of the FlexLayout component used for these tabs (e.g. "domainHostTree") */
  componentName: string;
  /**
   * Key name under `tab.config` where the id is stored.
   * Example: "domainId", "workspaceId", ...

   */
  configKey: string;

  /** ID of the tab where this layout is located. It is used to update the content after the tab is selected again. */
  insideTabId: string;
}

/**
 * Result of the persistent FlexLayout hook.

 */
export interface DomainFlexLayoutResult {
  model: FlexLayout.Model | null;
  setModel: (model: FlexLayout.Model | null) => void;
  handleModelChange: (model: FlexLayout.Model) => void;
}

/**
 * Hook that manages a FlexLayout.Model and keeps it in sync with:
 * - A list of "ids" (one tab per id)
 * - A persisted JSON representation of the layout

 *
 * It:
 * - Restores from persisted JSON if available (once on initialization)
 * - Merges added/removed ids into the existing layout when ids change
 * - Saves layout changes to storage via `storageKey`

 *
 * Important:
 * - We do NOT recreate the model on every model change.
 *   That would destroy internal component state (e.g. expanded tree nodes)
 *   whenever the user switches tabs or drags them.

 */
export default function useDomainFlexLayout<TId extends string | number>(
  options: DomainFlexLayoutOptions<TId>
): DomainFlexLayoutResult {
  const { storageKey, ids, componentName, configKey } = options;

  const [layoutJsonString, setLayoutJsonString] = useLocalStorage<string>(storageKey, "");
  const [model, setModel] = useState<FlexLayout.Model | null>(null);
  const initializedRef = useRef(false);
  const logCtx = useLoggingContext();

  /**
   * Create a tab JSON node for a given id.
   * The tab name is formatted as "Domain <id>" to provide a descriptive label.

   */
  const createTabForId = useCallback(
    (id: TId): JsonNode => ({
      type: "tab",
      name: `Domain ${id}`,
      component: componentName,
      enableClose: false,
      enableMaximize: false,
      config: { [configKey]: id },
    }),
    [componentName, configKey]
  );

  /**
   * Create a simple default layout with a single tabset containing one tab per id.

   */
  const createDefaultLayoutJson = useCallback(
    (idList: TId[]): FlexLayout.IJsonModel => ({
      global: { tabSetEnableMaximize: false },
      borders: [],
      layout: {
        type: "row",
        weight: 100,
        children: [
          {
            type: "tabset",
            weight: 100,
            enableClose: false,
            enableMaximize: false,
            children: idList.map((id) => createTabForId(id)),
          } as FlexLayout.IJsonTabSetNode,
        ],
      } as FlexLayout.IJsonRowNode,
    }),
    [createTabForId]
  );

  /**
   * Merge a saved layout JSON with the current ids:
   * - keep tabs whose id still exists
   * - add tabs for new ids
   * - remove tabs for ids that no longer exist
   * - normalize tab properties (name, component, flags, config)

   */
  const mergeLayoutJson = useCallback(
    (savedJson: FlexLayout.IJsonModel, idList: TId[]): FlexLayout.IJsonModel => {
      // Deep copy to avoid mutating the original object
      const json: FlexLayout.IJsonModel = JSON.parse(JSON.stringify(savedJson));
      const idSet = new Set(idList.map((id) => String(id)));
      const existingIds = new Set<string>();

      const processNode = (node: JsonNode | undefined): void => {
        if (!node || !node.children) {
          return;
        }

        // Process children first
        node.children.forEach(processNode);

        // Filter and normalize tab children with the configured id
        node.children = node.children.filter((child) => {
          if (child.type === "tab" && child.config && configKey in child.config) {
            const raw = child.config[configKey];
            const idStr = String(raw);

            if (!idSet.has(idStr)) {
              // Id is no longer present -> drop this tab
              return false;
            }

            // Id still exists -> normalize and keep
            existingIds.add(idStr);
            child.name = `Domain ${raw}`;
            child.component = componentName;
            child.enableClose = false;
            child.enableMaximize = false;
            child.config = { [configKey]: raw };
            return true;
          }

          // Keep non-matching children as they are
          return true;
        });
      };

      processNode(json.layout as JsonNode);

      // Add missing ids as new tabs
      const missing = idList.filter((id) => !existingIds.has(String(id)));
      if (missing.length === 0) {
        return json;
      }

      const findFirstTabset = (node: JsonNode | undefined): JsonNode | null => {
        if (!node) return null;
        if (node.type === "tabset") return node;

        if (node.children) {
          for (const child of node.children) {
            const result = findFirstTabset(child);
            if (result) return result;
          }
        }

        return null;
      };

      const tabset = findFirstTabset(json.layout as JsonNode);
      if (!tabset) {
        // As a fallback, rebuild a simple default layout
        return createDefaultLayoutJson(idList);
      }

      if (!tabset.children) {
        tabset.children = [];
      }

      for (const id of missing) {
        tabset.children.push(createTabForId(id));
      }

      return json;
    },
    [componentName, configKey, createDefaultLayoutJson, createTabForId]
  );

  /**
   * Handler for FlexLayout.Layout.onModelChange.
   * Only persists the JSON, does NOT update the model state,
   * to avoid resetting the UI on every interaction.

   *
   * The model instance remains the same while the user interacts
   * (switches tabs, drags tabs, resizes, ...).

   */
  const handleModelChange = useCallback(
    (nextModel: FlexLayout.Model): void => {
      try {
        const json = nextModel.toJson();
        setLayoutJsonString(JSON.stringify(json));
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        logCtx.warn("Failed to serialize layout model", message, "layout not saved");
      }
    },
    [setLayoutJsonString, logCtx]
  );

  /**
   * Initialize layout from storage on first run,
   * then only react to changes in the ids list (e.g. new/removed domains).

   *
   * Important:
   * - We intentionally do NOT depend on `layoutJsonString` or `model` here,
   *   so the model is not recreated on every user interaction.
   * - `layoutJsonString` is read once during initialization via closure.

   */
  useEffect(() => {
    if (ids.length === 0) {
      setModel(null);
      return;
    }

    const idList = [...ids];

    // First initialization: restore from storage or create default layout
    if (!initializedRef.current) {
      initializedRef.current = true;

      let baseJson: FlexLayout.IJsonModel | null = null;
      if (layoutJsonString) {
        try {
          baseJson = JSON.parse(layoutJsonString) as FlexLayout.IJsonModel;
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          logCtx.warn("Failed to parse saved layout, using default layout", message, "layout restore failed");
        }
      }

      const finalJson = baseJson ? mergeLayoutJson(baseJson, idList) : createDefaultLayoutJson(idList);
      setModel(FlexLayout.Model.fromJson(finalJson));
      return;
    }

    // Subsequent updates: ids changed after initialization
    // We want to merge the current layout with the new ids
    // without losing user-defined layout (tab order, sizes, ...).
    if (model) {
      try {
        const currentJson = model.toJson();
        const mergedJson = mergeLayoutJson(currentJson, idList);
        setModel(FlexLayout.Model.fromJson(mergedJson));
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        logCtx.warn("Failed to merge layout, recreating layout", message, "layout merge failed");
        const fallbackJson = createDefaultLayoutJson(idList);
        setModel(FlexLayout.Model.fromJson(fallbackJson));
      }
    } else {
      // Model was null (e.g. after ids were empty before)
      // -> create a fresh model
      const fallbackJson = createDefaultLayoutJson(idList);
      setModel(FlexLayout.Model.fromJson(fallbackJson));
    }

    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ids, mergeLayoutJson, createDefaultLayoutJson, logCtx.warn]);
  // Note:
  // - We intentionally omit `layoutJsonString` and `model` from dependencies
  //   to avoid recreating the model on every storage update or model change.
  //   This keeps tab contents (e.g. expanded tree nodes) stable.

  return { model, setModel, handleModelChange };
}

/**
 * Props for the generic DomainFlexLayout component.

 */
export interface DomainFlexLayoutProps<TId extends string | number> extends DomainFlexLayoutOptions<TId> {
  /**
   * Factory that renders the content for each tab.
   * The hook ensures that `id` matches the value stored under `configKey`.

   */
  factory: (tabNode: FlexLayout.TabNode, id: TId) => JSX.Element;
}

/**
 * Generic FlexLayout wrapper that:
 * - Uses useDomainFlexLayout to manage model + persistence
 * - Exposes a typed `factory` for rendering tab content

 */
export function DomainFlexLayout<TId extends string | number>(props: DomainFlexLayoutProps<TId>): JSX.Element | null {
  const { ids, storageKey, componentName, configKey, insideTabId, factory } = props;

  const [forceUpdate, setForceUpdate] = useReducer((x) => x + 1, 0);
  const { model, handleModelChange } = useDomainFlexLayout<TId>({
    storageKey,
    ids,
    componentName,
    configKey,
    insideTabId,
  });

  /**
   * Wrapper around the user-provided factory.
   * It extracts the id from the tab config using `configKey`
   * and passes it as a typed argument to the factory.

   */
  const nodeFactory = useCallback(
    (tabNode: FlexLayout.TabNode): JSX.Element => {
      const configUnknown = tabNode.getConfig() as unknown;

      const config =
        typeof configUnknown === "object" && configUnknown !== null
          ? (configUnknown as Record<string, unknown>)
          : ({} as Record<string, unknown>);

      const rawId = config[configKey];

      // Fallback: if the config does not contain the id, use the tab id as string
      const idValue =
        typeof rawId === "string" || typeof rawId === "number" ? (rawId as TId) : (String(tabNode.getId()) as TId);

      return factory(tabNode, idValue);
    },
    [factory, configKey]
  );

  useCustomEventListener(EVENT_SELECT_TAB, (data: { tabId: string }) => {
    // IMPORTANT: When the surrounding Nodes tab becomes active again,
    // rebuild the internal domain-specific FlexLayout model once.
    if (data.tabId === insideTabId) {
      setForceUpdate();
    }
  });

  useEffect(() => {
    window.dispatchEvent(new Event("resize"));
  }, [forceUpdate]);

  if (!model) {
    // If there is no model (e.g. no ids), render nothing
    return null;
  }

  return (
    <Box
      sx={{
        flex: 1,
        height: "100%",
        width: "100%",
        overflow: "hidden",
        position: "relative", // <- important: anchor for absolute FlexLayout
      }}
    >
      <FlexLayout.Layout model={model} factory={nodeFactory} onModelChange={handleModelChange} />
    </Box>
  );
}
