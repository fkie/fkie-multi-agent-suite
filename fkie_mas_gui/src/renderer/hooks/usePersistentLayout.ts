import { useDebounceCallback } from "@react-hook/debounce";
import { IJsonModel, Model } from "flexlayout-react";
import { useCallback, useEffect, useRef, useState } from "react";

import {
  asBorderNode,
  asRowNode,
  removeGenericTabs,
  sanitizeLayout,
  TJsonNode,
  TKeepTabFn,
} from "@/renderer/components/layout/LayoutPersistance";
import { useAppState } from "@/renderer/hooks/useAppState";
import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";

export type TUsePersistentLayoutOptions = {
  collection?: string;
  stateKey: string;
  defaultLayout: IJsonModel;
  version?: number;
  migrateFromLocalStorageKey?: string;
  keepTab: TKeepTabFn;
  repairLayout?: (json: IJsonModel) => IJsonModel;
  saveDelay?: number;
  /** Pass AppStateContext.ready if available, otherwise a heuristic is used. */
  isHydrated?: boolean;
  hydrationTimeoutMs?: number;
};

export type TUsePersistentLayoutResult = {
  /** Never null: built synchronously, rebuilt once when the stored layout arrives. */
  model: Model;
  layoutJson: IJsonModel;
  /** True as soon as the persisted layout has been applied. */
  isHydrated: boolean;
  saveLayout: (model: Model) => void;
  replaceLayout: (json: IJsonModel) => void;
  resetToDefaultLayout: () => void;
};

export function usePersistentLayout(options: TUsePersistentLayoutOptions): TUsePersistentLayoutResult {
  const {
    collection = "layouts",
    stateKey,
    defaultLayout,
    version = 1,
    migrateFromLocalStorageKey,
    keepTab,
    repairLayout,
    saveDelay = 500,
    isHydrated: externalHydrated,
    hydrationTimeoutMs = 300,
  } = options;

  const logCtx = useLoggingContext();

  const { value: layoutJson, set: setLayoutJson } = useAppState<IJsonModel>(collection, stateKey, defaultLayout, {
    version,
    ...(migrateFromLocalStorageKey ? { migrateFrom: { localStorageKey: migrateFromLocalStorageKey } } : {}),
  });

  const layoutJsonRef = useRef<IJsonModel>(layoutJson);
  layoutJsonRef.current = layoutJson;

  const defaultLayoutRef = useRef<IJsonModel>(defaultLayout);
  defaultLayoutRef.current = defaultLayout;

  /* ---------------- model factory (never throws) ---------------- */

  const createModel = useCallback(
    (json: IJsonModel): Model => {
      try {
        return Model.fromJson(sanitizeLayout(json));
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        logCtx.warn("Failed to restore layout, using default layout", message, "load layout failed");
        return Model.fromJson(structuredClone(defaultLayoutRef.current));
      }
    },
    [logCtx]
  );

  /* ---------------- hydration ---------------- */
  // useAppState returns the frozen default value while IndexedDB is still loading.
  // A value !== the default reference means the store has been read.
  const [hydrated, setHydrated] = useState<boolean>(externalHydrated ?? layoutJson !== defaultLayout);

  useEffect(() => {
    if (externalHydrated !== undefined) {
      setHydrated(externalHydrated);
      return;
    }
    if (hydrated) return;
    if (layoutJson !== defaultLayoutRef.current) {
      setHydrated(true);
      return;
    }
    // nothing stored at all -> accept the default after a short grace period
    const timer = setTimeout(() => setHydrated(true), hydrationTimeoutMs);
    return () => clearTimeout(timer);
  }, [externalHydrated, hydrated, layoutJson, hydrationTimeoutMs]);

  const hydratedRef = useRef(hydrated);
  hydratedRef.current = hydrated;

  /* ---------------- model instance ---------------- */
  // Built synchronously on first render so `model` is never null.
  const [modelState, setModelState] = useState<{ model: Model; stateKey: string; hydrated: boolean }>(() => ({
    model: createModel(layoutJsonRef.current),
    stateKey,
    hydrated,
  }));

  useEffect(() => {
    const keyChanged = modelState.stateKey !== stateKey;
    // rebuild exactly once when the persisted layout arrives after the first render
    const lateHydration = hydrated && !modelState.hydrated;
    if (!keyChanged && !lateHydration) return;

    setModelState({ model: createModel(layoutJsonRef.current), stateKey, hydrated });
  }, [stateKey, hydrated, createModel, modelState]);

  /* ---------------- save ---------------- */

  const saveLayout = useDebounceCallback((current: Model) => {
    // never persist before the stored layout was applied
    if (!hydratedRef.current) return;

    const json = current.toJson();

    for (const border of json.borders ?? []) {
      border.selected = -1;
      removeGenericTabs(asBorderNode(border as TJsonNode) as TJsonNode, keepTab);
    }
    json.layout = asRowNode(removeGenericTabs(json.layout as TJsonNode, keepTab));

    setLayoutJson(repairLayout ? repairLayout(json) : json);
  }, saveDelay);

  const replaceLayout = useCallback(
    (json: IJsonModel): void => {
      setLayoutJson(json);
      setModelState({ model: createModel(json), stateKey, hydrated: true });
      hydratedRef.current = true;
      setHydrated(true);
    },
    [setLayoutJson, createModel, stateKey]
  );

  const resetToDefaultLayout = useCallback((): void => {
    replaceLayout(structuredClone(defaultLayoutRef.current));
  }, [replaceLayout]);

  return { model: modelState.model, layoutJson, isHydrated: hydrated, saveLayout, replaceLayout, resetToDefaultLayout };
}
