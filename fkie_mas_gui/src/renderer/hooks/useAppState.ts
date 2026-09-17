import { useCallback, useContext, useEffect, useRef } from "react";
import { AppStateContext } from "@/renderer/context/AppStateContext";
import { MigrationEntry } from "@/renderer/db/persistanceCore";
import { JSONValue } from "@/types";

/* ======================== Types =========================== */

/**
 * Converts between any type T and JSONValue for IndexedDB persistence.
 * Required when T is not natively JSON-serializable (e.g. Set, Map, Date, class instances).
 *
 * If T is already JSON-compatible (primitives, plain objects, arrays), no serializer is needed.
 *
 * @example
 * const dateSerializer: Serializer<Date> = {
 *   serialize: (d) => d.toISOString(),
 *   deserialize: (raw) => new Date(raw as string),
 * };
 *
 * @example
 * const setSerializer: Serializer<Set<string>> = {
 *   serialize: (s) => [...s],
 *   deserialize: (raw) => new Set(raw as string[]),
 * };
 */
export interface Serializer<T> {
  /** Convert T → JSONValue before writing to DB. */
  serialize: (value: T) => JSONValue;
  /** Convert JSONValue → T after reading from DB. */
  deserialize: (raw: JSONValue) => T;
}

/**
 * Migration function called when the stored version does not match the current version.
 *
 * The input is always the raw JSONValue from the database (before deserialization).
 * The output is the migrated value as T (your application type).
 *
 * If a serializer is provided, the hook will call `serializer.serialize(migratedT)`
 * before persisting the migrated value back to the DB.
 *
 * Return `undefined` to discard the stored value and fall back to `defaultValue`.
 *
 * @param oldValue - The raw JSONValue as stored in IndexedDB.
 * @param oldVersion - The version tag stored alongside the value.
 *                     `undefined` if the entry was stored without a version
 *                     (e.g. migrated from localStorage).
 * @returns The migrated value as T, or undefined to discard.
 *
 * @example
 * migrate: (oldValue, oldVersion) => {
 *   if (oldVersion === 1) {
 *     const old = oldValue as OldFormat;
 *     return { ...old, newField: "default" }; // → CurrentFormat (T)
 *   }
 *   return undefined; // unknown version → use defaultValue
 * }
 */
export type MigrateFn<T> = (oldValue: JSONValue, oldVersion: number | string | undefined) => T | undefined;

/**
 * Setter type that supports both direct values and functional updates,
 * matching the React useState API.
 *
 * @example
 * // Direct value
 * set(newItems);
 *
 * @example
 * // Functional update
 * set((prev) => [...prev, newItem]);
 */
export type AppStateSetter<T> = (valueOrUpdater: T | ((prev: T) => T)) => void;

/**
 * Configuration options for useAppState.
 *
 * @template T - The application-level type stored by this hook.
 *               Can be any type; provide a `serializer` if T is not JSON-native.
 */
export interface UseAppStateOptions<T> {
  /**
   * If true, DB writes are debounced.
   * In-memory state and cross-tab sync update immediately;
   * only the IndexedDB write is delayed.
   * Useful for sliders, drag-resize, or other high-frequency updates.
   */
  debounce?: boolean;

  /** Debounce delay in milliseconds. Default: 300. */
  debounceMs?: number;

  /**
   * Current schema version for this entry.
   * When the stored version differs from this value, `migrate` is called.
   * If undefined, no version tracking or migration is performed.
   */
  version?: number | string;

  /**
   * Custom serializer for non-JSON-compatible types.
   * If not provided, T is stored/read via direct cast (must be JSON-compatible).
   */
  serializer?: Serializer<T>;

  /**
   * Migration function for version upgrades.
   * Called when `stored.version !== options.version`.
   *
   * Also called during `importJSON()` if the imported file has a different version.
   *
   * @see MigrateFn
   */
  migrate?: MigrateFn<T>;

  /**
   * Automatically migrate a value from localStorage on first app start.
   * The localStorage entry is removed after successful migration.
   *
   * This registration is idempotent — calling it multiple times with the
   * same key has no additional effect.
   */
  migrateFrom?: {
    /** The localStorage key to read and migrate from. */
    localStorageKey: string;
    /**
     * Optional parser for the raw localStorage string.
     * Default: JSON.parse with automatic detection of the
     * `{ version, value }` wrapper used by the old useLocalStorage hook.
     */
    parse?: (raw: string) => JSONValue | undefined;
  };
}

/**
 * Return value of useAppState<T>.
 *
 * @template T - The application-level type.
 */
export interface UseAppStateReturn<T> {
  /** Current value. Falls back to `defaultValue` if nothing is stored or migration fails. */
  value: T;

  /**
   * Persist a new value. Supports both direct values and functional updates
   * (same API as React's useState setter).
   * Respects debounce and version options.
   *
   * @example
   * // Direct value
   * set([...items, newItem]);
   *
   * @example
   * // Functional update (receives current value)
   * set((prev) => [...prev, newItem].sort((a, b) => a.name.localeCompare(b.name)));
   */
  set: AppStateSetter<T>;

  /** Remove the stored entry. Next read will return `defaultValue`. */
  remove: () => void;

  /**
   * Export this single entry as a JSON string.
   * Includes metadata (namespace, key, version, timestamp).
   * Can be saved to a file and later imported via `importJSON()`.
   */
  exportJSON: () => string;

  /**
   * Import a value from a JSON string (e.g. from a file).
   * Supports both the wrapped format from `exportJSON()` and raw JSON values.
   *
   * If the imported version differs from the current version,
   * the `migrate` function is called automatically.
   *
   * @returns true on success, false on parse/validation/migration failure.
   */
  importJSON: (json: string) => boolean;
}

/* ======================== useAppState =========================== */

/**
 * Generic state persistence hook backed by IndexedDB.
 *
 * Features:
 * - **Any type T**: No restriction to JSONValue. Provide a `serializer` for non-JSON types.
 * - **Functional updates**: `set((prev) => ...)` works like React's useState.
 * - **Versioned migrations**: Automatically migrates stored data when version changes.
 * - **localStorage migration**: One-time migration from legacy localStorage keys.
 * - **Debounced writes**: Immediate UI updates with batched DB writes.
 * - **Cross-tab sync**: Changes propagate to all open tabs via BroadcastChannel.
 * - **Export/Import**: Per-entry serialization to/from JSON files.
 * - **Encryption-ready**: Uses the transformer layer from AppStateContext.
 *
 * @template T - The application type. Can be anything (classes, Sets, Maps, etc.)
 *
 * @param namespace - Logical group for this entry (e.g. "layout", "provider", "ui").
 * @param key - Unique key within the namespace.
 * @param defaultValue - Fallback when no value is stored or migration fails.
 * @param options - Configuration for versioning, serialization, migration, debounce.
 *
 * @example
 * // Simple JSON-compatible value (no serializer needed)
 * const { value: count, set: setCount } = useAppState<number>("counters", "clicks", 0);
 *
 * @example
 * // Functional update
 * const { value: items, set: setItems } = useAppState<Item[]>("data", "items", []);
 * setItems((prev) => [...prev, newItem]);
 *
 * @example
 * // Custom type with serializer
 * interface TagCollection { name: string; tags: Set<string>; }
 * const { value, set } = useAppState<TagCollection>("data", "tags", defaultTags, {
 *   serializer: {
 *     serialize: (v) => ({ name: v.name, tags: [...v.tags] }),
 *     deserialize: (raw) => {
 *       const o = raw as { name: string; tags: string[] };
 *       return { name: o.name, tags: new Set(o.tags) };
 *     },
 *   },
 * });
 *
 * @example
 * // With version migration
 * const { value } = useAppState<ConfigV3>("config", "main", defaultV3, {
 *   version: 3,
 *   migrate: (oldValue, oldVersion) => {
 *     if (oldVersion === 2) return upgradeV2toV3(oldValue);
 *     if (oldVersion === 1) return upgradeV1toV3(oldValue);
 *     return undefined; // discard unknown
 *   },
 * });
 *
 * @example
 * // With localStorage migration + version migration
 * const { value } = useAppState<MyConfig[]>("provider", "configurations", [], {
 *   version: 1,
 *   migrateFrom: { localStorageKey: "Provider:startConfigurations" },
 *   migrate: (oldValue, oldVersion) => {
 *     if (oldVersion === undefined) return oldValue as unknown as MyConfig[];
 *     return [];
 *   },
 * });
 *
 * @example
 * // Debounced for high-frequency updates (slider, resize)
 * const { value: width, set: setWidth } = useAppState<number>("ui", "sidebar", 300, {
 *   debounce: true,
 *   debounceMs: 500,
 * });
 */
export function useAppState<T>(
  namespace: string,
  key: string,
  defaultValue: T,
  options?: UseAppStateOptions<T>
): UseAppStateReturn<T> {
  const ctx = useContext(AppStateContext);
  if (!ctx) throw new Error("useAppState must be used inside AppStateProvider");

  // Freeze defaultValue like useState does: only the first-render value is used.
  // This prevents new references (e.g. [] or {}) from causing infinite update loops.
  const defaultRef = useRef(defaultValue);

  const serRef = useRef(options?.serializer);
  serRef.current = options?.serializer;

  const migrateRef = useRef(options?.migrate);
  migrateRef.current = options?.migrate;

  const currentVersion = options?.version;

  // Stable identity for the composite key (avoids unnecessary effect re-runs)
  const compositeKey = `${namespace}:${key}`;

  // Tracks whether migration has been persisted to prevent re-triggering
  const migrationDone = useRef<string | null>(null);

  /* ================ localStorage Migration Registration ================ */

  const lsMigrated = useRef(false);
  useEffect(() => {
    if (options?.migrateFrom && !lsMigrated.current) {
      lsMigrated.current = true;
      const entry: MigrationEntry = {
        localStorageKey: options.migrateFrom.localStorageKey,
        dbKey: compositeKey,
        namespace,
        parse: options.migrateFrom.parse,
        version: options.version,
      };
      ctx.registerMigrations([entry]);
    }
  }, [compositeKey]);

  /* ================ Read Record ================ */

  const record = ctx.getRecord(namespace, key);

  // Extract stable primitives for effect dependencies (avoids object reference issues)
  const recordValue = record?.value;
  const recordVersion = record?.version;
  const hasRecord = record !== undefined;

  /* ================ Serialize / Deserialize Helpers ================ */

  function serializeValue(v: T): JSONValue {
    if (serRef.current) return serRef.current.serialize(v);
    return v as unknown as JSONValue;
  }

  function deserializeRaw(raw: JSONValue): T {
    if (serRef.current) {
      try {
        return serRef.current.deserialize(raw);
      } catch {
        return defaultRef.current;
      }
    }
    return raw as unknown as T;
  }
  /* ================ Determine if migration is needed ================ */

  const needsMigration = hasRecord && currentVersion !== undefined && recordVersion !== currentVersion;

  /* ================ Compute Value ================ */

  const value: T = (() => {
    // Nothing stored → default
    if (!hasRecord || recordValue === undefined) return defaultRef.current;

    // No version tracking or versions match → deserialize directly
    if (!needsMigration) {
      return deserializeRaw(recordValue);
    }

    // Version mismatch → compute migrated value for display
    const migrateFn = migrateRef.current;
    if (migrateFn) {
      const migrated = migrateFn(recordValue, recordVersion);
      if (migrated !== undefined) return migrated;
    }

    // Migration failed or not provided → default
    return defaultRef.current;
  })();

  // Keep a ref to the current value so functional updates always use the latest
  const valueRef = useRef(value);
  valueRef.current = value;

  /* ================ Persist Migration (one-time side-effect) ================ */

  useEffect(() => {
    if (!needsMigration) return;

    // Prevent double-migration: track by compositeKey + storedVersion
    const migrationId = `${compositeKey}:${String(recordVersion)}→${String(currentVersion)}`;
    if (migrationDone.current === migrationId) return;
    migrationDone.current = migrationId;

    const migrateFn = migrateRef.current;

    if (!migrateFn) {
      // No migration function → remove stale entry
      ctx.remove(namespace, key);
      return;
    }

    if (recordValue === undefined) return;

    const migrated = migrateFn(recordValue, recordVersion);

    if (migrated !== undefined) {
      // Serialize T → JSONValue and persist with current version
      const serialized = serRef.current ? serRef.current.serialize(migrated) : (migrated as unknown as JSONValue);
      ctx.set(namespace, key, serialized, currentVersion);
    } else {
      // Migration returned undefined → discard stale entry
      ctx.remove(namespace, key);
    }
    // Dependencies use primitives only to avoid reference-change loops
  }, [needsMigration, compositeKey, recordVersion, currentVersion]);

  /* ================ Write (supports functional updates) ================ */

  const set: AppStateSetter<T> = useCallback(
    (valueOrUpdater) => {
      const newValue: T = valueOrUpdater instanceof Function ? valueOrUpdater(valueRef.current) : valueOrUpdater;

      const json = serRef.current ? serRef.current.serialize(newValue) : (newValue as unknown as JSONValue);

      if (options?.debounce) {
        ctx.setDebounced(namespace, key, json, options.debounceMs);
      } else {
        ctx.set(namespace, key, json, currentVersion);
      }
    },

    [ctx, compositeKey, options?.debounce, options?.debounceMs, currentVersion]
  );

  const remove = useCallback(() => ctx.remove(namespace, key), [ctx, namespace, key]);

  /* ================ Export ================ */

  const exportJSON = useCallback((): string => {
    const json = serializeValue(value);
    return JSON.stringify(
      {
        _meta: {
          type: "app-state-entry",
          namespace,
          key,
          version: currentVersion,
          exportedAt: new Date().toISOString(),
        },
        value: json,
      },
      null,
      2
    );
  }, [value, compositeKey, currentVersion]);

  /* ================ Import ================ */

  const importJSON = useCallback(
    (input: string): boolean => {
      try {
        const parsed = JSON.parse(input);
        let rawValue: JSONValue;
        let importVersion: number | string | undefined;

        // Detect wrapped format (from exportJSON) vs raw value
        if (parsed._meta && "value" in parsed) {
          rawValue = parsed.value;
          importVersion = parsed._meta.version;
        } else {
          rawValue = parsed;
          importVersion = undefined;
        }

        if (rawValue === undefined || rawValue === null) return false;

        // If imported version differs from current, run migration
        if (currentVersion !== undefined && importVersion !== currentVersion && migrateRef.current) {
          const migrated = migrateRef.current(rawValue, importVersion);
          if (migrated === undefined) return false;
          rawValue = serRef.current ? serRef.current.serialize(migrated) : (migrated as unknown as JSONValue);
        } else {
          // No migration needed — validate that deserialization works
          if (serRef.current) {
            try {
              serRef.current.deserialize(rawValue);
            } catch {
              return false;
            }
          }
        }

        if (options?.debounce) {
          ctx.setDebounced(namespace, key, rawValue, options.debounceMs);
        } else {
          ctx.set(namespace, key, rawValue, currentVersion);
        }
        return true;
      } catch {
        return false;
      }
    },

    [ctx, compositeKey, options?.debounce, options?.debounceMs, currentVersion]
  );

  return { value, set, remove, exportJSON, importJSON };
}

/* ======================== useAppStateNamespace =========================== */

/**
 * Return value of useAppStateNamespace.
 */
export interface UseAppStateNamespaceReturn {
  /** All entries in the namespace as a flat key → value map. */
  entries: Record<string, JSONValue>;

  /** Set a value by key within this namespace. */
  set: (key: string, value: JSONValue) => void;

  /** Remove a single entry by key. */
  remove: (key: string) => void;

  /** Remove all entries in this namespace. */
  removeAll: () => Promise<void>;

  /**
   * Export the entire namespace as a JSON string.
   * Suitable for saving to a file.
   */
  exportJSON: () => Promise<string>;

  /**
   * Import entries into this namespace from a JSON string.
   * Supports the format produced by `exportJSON()` and raw key-value objects.
   *
   * @param json - The JSON string to import.
   * @param options.replace - If true, clears all existing entries before importing.
   * @returns Count of imported entries and list of skipped keys.
   */
  importJSON: (json: string, options?: { replace?: boolean }) => Promise<{ imported: number; skipped: string[] }>;
}

/**
 * Hook for managing all entries in a namespace as raw JSONValue.
 *
 * Use this for bulk operations (export/import/clear all).
 * For typed access to individual keys, use `useAppState<T>` with a serializer.
 *
 * @param namespace - The namespace to manage.
 *
 * @example
 * const { entries, set, remove, removeAll, exportJSON, importJSON } =
 *   useAppStateNamespace("publisher-configs");
 *
 * // List all
 * Object.entries(entries).map(([key, config]) => ...);
 *
 * // Export to file
 * const json = await exportJSON();
 *
 * // Import from file (replace existing)
 * const { imported, skipped } = await importJSON(fileContent, { replace: true });
 */
export function useAppStateNamespace(namespace: string): UseAppStateNamespaceReturn {
  const ctx = useContext(AppStateContext);
  if (!ctx) throw new Error("useAppStateNamespace must be used inside AppStateProvider");

  const entries = ctx.getNamespace(namespace);

  const set = useCallback((k: string, v: JSONValue) => ctx.set(namespace, k, v), [ctx, namespace]);
  const remove = useCallback((k: string) => ctx.remove(namespace, k), [ctx, namespace]);
  const removeAll = useCallback(() => ctx.removeNamespace(namespace), [ctx, namespace]);
  const exportJSON = useCallback(() => ctx.exportState(namespace), [ctx, namespace]);

  const importJSON = useCallback(
    async (json: string, options?: { replace?: boolean }) => {
      return ctx.importState(json, { replace: options?.replace, namespace });
    },
    [ctx, namespace]
  );

  return { entries, set, remove, removeAll, exportJSON, importJSON };
}
