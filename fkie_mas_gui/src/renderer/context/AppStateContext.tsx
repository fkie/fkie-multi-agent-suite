import { IDBPDatabase } from "idb";
import React, { createContext, useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  AppDBSchema,
  dbClear,
  dbDelete,
  dbDeleteByNamespace,
  dbGetAll,
  dbPut,
  ITransformer,
  identityTransformer,
  initDB,
  requestPersistentStorage,
  STORE,
} from "@/renderer/db/appDB";
import {
  broadcast,
  createBroadcastChannel,
  DebounceManager,
  exportFromStore,
  ImportResult,
  importToStore,
  MigrationEntry,
  migrateLocalStorageEntries,
} from "@/renderer/db/persistanceCore";
import { JSONValue } from "@/types";

/* ======================== Constants =========================== */

export const APP_STATE_VERSION = 1;
const BROADCAST_NAME = "app-state-sync";

/* ======================== Types =========================== */

export interface AppStateEntry {
  value: JSONValue;
  version?: number | string;
}

export interface IAppStateContext {
  isReady: boolean;
  get: (namespace: string, key: string) => JSONValue | undefined;
  getRecord: (namespace: string, key: string) => AppStateEntry | undefined;
  set: (namespace: string, key: string, value: JSONValue, version?: number | string) => void;
  setDebounced: (namespace: string, key: string, value: JSONValue, delayMs?: number) => void;
  remove: (namespace: string, key: string) => void;
  removeNamespace: (namespace: string) => Promise<void>;
  clearAll: () => Promise<void>;
  getNamespace: (namespace: string) => Record<string, JSONValue>;
  getAllNamespaces: () => string[];
  exportState: (namespace?: string) => Promise<string>;
  importState: (json: string, options?: { replace?: boolean; namespace?: string }) => Promise<ImportResult>;
  registerMigrations: (entries: MigrationEntry[]) => void;
  migrateFromLocalStorage: () => Promise<{ migrated: string[]; failed: string[] }>;
}

/* ======================== Helpers =========================== */

function compositeKey(namespace: string, key: string): string {
  return `${namespace}:${key}`;
}

function parseCompositeKey(ck: string): { namespace: string; key: string } {
  const i = ck.indexOf(":");
  return i === -1 ? { namespace: "", key: ck } : { namespace: ck.slice(0, i), key: ck.slice(i + 1) };
}

/* ======================== Internal State =========================== */

type StateMap = Record<string, AppStateEntry>;

/* ======================== Provider =========================== */

export const AppStateContext = createContext<IAppStateContext | null>(null);

interface Props {
  children: React.ReactNode;
  transformer?: ITransformer;
  migrations?: MigrationEntry[];
}

export function AppStateProvider({ children, transformer, migrations: initialMigrations }: Props): React.ReactElement {
  const tx = transformer ?? identityTransformer;
  const txRef = useRef(tx);
  txRef.current = tx;

  const [stateMap, setStateMap] = useState<StateMap>({});
  const [isReady, setIsReady] = useState(false);
  const [, setTick] = useState(0);
  const forceUpdate = useCallback(() => setTick((t) => t + 1), []);

  const dbRef = useRef<IDBPDatabase<AppDBSchema> | null>(null);
  const debouncer = useRef(new DebounceManager());
  const channelRef = useRef<BroadcastChannel | null>(null);
  const migrationEntries = useRef<MigrationEntry[]>(initialMigrations ?? []);

  // Track which localStorage keys have been migrated (prevents re-migration)
  const migratedKeys = useRef<Set<string>>(new Set());

  /* ================ Broadcast ================ */

  useEffect(() => {
    const ch = createBroadcastChannel(BROADCAST_NAME, (msg) => {
      if (msg.store !== STORE.STATE) return;
      if (msg.type === "changed" && msg.snapshot) {
        setStateMap(msg.snapshot as unknown as StateMap);
        forceUpdate();
      } else if (msg.type === "cleared") {
        setStateMap({});
        forceUpdate();
      }
    });
    channelRef.current = ch;
    return () => {
      ch.close();
      channelRef.current = null;
    };
  }, [forceUpdate]);

  function notifyAll(snapshot: StateMap): void {
    broadcast(channelRef.current, {
      type: "changed",
      store: STORE.STATE,
      snapshot: snapshot as unknown as Record<string, JSONValue>,
    });
  }

  /* ================ Pending Migrations Runner ================ */

  /**
   * Runs migration for all registered entries that haven't been processed yet.
   * - Skips entries where DB already has data for that key.
   * - Removes processed entries from the registry (one-shot).
   * Safe to call multiple times.
   */
  async function runPendingMigrations(): Promise<boolean> {
    const db = dbRef.current;
    if (!db) return false;

    const pending = migrationEntries.current.filter((entry) => !migratedKeys.current.has(entry.localStorageKey));

    if (pending.length === 0) return false;

    const { migrated, skipped, failed } = await migrateLocalStorageEntries(db, STORE.STATE, pending, txRef.current);

    if (migrated.length > 0 || failed.length > 0) {
      console.info("[AppStateContext] Migration results:", { migrated, skipped, failed });
    }

    // Mark all pending entries as processed (migrated, skipped, or failed)
    for (const entry of pending) {
      migratedKeys.current.add(entry.localStorageKey);
    }

    // Remove processed entries from registry (they are one-shot)
    const processedKeys = new Set(pending.map((e) => e.localStorageKey));
    migrationEntries.current = migrationEntries.current.filter((entry) => !processedKeys.has(entry.localStorageKey));

    // If anything was actually written to DB, reload into memory
    if (migrated.length > 0) {
      const all = await dbGetAll(db, STORE.STATE, txRef.current);
      const loaded: StateMap = {};
      for (const rec of all) {
        if (rec.key.startsWith("__meta:")) continue;
        loaded[rec.key] = { value: rec.value, version: rec.version };
      }
      setStateMap(loaded);
      forceUpdate();
      notifyAll(loaded);
      return true;
    }

    return false;
  }
  /* ================ Init ================ */

  useEffect(() => {
    let cancelled = false;

    (async () => {
      await requestPersistentStorage();
      const db = await initDB();
      if (cancelled) return;
      dbRef.current = db;

      // Run any migrations that were passed as props (initialMigrations)
      await runPendingMigrations();

      // Load all state into memory
      const all = await dbGetAll(db, STORE.STATE, txRef.current);
      const loaded: StateMap = {};
      for (const rec of all) {
        if (rec.key.startsWith("__meta:")) continue;
        loaded[rec.key] = { value: rec.value, version: rec.version };
      }

      if (!cancelled) {
        setStateMap(loaded);
        setIsReady(true);
      }
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  /* ================ API ================ */

  const get = useCallback(
    (namespace: string, key: string): JSONValue | undefined => {
      return stateMap[compositeKey(namespace, key)]?.value;
    },
    [stateMap]
  );

  const getRecord = useCallback(
    (namespace: string, key: string): AppStateEntry | undefined => {
      return stateMap[compositeKey(namespace, key)];
    },
    [stateMap]
  );

  const getNamespace = useCallback(
    (namespace: string): Record<string, JSONValue> => {
      const prefix = `${namespace}:`;
      const result: Record<string, JSONValue> = {};
      for (const [ck, entry] of Object.entries(stateMap)) {
        if (ck.startsWith(prefix)) result[ck.slice(prefix.length)] = entry.value;
      }
      return result;
    },
    [stateMap]
  );

  const getAllNamespaces = useCallback((): string[] => {
    const ns = new Set<string>();
    for (const ck of Object.keys(stateMap)) {
      const { namespace } = parseCompositeKey(ck);
      if (namespace && namespace !== "__meta") ns.add(namespace);
    }
    return [...ns].sort();
  }, [stateMap]);

  const set = useCallback(
    (namespace: string, key: string, value: JSONValue, version?: number | string): void => {
      const ck = compositeKey(namespace, key);
      const v = version ?? APP_STATE_VERSION;
      setStateMap((prev) => {
        const n = { ...prev, [ck]: { value, version: v } };
        notifyAll(n);
        return n;
      });
      forceUpdate();

      if (dbRef.current) {
        void dbPut(
          dbRef.current,
          STORE.STATE,
          { key: ck, namespace, value, version: v, updatedAt: Date.now() },
          txRef.current
        );
      }
    },
    [forceUpdate]
  );

  const setDebounced = useCallback(
    (namespace: string, key: string, value: JSONValue, delayMs = 300): void => {
      const ck = compositeKey(namespace, key);
      setStateMap((prev) => {
        const n = { ...prev, [ck]: { value, version: APP_STATE_VERSION } };
        notifyAll(n);
        return n;
      });
      forceUpdate();

      debouncer.current.schedule(
        ck,
        () => {
          if (!dbRef.current) return;
          void dbPut(
            dbRef.current,
            STORE.STATE,
            { key: ck, namespace, value, version: APP_STATE_VERSION, updatedAt: Date.now() },
            txRef.current
          );
        },
        delayMs
      );
    },
    [forceUpdate]
  );

  const remove = useCallback(
    (namespace: string, key: string): void => {
      const ck = compositeKey(namespace, key);
      setStateMap((prev) => {
        const n = { ...prev };
        delete n[ck];
        notifyAll(n);
        return n;
      });
      forceUpdate();
      if (dbRef.current) void dbDelete(dbRef.current, STORE.STATE, ck);
    },
    [forceUpdate]
  );

  const removeNamespace = useCallback(
    async (namespace: string): Promise<void> => {
      const prefix = `${namespace}:`;
      setStateMap((prev) => {
        const n: StateMap = {};
        for (const [k, v] of Object.entries(prev)) {
          if (!k.startsWith(prefix)) n[k] = v;
        }
        notifyAll(n);
        return n;
      });
      forceUpdate();
      if (dbRef.current) await dbDeleteByNamespace(dbRef.current, namespace);
    },
    [forceUpdate]
  );

  const clearAll = useCallback(async (): Promise<void> => {
    setStateMap({});
    forceUpdate();
    if (dbRef.current) await dbClear(dbRef.current, STORE.STATE);
    broadcast(channelRef.current, { type: "cleared", store: STORE.STATE });
  }, [forceUpdate]);

  const exportState = useCallback(
    async (namespace?: string): Promise<string> => {
      if (!dbRef.current) {
        const data: Record<string, JSONValue> = {};
        for (const [k, entry] of Object.entries(stateMap)) data[k] = entry.value;
        return JSON.stringify(
          {
            _meta: {
              type: "state",
              version: APP_STATE_VERSION,
              exportedAt: new Date().toISOString(),
              appVersion: window.APP_VERSION ?? "unknown",
            },
            data,
          },
          null,
          2
        );
      }
      return exportFromStore(
        dbRef.current,
        STORE.STATE,
        APP_STATE_VERSION,
        txRef.current,
        namespace ? (r) => r.namespace === namespace : (r) => !r.key.startsWith("__meta:")
      );
    },
    [stateMap]
  );

  const importState = useCallback(
    async (json: string, options?: { replace?: boolean; namespace?: string }): Promise<ImportResult> => {
      if (!dbRef.current) throw new Error("DB not ready");

      const result = await importToStore(dbRef.current, STORE.STATE, json, {
        replace: options?.replace,
        version: APP_STATE_VERSION,
        namespace: options?.namespace,
        transformer: txRef.current,
      });

      // Reload
      const all = await dbGetAll(dbRef.current, STORE.STATE, txRef.current);
      const loaded: StateMap = {};
      for (const rec of all) {
        if (!rec.key.startsWith("__meta:")) loaded[rec.key] = { value: rec.value, version: rec.version };
      }
      setStateMap(loaded);
      forceUpdate();
      notifyAll(loaded);

      return result;
    },
    [forceUpdate]
  );

  /* ================ Register Migrations (runtime) ================ */

  const registerMigrations = useCallback((entries: MigrationEntry[]): void => {
    // Filter out already-processed entries
    const newEntries = entries.filter((e) => !migratedKeys.current.has(e.localStorageKey));
    if (newEntries.length === 0) return;

    // Add to registry
    migrationEntries.current = [...migrationEntries.current, ...newEntries];

    // Immediately run migration for newly registered entries
    if (dbRef.current) {
      void runPendingMigrations();
    }
  }, []);

  /* ================ Manual Migration Trigger ================ */

  const migrateFromLocalStorage = useCallback(async (): Promise<{ migrated: string[]; failed: string[] }> => {
    if (!dbRef.current) return { migrated: [], failed: ["DB not initialized"] };

    // Reset tracking to force re-attempt
    migratedKeys.current.clear();

    const result = await migrateLocalStorageEntries(
      dbRef.current,
      STORE.STATE,
      migrationEntries.current,
      txRef.current
    );

    // Mark all as done
    for (const entry of migrationEntries.current) {
      migratedKeys.current.add(entry.localStorageKey);
    }

    const all = await dbGetAll(dbRef.current, STORE.STATE, txRef.current);
    const loaded: StateMap = {};
    for (const rec of all) {
      if (!rec.key.startsWith("__meta:")) loaded[rec.key] = { value: rec.value, version: rec.version };
    }
    setStateMap(loaded);
    forceUpdate();

    return result;
  }, [forceUpdate]);

  /* ================ Cleanup ================ */

  useEffect(
    () => () => {
      debouncer.current.flush();
    },
    []
  );

  /* ================ Context ================ */

  const ctx = useMemo<IAppStateContext>(
    () => ({
      isReady,
      get,
      getRecord,
      set,
      setDebounced,
      remove,
      removeNamespace,
      clearAll,
      getNamespace,
      getAllNamespaces,
      exportState,
      importState,
      registerMigrations,
      migrateFromLocalStorage,
    }),
    [
      isReady,
      get,
      getRecord,
      set,
      setDebounced,
      remove,
      removeNamespace,
      clearAll,
      getNamespace,
      getAllNamespaces,
      exportState,
      importState,
      registerMigrations,
      migrateFromLocalStorage,
    ]
  );

  return <AppStateContext.Provider value={ctx}>{children}</AppStateContext.Provider>;
}

export default AppStateProvider;
