import { IDBPDatabase } from "idb";
import {
  AppDBSchema,
  dbClear,
  dbDeleteByNamespace,
  dbGetAll,
  dbPut,
  dbPutMany,
  ITransformer,
  identityTransformer,
  StoreName,
  StoreRecord,
} from "@/renderer/db/appDB";
import { JSONValue } from "@/types";

/* ======================== Tab Identity =========================== */

export const TAB_ID = `${Date.now()}-${Math.random().toString(36).slice(2)}`;

/* ======================== Broadcast =========================== */

export interface BroadcastPayload {
  type: "changed" | "cleared";
  store: StoreName;
  keys?: string[];
  snapshot?: unknown; // Record<string, JSONValue>;
  senderId: string;
}

export function createBroadcastChannel(name: string, onMessage: (payload: BroadcastPayload) => void): BroadcastChannel {
  const channel = new BroadcastChannel(name);

  channel.onmessage = (event: MessageEvent<BroadcastPayload>) => {
    const msg = event.data;
    if (msg.senderId === TAB_ID) return;
    onMessage(msg);
  };

  return channel;
}

export function broadcast(channel: BroadcastChannel | null, payload: Omit<BroadcastPayload, "senderId">): void {
  channel?.postMessage({ ...payload, senderId: TAB_ID });
}

/* ======================== Debounce Manager =========================== */

export class DebounceManager {
  private timers = new Map<string, ReturnType<typeof setTimeout>>();

  schedule(key: string, fn: () => void, delayMs: number): void {
    this.cancel(key);
    const timer = setTimeout(() => {
      this.timers.delete(key);
      fn();
    }, delayMs);
    this.timers.set(key, timer);
  }

  cancel(key: string): void {
    const existing = this.timers.get(key);
    if (existing) {
      clearTimeout(existing);
      this.timers.delete(key);
    }
  }

  flush(): void {
    for (const timer of this.timers.values()) {
      clearTimeout(timer);
    }
    this.timers.clear();
  }
}

/* ======================== localStorage Migration =========================== */

export interface MigrationEntry {
  /** localStorage key to read. */
  localStorageKey: string;
  /** Target key in DB. */
  dbKey: string;
  /** For state store: namespace. */
  namespace?: string;
  /** Optional parser. Default: JSON.parse with useLocalStorage wrapper detection. */
  parse?: (raw: string) => JSONValue | undefined;
  /** Version to assign. */
  version?: number | string;
}

/**
 * Default parser that handles raw JSON and the versioned wrapper
 * format used by the old useLocalStorage hook.
 */
export function defaultMigrationParser(raw: string): JSONValue | undefined {
  try {
    const parsed = JSON.parse(raw);
    if (typeof parsed === "object" && parsed !== null && "value" in parsed) {
      return (parsed as { value: JSONValue }).value;
    }
    return parsed as JSONValue;
  } catch {
    return undefined;
  }
}

export async function migrateLocalStorageEntries(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  entries: MigrationEntry[],
  transformer: ITransformer = identityTransformer
): Promise<{ migrated: string[]; skipped: string[]; failed: string[] }> {
  const migrated: string[] = [];
  const skipped: string[] = [];
  const failed: string[] = [];

  for (const entry of entries) {
    try {
      // Skip if DB already has an entry for this key
      const existing = await db.get(store, entry.dbKey);
      if (existing) {
        skipped.push(entry.localStorageKey);
        continue;
      }

      const raw = window.localStorage.getItem(entry.localStorageKey);
      if (raw === null) {
        skipped.push(entry.localStorageKey);
        continue;
      }

      const parser = entry.parse ?? defaultMigrationParser;
      const value = parser(raw);

      if (value === undefined || value === null) {
        failed.push(entry.localStorageKey);
        continue;
      }

      const record: StoreRecord = {
        key: entry.dbKey,
        value,
        namespace: entry.namespace,
        version: entry.version,
        updatedAt: Date.now(),
      };

      await dbPut(db, store, record, transformer);
      window.localStorage.removeItem(entry.localStorageKey);
      migrated.push(entry.localStorageKey);
    } catch (error) {
      console.warn(`[Migration] Failed for "${entry.localStorageKey}":`, error);
      failed.push(entry.localStorageKey);
    }
  }

  return { migrated, skipped, failed };
}

/* ======================== Export / Import =========================== */

export interface ExportPayload {
  _meta: {
    type: string;
    version: number | string;
    exportedAt: string;
    appVersion: string;
    filter?: string;
  };
  data: Record<string, JSONValue>;
}

export async function exportFromStore(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  version: number | string,
  transformer: ITransformer = identityTransformer,
  filter?: (record: StoreRecord) => boolean
): Promise<string> {
  const records = await dbGetAll(db, store, transformer);
  const data: Record<string, JSONValue> = {};

  for (const record of records) {
    if (filter && !filter(record)) continue;
    data[record.key] = record.value;
  }

  const payload: ExportPayload = {
    _meta: {
      type: store,
      version,
      exportedAt: new Date().toISOString(),
      appVersion: window.APP_VERSION ?? "unknown",
    },
    data,
  };

  return JSON.stringify(payload, null, 2);
}

export interface ImportResult {
  imported: number;
  skipped: string[];
}

/**
 * Import key-value data into a DB store.
 *
 * Supports two input formats:
 * - Wrapped: `{ _meta: {...}, data: { key: value, ... } }`
 * - Raw: `{ key: value, ... }`
 *
 * When `namespace` is provided:
 * - Keys are prefixed with "namespace:" to form composite keys
 * - `replace: true` only clears entries in that namespace (not the whole store)
 *
 * When `namespace` is NOT provided:
 * - Keys are stored as-is
 * - `replace: true` clears the entire store
 */
export async function importToStore(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  json: string,
  options: {
    replace?: boolean;
    version: number | string;
    validate?: (key: string, value: JSONValue) => JSONValue | undefined;
    namespace?: string;
    transformer?: ITransformer;
  }
): Promise<ImportResult> {
  const parsed = JSON.parse(json);
  const transformer = options.transformer ?? identityTransformer;
  const skipped: string[] = [];

  let data: Record<string, JSONValue>;

  if (parsed._meta && parsed.data) {
    data = parsed.data as Record<string, JSONValue>;
  } else {
    data = parsed as Record<string, JSONValue>;
  }

  const records: StoreRecord[] = [];

  for (const [key, value] of Object.entries(data)) {
    if (value === undefined || value === null) {
      skipped.push(key);
      continue;
    }

    const validated = options.validate ? options.validate(key, value) : value;
    if (validated === undefined) {
      skipped.push(key);
      continue;
    }

    // Build composite key: "namespace:key" or just "key"
    const dbKey = options.namespace ? `${options.namespace}:${key}` : key;

    records.push({
      key: dbKey,
      value: validated,
      namespace: options.namespace,
      version: options.version,
      updatedAt: Date.now(),
    });
  }

  // Handle replace
  if (options.replace) {
    if (options.namespace) {
      // Only clear entries belonging to this namespace
      await dbDeleteByNamespace(db, options.namespace);
    } else {
      // Clear entire store (settings import, or namespace-less state)
      await dbClear(db, store);
    }
  }

  if (records.length > 0) {
    await dbPutMany(db, store, records, transformer);
  }

  return { imported: records.length, skipped };
}

/* ======================== Helpers =========================== */

/**
 * Build records from a flat config object (used by SettingsContext).
 */
export function configToRecords(config: Record<string, JSONValue>, version: number | string): StoreRecord[] {
  return Object.entries(config).map(([key, value]) => ({
    key,
    value,
    version,
    updatedAt: Date.now(),
  }));
}

/**
 * Build a flat config object from records.
 */
export function recordsToConfig(records: StoreRecord[]): Record<string, JSONValue> {
  const result: Record<string, JSONValue> = {};
  for (const record of records) {
    result[record.key] = record.value;
  }
  return result;
}
