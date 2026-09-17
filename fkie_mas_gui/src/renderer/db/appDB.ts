import { DBSchema, IDBPDatabase, openDB } from "idb";
import { JSONValue } from "@/types";

/* ======================== Types =========================== */

const DB_NAME = "AppDB";
const DB_VERSION = 1;

export const STORE = {
  SETTINGS: "settings",
  STATE: "state",
} as const;

export type StoreName = (typeof STORE)[keyof typeof STORE];

export interface StoreRecord {
  key: string;
  value: JSONValue;
  namespace?: string;
  version?: number | string;
  updatedAt: number;
}

interface AppDBSchema extends DBSchema {
  settings: {
    key: string;
    value: StoreRecord;
  };
  state: {
    key: string;
    value: StoreRecord;
    indexes: { "by-namespace": string };
  };
}

export type { AppDBSchema };

/* ================ Transformer (Encryption-ready) ================ */

export interface ITransformer {
  serialize: (value: JSONValue) => JSONValue | Promise<JSONValue>;
  deserialize: (value: JSONValue) => JSONValue | Promise<JSONValue>;
}

export const identityTransformer: ITransformer = {
  serialize: (v) => v,
  deserialize: (v) => v,
};

/* ================ Persistence Guarantee ================ */

export async function requestPersistentStorage(): Promise<boolean> {
  if (!navigator.storage?.persist) {
    console.warn("[AppDB] navigator.storage.persist() not available.");
    return false;
  }

  const persisted = await navigator.storage.persisted();
  if (persisted) return true;

  const granted = await navigator.storage.persist();
  console.info(`[AppDB] Persistent storage ${granted ? "granted" : "denied"}.`);
  return granted;
}

/* ================ DB Singleton ================ */

let dbInstance: IDBPDatabase<AppDBSchema> | null = null;

export async function initDB(): Promise<IDBPDatabase<AppDBSchema>> {
  if (dbInstance) return dbInstance;

  dbInstance = await openDB<AppDBSchema>(DB_NAME, DB_VERSION, {
    upgrade(db, oldVersion) {
      if (oldVersion < 1) {
        db.createObjectStore(STORE.SETTINGS, { keyPath: "key" });
        const stateStore = db.createObjectStore(STORE.STATE, { keyPath: "key" });
        stateStore.createIndex("by-namespace", "namespace");
      }
    },
  });

  return dbInstance;
}

/* ================ Generic CRUD ================ */

export async function dbGetAll(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  transformer: ITransformer = identityTransformer
): Promise<StoreRecord[]> {
  const raw = await db.getAll(store);
  const results: StoreRecord[] = [];
  for (const record of raw) {
    results.push({ ...record, value: (await transformer.deserialize(record.value)) as JSONValue });
  }
  return results;
}

export async function dbGet(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  key: string,
  transformer: ITransformer = identityTransformer
): Promise<StoreRecord | undefined> {
  const record = await db.get(store, key);
  if (!record) return undefined;
  return { ...record, value: (await transformer.deserialize(record.value)) as JSONValue };
}

export async function dbPut(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  record: StoreRecord,
  transformer: ITransformer = identityTransformer
): Promise<void> {
  await db.put(store, { ...record, value: (await transformer.serialize(record.value)) as JSONValue });
}

export async function dbPutMany(
  db: IDBPDatabase<AppDBSchema>,
  store: StoreName,
  records: StoreRecord[],
  transformer: ITransformer = identityTransformer
): Promise<void> {
  const tx = db.transaction(store, "readwrite");
  for (const record of records) {
    await tx.store.put({ ...record, value: (await transformer.serialize(record.value)) as JSONValue });
  }
  await tx.done;
}

export async function dbDelete(db: IDBPDatabase<AppDBSchema>, store: StoreName, key: string): Promise<void> {
  await db.delete(store, key);
}

export async function dbDeleteMany(db: IDBPDatabase<AppDBSchema>, store: StoreName, keys: string[]): Promise<void> {
  const tx = db.transaction(store, "readwrite");
  for (const key of keys) {
    await tx.store.delete(key);
  }
  await tx.done;
}

export async function dbClear(db: IDBPDatabase<AppDBSchema>, store: StoreName): Promise<void> {
  await db.clear(store);
}

/* ================ Namespace Queries (state store only) ================ */

export async function dbGetByNamespace(
  db: IDBPDatabase<AppDBSchema>,
  namespace: string,
  transformer: ITransformer = identityTransformer
): Promise<StoreRecord[]> {
  const raw = await db.getAllFromIndex(STORE.STATE, "by-namespace", namespace);
  const results: StoreRecord[] = [];
  for (const record of raw) {
    results.push({ ...record, value: (await transformer.deserialize(record.value)) as JSONValue });
  }
  return results;
}

export async function dbDeleteByNamespace(db: IDBPDatabase<AppDBSchema>, namespace: string): Promise<void> {
  const tx = db.transaction(STORE.STATE, "readwrite");
  const index = tx.store.index("by-namespace");
  let cursor = await index.openCursor(namespace);
  while (cursor) {
    await cursor.delete();
    cursor = await cursor.continue();
  }
  await tx.done;
}
