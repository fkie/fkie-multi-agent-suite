import { DBSchema, IDBPDatabase, openDB } from "idb";
import React, { createContext, useCallback, useContext, useEffect, useRef, useState } from "react";

import { TRosMessageStruct } from "@/types";
import useLocalStorage from "../hooks/useLocalStorage";

/* ======================== Types =========================== */

export type TMsgHistoryEntry = {
  messageType: string;
  id: number;
  name: string;
  favorite: boolean;
  rate: string;
  skw: boolean;
  data: TRosMessageStruct;
  createdAt: number;
};

interface MsgHistoryDB extends DBSchema {
  history: {
    key: [string, number];
    value: TMsgHistoryEntry;
  };
}

interface MsgHistoryContextType {
  addEntry: (entry: Omit<TMsgHistoryEntry, "id">) => Promise<TMsgHistoryEntry>;
  updateEntryMeta: (messageType: string, id: number, updates: { name?: string; favorite?: boolean }) => Promise<void>;
  deleteEntry: (messageType: string, id: number) => Promise<void>;
  ensureLoaded: (messageType: string) => Promise<void>;
  setMaxEntries: (max: number) => Promise<void>;
  historyByType: Record<string, TMsgHistoryEntry[]>;
  maxEntries: number;
}

/* ====================== DB Setup ========================== */

const DB_NAME = "MsgHistoryDB";
const STORE_NAME = "history";
const DEFAULT_MAX = 5;
export const DB_MAX_MSGS = 10; // we store always 10 entries

async function initDB(): Promise<IDBPDatabase<MsgHistoryDB>> {
  return openDB<MsgHistoryDB>(DB_NAME, 1, {
    upgrade(db) {
      db.createObjectStore(STORE_NAME, { keyPath: ["messageType", "id"] });
    },
  });
}

/* ====================== Context =========================== */

export const MsgHistoryContext = createContext<MsgHistoryContextType | null>(null);

export const MsgHistoryProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [db, setDb] = useState<IDBPDatabase<MsgHistoryDB> | null>(null);

  const [historyByType, setHistoryByType] = useState<Record<string, TMsgHistoryEntry[]>>({});

  const [maxEntries, setMaxEntriesState] = useLocalStorage<number>("MessageHistory:maxEntries", DEFAULT_MAX);

  const loadedTypes = useRef<Set<string>>(new Set());
  const loadingTypes = useRef<Set<string>>(new Set());

  useEffect(() => {
    initDB().then(setDb);
  }, []);

  useEffect(() => {
    updateAfterMaxEntries()
  }, [maxEntries]);

  /* ================= Internal Loader ======================= */

  const loadFromDB = useCallback(
    async (messageType: string) => {
      if (!db) return;

      const tx = db.transaction(STORE_NAME, "readonly");
      const store = tx.objectStore(STORE_NAME);

      const range = IDBKeyRange.bound([messageType, 0], [messageType, Number.MAX_SAFE_INTEGER]);

      const result: TMsgHistoryEntry[] = [];
      let cursor = await store.openCursor(range);

      while (cursor) {
        result.push(cursor.value);
        cursor = await cursor.continue();
      }

      result.sort((a, b) => {
        if (a.favorite !== b.favorite) return a.favorite ? -1 : 1;
        return b.id - a.id;
      });
      setHistoryByType((prev) => ({
        ...prev,
        [messageType]: result.slice(0, maxEntries),
      }));

      loadedTypes.current.add(messageType);
    },
    [db, maxEntries]
  );

  /* ================= Lazy Hydration ======================== */

  const ensureLoaded = useCallback(
    async (messageType: string) => {
      if (!db) return;
      if (loadedTypes.current.has(messageType)) return;
      if (loadingTypes.current.has(messageType)) return;

      loadingTypes.current.add(messageType);
      await loadFromDB(messageType);
      loadingTypes.current.delete(messageType);
    },
    [db, loadFromDB]
  );

  /* ====================== Add Entry ======================== */

  const addEntry = async (entryWithoutId: Omit<TMsgHistoryEntry, "id">): Promise<TMsgHistoryEntry> => {
    if (!db) throw new Error("DB not initialized");

    const tx = db.transaction(STORE_NAME, "readwrite");
    const store = tx.objectStore(STORE_NAME);
    const { messageType } = entryWithoutId;

    const range = IDBKeyRange.bound([messageType, 0], [messageType, Number.MAX_SAFE_INTEGER]);

    let maxId = 0;
    const all: TMsgHistoryEntry[] = [];

    let cursor = await store.openCursor(range);
    while (cursor) {
      all.push(cursor.value);
      if (cursor.value.id > maxId) maxId = cursor.value.id;
      cursor = await cursor.continue();
    }

    const newId = maxId + 1;
    const newEntry: TMsgHistoryEntry = { ...entryWithoutId, id: newId };

    await store.put(newEntry);

    all.push(newEntry);
    all.sort((a, b) => {
      if (a.favorite !== b.favorite) return a.favorite ? 1 : -1;
      return a.id - b.id; // remove oldest first
    });

    while (all.length > DB_MAX_MSGS) {
      const toDelete = all.shift();
      if (toDelete) await store.delete([toDelete.messageType, toDelete.id]);
    }

    await tx.done;

    if (loadedTypes.current.has(messageType)) await loadFromDB(messageType);

    return newEntry;
  };

  /* =================== Update Metadata ===================== */

  const updateEntryMeta = async (messageType: string, id: number, updates: { name?: string; favorite?: boolean }) => {
    if (!db) return;

    const key: [string, number] = [messageType, id];
    const existing = await db.get(STORE_NAME, key);
    if (!existing) return;

    const updated = {
      ...existing,
      ...(updates.name !== undefined && { name: updates.name }),
      ...(updates.favorite !== undefined && { favorite: updates.favorite }),
    };

    await db.put(STORE_NAME, updated);

    if (loadedTypes.current.has(messageType)) await loadFromDB(messageType);
  };

  /* ======================= Delete ========================== */

  const deleteEntry = async (messageType: string, id: number) => {
    if (!db) return;

    await db.delete(STORE_NAME, [messageType, id]);

    if (loadedTypes.current.has(messageType)) await loadFromDB(messageType);
  };

  /* ===================== Global Max ======================== */

  const setMaxEntriesGlobal = async (max: number) => {
    if (max === 0 || maxEntries === max) return;

    setMaxEntriesState(max);

    // nur UI neu laden
    for (const type of loadedTypes.current) {
      await loadFromDB(type);
    }
  };

  const updateAfterMaxEntries = useCallback(async () => {
    // nur UI neu laden
    for (const type of loadedTypes.current) {
      await loadFromDB(type);
    }
  }, [maxEntries]);

  return (
    <MsgHistoryContext.Provider
      value={{
        maxEntries,
        addEntry,
        updateEntryMeta,
        deleteEntry,
        ensureLoaded,
        setMaxEntries: setMaxEntriesGlobal,
        historyByType,
      }}
    >
      {children}
    </MsgHistoryContext.Provider>
  );
};

/* ======================== Hook ============================ */

export const useMsgHistory = () => {
  const ctx = useContext(MsgHistoryContext);
  if (!ctx) throw new Error("useMsgHistory must be used inside MsgHistoryProvider");
  return ctx;
};
