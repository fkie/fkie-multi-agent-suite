import { JSONObject, JSONValue } from "@/types";
import { IDBPDatabase } from "idb";
import React, { createContext, useCallback, useEffect, useMemo, useRef, useState } from "react";

import {
  AppDBSchema,
  dbDelete,
  dbDeleteMany,
  dbGet,
  dbGetAll,
  dbPut,
  dbPutMany,
  identityTransformer,
  initDB,
  ITransformer,
  requestPersistentStorage,
  STORE,
  StoreRecord,
} from "@/renderer/db/appDB";
import {
  broadcast,
  createBroadcastChannel,
  DebounceManager,
  defaultMigrationParser,
  exportFromStore,
  ImportResult,
  importToStore,
} from "@/renderer/db/persistanceCore";
import URI from "@/renderer/models/uris";
import CliArgs from "../assets/cliArgs.json";

/* ======================== Constants =========================== */

export const SETTINGS_VERSION = 1;
const BROADCAST_NAME = "settings-sync";

/* ======================== Settings Definitions =========================== */

export const getDefaultPortFromRos: (
  connectionType: string,
  rosVersion: string,
  ros1MasterUri: string,
  domainId: number
) => number = (connectionType, rosVersion, ros1MasterUri, domainId) => {
  if (connectionType === "crossbar-wamp") {
    return rosVersion === "2" ? 11811 + domainId : 11911 + domainId;
  }
  let uriShift = 0;
  if (rosVersion === "1" && ros1MasterUri && ros1MasterUri !== "default") {
    uriShift = (Number.parseInt(ros1MasterUri.split(":").slice(-1)[0]) - 11311) * 101;
  }
  return rosVersion === "2" ? 35430 + domainId : 35685 + uriShift + domainId;
};

export const LOG_LEVEL_LIST = ["DEBUG", "INFO", "SUCCESS", "WARN", "ERROR"];
export const BUTTON_LOCATIONS = { LEFT: "LEFT", RIGHT: "RIGHT" };
export const LAUNCH_FILE_EXTENSIONS = [".launch", "launch.xml", "launch.py", "launch.yaml", "launch.yml"];

export interface ISettingsParam {
  label?: string;
  default: JSONValue;
  type: string;
  placeholder?: string;
  options?: string | JSONValue[];
  freeSolo?: boolean;
  readOnly?: boolean;
  description?: string;
  group?: string;
  min?: number;
  max?: number;
  cb?: (get: (attr: string) => JSONValue | undefined, set: (attr: string, val: JSONValue) => void) => void;
  validate?: (value: JSONValue) => JSONValue;
  isValid?: (value: JSONValue) => boolean;
}

export const SETTINGS_DEF: Record<string, ISettingsParam> = {
  useDarkMode: {
    label: "Dark mode",
    default: false,
    type: "boolean",
    description: "",
    cb: (get, set) => {
      const v = get("useDarkMode");
      set("color", v ? "#B8E7FB" : "#1a73e8");
      set("backgroundColor", v ? "#424242" : "#fafafa");
    },
    group: "Appearance",
  },
  colorizeHosts: {
    label: "Colorize hosts",
    default: true,
    type: "boolean",
    description: "Each host is assigned a color. Everything related to this host is marked with this color.",
    group: "Appearance",
  },
  showButtonsForKeyModifiers: {
    label: "Show buttons for key modifiers",
    default: false,
    type: "boolean",
    description: "Display buttons for additional functions that are otherwise accessible via key modifiers",
    group: "Appearance",
  },
  buttonLocation: {
    label: "Location of the control buttons",
    type: "string",
    default: BUTTON_LOCATIONS.RIGHT,
    options: [BUTTON_LOCATIONS.RIGHT, BUTTON_LOCATIONS.LEFT],
    description: "",
    group: "Appearance",
  },
  checkForUpdates: { label: "Check for updates on start", default: true, type: "boolean", description: "" },
  fontSizeTerminal: {
    label: "Font size in terminal",
    type: "number",
    default: 14,
    min: 2,
    description: "Only affects the terminal tab.",
    group: "Appearance",
  },
  fontSize: {
    label: "Font size",
    type: "number",
    default: 14,
    min: 2,
    description: "Global font size except terminal.",
    group: "Appearance",
  },
  resetLayout: {
    label: "Reset layout",
    type: "button",
    default: false,
    description: "Restores default layout.",
    group: "Appearance",
  },
  rosVersion: {
    label: "Default ROS version",
    default: CliArgs["ros-version"].default || "2",
    type: "string",
    options: ["1", "2"],
    readOnly: false,
    description:
      "Standard ROS version used to start remote daemon and discovery nodes. Only if automatic detection has failed.",
  },
  debugByUri: {
    label: "Interface URIs",
    type: "string[]",
    default: [URI.ROS_PROVIDER_GET_LIST, URI.ROS_DAEMON_READY, URI.ROS_DISCOVERY_READY],
    options: Object.values(URI).sort(),
    description: "URIs output as debug messages.",
    group: "Logging",
  },
  logPrintToConsole: { label: "Print to console", default: true, type: "boolean", description: "", group: "Logging" },
  openScreenByDefault: {
    label: "Open screen by default",
    default: false,
    type: "boolean",
    description:
      "Double-click behavior on running nodes. If true, double-clicking on a running node opens a screen terminal. Otherwise, the log file is opened. You can reverse the behavior by pressing the Shift key.",
    group: "Logging",
  },
  capabilityGroupParameter: {
    label: "Capability Group parameter",
    freeSolo: true,
    type: "string",
    default: "capability_group",
    description:
      "ROS1 parameter for node grouping. If the ROS node does not have this parameter it use a global one or group according to the namespace.",
    validate: (v) => ((v as string).startsWith("/") ? (v as string).substring(1) : v),
  },
  launchHistoryLength: {
    label: "Launch History Length",
    type: "number",
    default: 5,
    min: 0,
    max: 15,
    description: "Recently loaded files count.",
  },
  logCommand: {
    label: "Log command prefix",
    description: "Terminal command to display the log file. The file name is appended. (+F: waiting for more data)",
    type: "string",
    freeSolo: true,
    default: "while [ ! -f {LOG_FILE} ]; do sleep 1.0; done; /usr/bin/less -fLQR +G +F",
    options: [
      "/usr/bin/less -fLQR +G",
      "/usr/bin/less -fLQR +G +F",
      "while [ ! -f {LOG_FILE} ]; do sleep 1.0; done; /usr/bin/less -fLQR +G +F",
    ],
    group: "Logging",
  },
  color: { label: "Color", type: "none", default: "#1a73e8" },
  backgroundColor: { label: "Color", type: "none", default: "#fafafa" },
  timeDiffThreshold: { label: "Time Diff Threshold [ms]", type: "numberX", default: 500, min: 0 },
  namespaceSystemNodes: { label: "Namespace for system nodes", type: "none", default: "/{SYSTEM}" },
  tooltipEnterDelay: { label: "Tooltip enter delay (ms)", type: "number", default: 500, group: "Appearance" },
  actionOnChangeLaunch: {
    label: "Action on launch file change",
    type: "string",
    default: "ASK",
    options: ["ASK", "DISMISS", "RELOAD"],
    description: "",
  },
  dedicatedTabsFor: {
    label: "Use a dedicated tab for each domain or host",
    type: "string",
    default: "DOMAINS",
    options: ["DOMAINS", "HOSTS"],
    description: "",
  },
  openAsPopout: {
    label: "Open external tabs as popout",
    default: false,
    type: window.commandExecutor ? "undefined": "boolean",
    description: "",
    group: "Window behavior",
  },
  editorOpenLocation: {
    label: "Editor tab location",
    type: "string",
    default: "CENTER",
    options: ["BORDER_TOP", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  nodeLoggerOpenLocation: {
    label: "Log level tab location",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  nodeParamOpenLocation: {
    label: "Node parameter tab location",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  publisherOpenLocation: {
    label: "Publisher tab location",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "BORDER_LEFT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  subscriberOpenLocation: {
    label: "Subscriber tab location",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "BORDER_LEFT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  avoidGroupWithOneItem: { label: "Avoid groups with one item", default: true, type: "boolean", description: "" },
  tabFullName: { label: "Show tab names with namespace", default: true, type: "boolean", description: "" },
  showLaunchFileIndicatorForNodes: {
    label: "Show launchfile indicator",
    default: true,
    type: "boolean",
    description: "",
  },
  delayROSUpdateAfterAction: {
    label: "Delay ROS state update",
    type: "number",
    default: 2,
    min: 0,
    max: 30,
    description: "Delay in seconds after an action (e.g. start/stop nodes) before updating ROS state.",
  },
  spamNodes: {
    label: "Spam Nodes",
    freeSolo: true,
    type: "string",
    default: ".*_impl_,/*_ros2cli",
    options: [
      ".*_impl_,/*_ros2cli",
      ".*_impl_,/*_ros2cli,/mas/*,/_mas_*,ttyd*",
      ".*_impl_,/*_ros2cli,/mas/*,/_mas_*,ttyd*,zenoh-daemon",
    ],
    description: "Nodes placed in {SPAM} group.",
    isValid: (value) => {
      try {
        const splits: string[] = ((value as string) || "").split(",");
        for (const item of splits) {
          try {
            new RegExp(`/(${item})/`);
          } catch (error) {
            console.log(`error while test: ${JSON.stringify(error)}`);
            return false;
          }
        }
        return true;
      } catch {
        return false;
      }
    },
    validate: (v) =>
      ((v as string) || "")
        .split(",")
        .filter((i) => {
          try {
            new RegExp(`/(${i})/`);
            return true;
          } catch {
            return false;
          }
        })
        .join(","),
  },
  ignoreProcessesOnShutdown: {
    label: "Ignore processes on shutdown",
    freeSolo: true,
    type: "string",
    default: "colcon, cmake, CMakeFiles",
    options: ["colcon, cmake, CMakeFiles"],
    description:
      "When closing the GUI, the ROS nodes can also be terminated. Here you can specify which processes containing 'ros2' should not be terminated.",
    isValid: (value) => {
      try {
        const splits: string[] = ((value as string) || "").split(",");
        for (const item of splits) {
          try {
            new RegExp(`/(${item})/`);
          } catch (error) {
            console.log(`error while test: ${JSON.stringify(error)}`);
            return false;
          }
        }
        return true;
      } catch {
        return false;
      }
    },
    validate: (v) =>
      ((v as string) || "")
        .split(",")
        .filter((i) => {
          try {
            new RegExp(`/(${i})/`);
            return true;
          } catch {
            return false;
          }
        })
        .join(","),
  },
  ntpServer: {
    label: "NTP Server",
    freeSolo: true,
    type: "string[]",
    default: ["ntp.ubuntu.com"],
    options: ["ntp.ubuntu.com"],
  },
  editorOpenExternal: {
    label: "Open editor externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  logOpenExternal: {
    label: "Open logs externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  screenOpenExternal: {
    label: "Open screen externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  publisherOpenExternal: {
    label: "Open publisher externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  subscriberOpenExternal: {
    label: "Open subscriber externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  serviceOpenExternal: {
    label: "Open service externally",
    default: false,
    type: "boolean",
    description: "",
    group: "Window behavior",
  },
  showParameterType: {
    label: "Show parameter types",
    default: true,
    type: "boolean",
    description: "",
    group: "hidden",
  },
};

/* ======================== Context Interface =========================== */

export interface ISettingsContext {
  MIN_VERSION_DAEMON: string;
  changed: number;
  isReady: boolean;
  get: (key: string) => JSONValue | undefined;
  getDefault: (key: string) => JSONValue | undefined;
  set: (key: string, value: JSONValue) => void;
  setDebounced: (key: string, value: JSONValue, delayMs?: number) => void;
  resetToDefault: (key: string) => void;
  resetAll: () => Promise<void>;
  getParamList: () => { name: string; param: ISettingsParam }[];
  getChangedCount: () => number;
  exportSettings: () => Promise<string>;
  importSettings: (json: string) => Promise<ImportResult>;
}

/* ======================== Validation =========================== */

function validateValue(value: JSONValue, def: ISettingsParam): JSONValue | undefined {
  switch (def.type) {
    case "number":
    case "numberX": {
      const n = Number(value);
      if (Number.isNaN(n)) return undefined;
      if (def.min !== undefined && n < def.min) return undefined;
      if (def.max !== undefined && n > def.max) return undefined;
      return n;
    }
    case "boolean":
      if (typeof value === "boolean") return value;
      if (value === "true") return true;
      if (value === "false") return false;
      return undefined;
    case "string":
      if (typeof value !== "string") return undefined;
      if (def.options && !def.freeSolo && !(def.options as JSONValue[]).includes(value)) return undefined;
      break;
    case "string[]":
      if (!Array.isArray(value) || !value.every((v) => typeof v === "string")) return undefined;
      break;
    case "none":
    case "button":
      break;
  }

  let result = value;
  if (def.validate) result = def.validate(value);
  if (def.isValid && !def.isValid(result)) return undefined;
  return result;
}

function isDefault(key: string, value: JSONValue): boolean {
  return JSON.stringify(value) === JSON.stringify(SETTINGS_DEF[key]?.default);
}

/* ======================== Provider =========================== */

export const SettingsContext = createContext<ISettingsContext | null>(null);

interface Props {
  children: React.ReactNode;
  transformer?: ITransformer;
}

export function SettingsProvider({ children, transformer }: Props): React.ReactElement {
  const MIN_VERSION_DAEMON = "5.9.0";
  const tx = transformer ?? identityTransformer;
  const txRef = useRef(tx);
  txRef.current = tx;

  const [config, setConfig] = useState<JSONObject>({});
  const [isReady, setIsReady] = useState(false);
  const [changed, setChanged] = useState(0);
  const forceUpdate = useCallback(() => setChanged((c) => c + 1), []);

  const dbRef = useRef<IDBPDatabase<AppDBSchema> | null>(null);
  const debouncer = useRef(new DebounceManager());
  const channelRef = useRef<BroadcastChannel | null>(null);

  /* ================ Broadcast ================ */

  useEffect(() => {
    const ch = createBroadcastChannel(BROADCAST_NAME, (msg) => {
      if (msg.store !== STORE.SETTINGS) return;
      if (msg.type === "changed" && msg.snapshot) {
        setConfig(msg.snapshot as JSONObject);
        forceUpdate();
      } else if (msg.type === "cleared") {
        setConfig({});
        forceUpdate();
      }
    });
    channelRef.current = ch;
    return () => {
      ch.close();
      channelRef.current = null;
    };
  }, [forceUpdate]);

  function notify(snapshot: JSONObject): void {
    broadcast(channelRef.current, { type: "changed", store: STORE.SETTINGS, snapshot });
  }

  /* ================ Init ================ */

  useEffect(() => {
    let cancelled = false;

    (async () => {
      await requestPersistentStorage();
      const db = await initDB();
      if (cancelled) return;
      dbRef.current = db;

      // Auto-migrate localStorage
      const meta = await dbGet(db, STORE.SETTINGS, "__meta:migrated");
      if (!meta) {
        const rawEntry = window.localStorage.getItem("SettingsContext:config");
        if (rawEntry) {
          const parsed = defaultMigrationParser(rawEntry);
          if (parsed && typeof parsed === "object" && !Array.isArray(parsed)) {
            const bulk = parsed as JSONObject;
            const records: StoreRecord[] = [];
            for (const [k, v] of Object.entries(bulk)) {
              if (!(k in SETTINGS_DEF) || v === null || v === undefined) continue;
              const validated = validateValue(v, SETTINGS_DEF[k]);
              if (validated === undefined || isDefault(k, validated)) continue;
              records.push({ key: k, value: validated, version: SETTINGS_VERSION, updatedAt: Date.now() });
            }
            if (records.length > 0) await dbPutMany(db, STORE.SETTINGS, records, txRef.current);
            window.localStorage.removeItem("SettingsContext:config");
          }
        }
        await dbPut(db, STORE.SETTINGS, { key: "__meta:migrated", value: Date.now(), updatedAt: Date.now() });
      }

      // Load into memory
      const all = await dbGetAll(db, STORE.SETTINGS, txRef.current);
      const loaded: JSONObject = {};
      for (const rec of all) {
        if (rec.key.startsWith("__meta:")) continue;
        if (!(rec.key in SETTINGS_DEF)) continue;
        loaded[rec.key] = rec.value;
      }

      if (!cancelled) {
        setConfig(loaded);
        setIsReady(true);
      }
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  /* ================ Persist helpers ================ */

  async function persist(newConfig: JSONObject, changedKeys: string[]): Promise<void> {
    const db = dbRef.current;
    if (!db || changedKeys.length === 0) return;

    const toPut: StoreRecord[] = [];
    const toDelete: string[] = [];

    for (const key of changedKeys) {
      if (key in newConfig) {
        toPut.push({ key, value: newConfig[key], version: SETTINGS_VERSION, updatedAt: Date.now() });
      } else {
        toDelete.push(key);
      }
    }

    if (toPut.length > 0) await dbPutMany(db, STORE.SETTINGS, toPut, txRef.current);
    if (toDelete.length > 0) await dbDeleteMany(db, STORE.SETTINGS, toDelete);
  }

  function computeChanges(prev: JSONObject, next: JSONObject): string[] {
    const keys = new Set([...Object.keys(prev), ...Object.keys(next)]);
    const changed: string[] = [];
    for (const k of keys) {
      if (JSON.stringify(prev[k]) !== JSON.stringify(next[k])) changed.push(k);
    }
    return changed;
  }

  /* ================ Public API ================ */

  const get = useCallback(
    (key: string): JSONValue | undefined => {
      if (key in config) return config[key];
      if (key in SETTINGS_DEF) return SETTINGS_DEF[key].default;
      throw new Error(`Setting "${key}" not found.`);
    },
    [config]
  );

  const getChangedCount = useCallback((): number => {
    return Object.keys(config).length;
  }, [config]);

  const getDefault = useCallback((key: string): JSONValue | undefined => {
    if (key in SETTINGS_DEF) return SETTINGS_DEF[key].default;
    throw new Error(`Setting "${key}" not found.`);
  }, []);

  const set = useCallback(
    (key: string, value: JSONValue): void => {
      if (!SETTINGS_DEF[key]) throw new Error(`Setting "${key}" not found.`);

      const next: JSONObject = { ...config, [key]: value };

      // Run side-effect callbacks
      SETTINGS_DEF[key].cb?.(
        (attr) => (attr in next ? next[attr] : SETTINGS_DEF[attr]?.default),
        (attr, val) => {
          next[attr] = val;
        }
      );

      // Remove defaults
      for (const [k, v] of Object.entries(next)) {
        if (SETTINGS_DEF[k] && isDefault(k, v)) delete next[k];
      }

      const changed = computeChanges(config, next);
      setConfig(next);
      forceUpdate();
      void persist(next, changed);
      notify(next);
    },
    [config, forceUpdate]
  );

  const setDebounced = useCallback(
    (key: string, value: JSONValue, delayMs = 300): void => {
      if (!SETTINGS_DEF[key]) throw new Error(`Setting "${key}" not found.`);

      setConfig((prev) => {
        const next = { ...prev, [key]: value };
        if (isDefault(key, value)) delete next[key];
        notify(next);
        return next;
      });
      forceUpdate();

      debouncer.current.schedule(
        key,
        () => {
          const db = dbRef.current;
          if (!db) return;
          if (isDefault(key, value)) {
            void dbDelete(db, STORE.SETTINGS, key);
          } else {
            void dbPut(
              db,
              STORE.SETTINGS,
              { key, value, version: SETTINGS_VERSION, updatedAt: Date.now() },
              txRef.current
            );
          }
        },
        delayMs
      );
    },
    [forceUpdate]
  );

  const resetToDefault = useCallback(
    (key: string): void => {
      if (!SETTINGS_DEF[key]) throw new Error(`Setting "${key}" not found.`);

      const next = { ...config };
      delete next[key];

      // Run callback with default
      SETTINGS_DEF[key].cb?.(
        (attr) => (attr in next ? next[attr] : SETTINGS_DEF[attr]?.default),
        (attr, val) => {
          if (!isDefault(attr, val)) next[attr] = val;
          else delete next[attr];
        }
      );

      const changed = computeChanges(config, next);
      setConfig(next);
      forceUpdate();
      void persist(next, changed);
      notify(next);
    },
    [config, forceUpdate]
  );

  const resetAll = useCallback(async (): Promise<void> => {
    if (dbRef.current) {
      // Keep meta keys
      const all = await dbGetAll(dbRef.current, STORE.SETTINGS);
      const toDelete = all.filter((r) => !r.key.startsWith("__meta:")).map((r) => r.key);
      if (toDelete.length > 0) await dbDeleteMany(dbRef.current, STORE.SETTINGS, toDelete);
    }
    setConfig({});
    forceUpdate();
    broadcast(channelRef.current, { type: "cleared", store: STORE.SETTINGS });
  }, [forceUpdate]);

  const exportSettings = useCallback(async (): Promise<string> => {
    if (!dbRef.current)
      return JSON.stringify(
        {
          _meta: {
            type: "settings",
            version: SETTINGS_VERSION,
            exportedAt: new Date().toISOString(),
            appVersion: window.APP_VERSION ?? "unknown",
          },
          data: config,
        },
        null,
        2
      );
    return exportFromStore(
      dbRef.current,
      STORE.SETTINGS,
      SETTINGS_VERSION,
      txRef.current,
      (r) => !r.key.startsWith("__meta:")
    );
  }, [config]);

  const importSettings = useCallback(
    async (json: string): Promise<ImportResult> => {
      if (!dbRef.current) throw new Error("DB not ready");

      const result = await importToStore(dbRef.current, STORE.SETTINGS, json, {
        replace: true,
        version: SETTINGS_VERSION,
        transformer: txRef.current,
        validate: (key, value) => {
          if (!(key in SETTINGS_DEF)) return undefined;
          const validated = validateValue(value, SETTINGS_DEF[key]);
          if (validated === undefined || isDefault(key, validated)) return undefined;
          return validated;
        },
      });

      // Reload
      const all = await dbGetAll(dbRef.current, STORE.SETTINGS, txRef.current);
      const loaded: JSONObject = {};
      for (const rec of all) {
        if (rec.key.startsWith("__meta:")) continue;
        if (!(rec.key in SETTINGS_DEF)) continue;
        loaded[rec.key] = rec.value;
      }
      setConfig(loaded);
      forceUpdate();
      notify(loaded);

      return result;
    },
    [forceUpdate]
  );

  const getParamList = useCallback(
    () => Object.keys(SETTINGS_DEF).map((name) => ({ name, param: SETTINGS_DEF[name] })),
    []
  );

  /* ================ Memo ================ */

  const ctx = useMemo<ISettingsContext>(
    () => ({
      MIN_VERSION_DAEMON,
      changed,
      isReady,
      get,
      getDefault,
      set,
      setDebounced,
      resetToDefault,
      resetAll,
      getParamList,
      getChangedCount,
      exportSettings,
      importSettings,
    }),
    [
      changed,
      isReady,
      get,
      getDefault,
      set,
      setDebounced,
      resetToDefault,
      resetAll,
      getParamList,
      getChangedCount,
      exportSettings,
      importSettings,
    ]
  );

  /* ================ Cleanup ================ */

  useEffect(
    () => () => {
      debouncer.current.flush();
    },
    []
  );

  return <SettingsContext.Provider value={ctx}>{children}</SettingsContext.Provider>;
}

export default SettingsProvider;
