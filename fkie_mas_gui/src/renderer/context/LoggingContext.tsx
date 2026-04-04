import React, { createContext, useCallback, useEffect, useMemo, useState } from "react";

import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { LogEvent, LoggingLevel } from "@/renderer/models";
import {
    sendStateError,
    sendStateInfo,
    sendStateSuccess,
    sendStateWarn,
} from "@/renderer/pages/NodeManager/layout/events";
import { JSONObject, TResult } from "@/types";

export interface ILoggingContext {
  logs: LogEvent[];
  countErrors: number;
  debug: (description: string, details?: string) => void;
  info: (description: string, details?: string, stateInfo?: string) => void;
  success: (description: string, details?: string, stateInfo?: string) => void;
  warn: (description: string, details?: string, stateInfo?: string) => void;
  error: (description: string, details?: string, stateInfo?: string) => void;
  clearLogs: () => void;
  debugInterface: (uri: string, result: TResult | JSONObject | string, details?: string, providerName?: string) => void;
}

export interface ILoggingProvider {
  children: React.ReactNode;
}
export const DEFAULT_LOGGING = {
  logs: [],
  countErrors: 0,
  debug: (): void => {},
  info: (): void => {},
  success: (): void => {},
  warn: (): void => {},
  error: (): void => {},
  clearLogs: (): void => {},
  debugInterface: (): void => {},
};

// TODO Add valid github issue repository

// This text should appear every time a "bug" is found (Things that the user can not control/configure)
export const DEFAULT_BUG_TEXT = "A bug occurred, please consider report it as an issue on our github repository";

export const LoggingContext = createContext<ILoggingContext>(DEFAULT_LOGGING);

export function LoggingProvider({ children }: ILoggingProvider) {
  const settingsCtx = useSettingsContext();

  const [logs, setLogs] = useState<LogEvent[]>([]);
  const [countErrors, setCountErrors] = useState(0);

  const [debugByUri, setDebugByUri] = useState<string[]>([]);
  const [printToConsole, setPrintToConsole] = useState(false);

  useEffect(() => {
    setDebugByUri(settingsCtx.get("debugByUri") as string[]);
    setPrintToConsole(settingsCtx.get("logPrintToConsole") as boolean);
  }, [settingsCtx.changed]);

  const createLog = useCallback(
    (level: LoggingLevel, description: string, details: string) => {
      setLogs((prev) => [new LogEvent(level, description, details), ...prev].slice(0, 500));

      if (!printToConsole) return;

      switch (level) {
        case LoggingLevel.ERROR:
          console.error(level, description, details);
          setCountErrors((prev) => prev + 1);
          break;
        case LoggingLevel.WARN:
          console.warn(level, description, details);
          break;
        case LoggingLevel.DEBUG:
          console.debug(level, description, details);
          break;
        default:
          console.info(level, description, details);
      }
    },
    [printToConsole]
  );

  const debug = useCallback(
    (description: string, details = "") => createLog(LoggingLevel.DEBUG, description, details),
    [createLog]
  );

  const info = useCallback(
    (description: string, details = "", stateInfo?: string) => {
      createLog(LoggingLevel.INFO, description, details);
      if (stateInfo) sendStateInfo(stateInfo);
    },
    [createLog]
  );

  const success = useCallback(
    (description: string, details = "", stateInfo?: string) => {
      createLog(LoggingLevel.SUCCESS, description, details);
      if (stateInfo) sendStateSuccess(stateInfo);
    },
    [createLog]
  );

  const warn = useCallback(
    (description: string, details = "", stateInfo?: string) => {
      createLog(LoggingLevel.WARN, description, details);
      if (stateInfo) sendStateWarn(stateInfo);
    },
    [createLog]
  );

  const error = useCallback(
    (description: string, details = "", stateInfo?: string) => {
      createLog(LoggingLevel.ERROR, description, details);
      if (stateInfo) sendStateError(stateInfo);
    },
    [createLog]
  );

  const clearLogs = useCallback(() => setLogs([]), []);

  const debugInterface = useCallback(
    (uri: string, msg: TResult | JSONObject | string, details?: string, providerName?: string) => {
      if (!debugByUri.includes(uri)) return;

      let parsed: unknown = msg;

      try {
        if (typeof msg === "string") parsed = JSON.parse(msg);
      } catch {
        parsed = msg;
      }

      debug(`ws://${providerName}?${uri}${details ? ` : ${details}` : ""}`, JSON.stringify(parsed));
    },
    [debugByUri, debug]
  );

  const value = useMemo(
    () => ({
      logs,
      countErrors,
      debug,
      info,
      success,
      warn,
      error,
      clearLogs,
      debugInterface,
    }),
    [logs, countErrors, debug, info, success, warn, error, clearLogs, debugInterface]
  );

  return <LoggingContext.Provider value={value}>{children}</LoggingContext.Provider>;
}

export default LoggingContext;
