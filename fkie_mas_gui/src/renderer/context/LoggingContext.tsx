import React, { createContext, useContext, useEffect, useMemo, useState } from "react";

import { LogEvent, LoggingLevel } from "@/renderer/models";
import {
  sendStateError,
  sendStateInfo,
  sendStateSuccess,
  sendStateWarn,
} from "@/renderer/pages/NodeManager/layout/events";
import { JSONObject, TResult } from "@/types";
import { SettingsContext } from "./SettingsContext";

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

export function LoggingProvider({ children }: ILoggingProvider): ReturnType<React.FC<ILoggingProvider>> {
  const settingsCtx = useContext(SettingsContext);
  const [logs, setLogs] = useState<LogEvent[]>([]);
  const [countErrors, setCountErrors] = useState<number>(0);
  const [debugByUri, setDebugByUri] = useState<string[]>(settingsCtx.get("debugByUri") as string[]);
  const [currentLogLevel, setCurrentLogLevel] = useState<string[]>(settingsCtx.get("guiLogLevel") as string[]);
  const [printToConsole, setPrintToConsole] = useState<boolean>(settingsCtx.get("logPrintToConsole") as boolean);

  useEffect(() => {
    setDebugByUri([...(settingsCtx.get("debugByUri") as string[])]);
    setCurrentLogLevel([...(settingsCtx.get("guiLogLevel") as string[])]);
    setPrintToConsole(settingsCtx.get("logPrintToConsole") as boolean);
  }, [settingsCtx, settingsCtx.changed]);

  function createLog(level: LoggingLevel, description: string, details: string): void {
    // add new log event
    setLogs((prevLogs) => [new LogEvent(level, description, details), ...prevLogs]);
    if (printToConsole) {
      if (level === LoggingLevel.ERROR) {
        console.error(level, description, details);
        setCountErrors(countErrors + 1);
      } else if (level === LoggingLevel.WARN) console.warn(level, description, details);
      else if (level === LoggingLevel.DEBUG) console.debug(level, description, details);
      else console.info(level, description, details);
    }
    // check settings logger level
    if (Array.isArray(currentLogLevel) && !currentLogLevel.includes(level)) return;
  }

  function debug(description: string, details: string = ""): void {
    createLog(LoggingLevel.DEBUG, description, details);
  }

  function info(description: string, details: string = "", stateInfo?: string): void {
    createLog(LoggingLevel.INFO, description, details);
    if (stateInfo) sendStateInfo(stateInfo);
  }

  function success(description: string, details: string = "", stateInfo?: string): void {
    createLog(LoggingLevel.SUCCESS, description, details);
    if (stateInfo) sendStateSuccess(stateInfo);
  }

  function warn(description: string, details: string = "", stateInfo?: string): void {
    createLog(LoggingLevel.WARN, description, details);
    if (stateInfo) sendStateWarn(stateInfo);
  }

  function error(description: string, details: string = "", stateInfo?: string): void {
    createLog(LoggingLevel.ERROR, description, details);
    if (stateInfo) sendStateError(stateInfo);
  }

  function clearLogs(): void {
    setLogs(() => []);
  }

  function debugInterface(
    uri: string,
    msg: TResult | JSONObject | string,
    details?: string,
    providerName?: string
  ): void {
    // check settings to debug by uri

    let parsedMsg: TResult | JSONObject | string = {};
    try {
      if (msg && typeof msg === "string" && msg.length > 0) parsedMsg = JSON.parse(msg);
      else parsedMsg = msg;
    } catch (errorParse: unknown) {
      warn(`ws://${providerName}?${uri}: error while read debug message`, JSON.stringify(errorParse));
      parsedMsg = msg;
    }

    const detailsStr = details ? `: ${details}` : "";

    if (Array.isArray(debugByUri) && debugByUri.includes(uri)) {
      debug(`ws://${providerName}?${uri}${detailsStr}`, JSON.stringify(parsedMsg));
    }
  }

  const attributesMemo = useMemo(
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
    [logs, countErrors]
  );

  return <LoggingContext.Provider value={attributesMemo}>{children}</LoggingContext.Provider>;
}

export default LoggingContext;
