import Fade from "@mui/material/Fade";
import { VariantType, useSnackbar } from "notistack";
import React, { createContext, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import LoggingDetailsComponent from "@/renderer/components/UI/LoggingDetailsComponent";
import { LogEvent, LoggingLevel } from "@/renderer/models";
import { LAYOUT_TABS, LAYOUT_TAB_SETS } from "@/renderer/pages/NodeManager/layout/LayoutDefines";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import { JSONObject, TResult } from "@/types";
import { SettingsContext } from "./SettingsContext";

export interface ILoggingContext {
  logs: LogEvent[];
  countErrors: number;
  debug: (description: string, details?: string, showSnackbar?: boolean) => void;
  info: (description: string, details?: string, showSnackbar?: boolean) => void;
  success: (description: string, details?: string, showSnackbar?: boolean) => void;
  warn: (description: string, details?: string, showSnackbar?: boolean) => void;
  error: (description: string, details?: string, showSnackbar?: boolean) => void;
  clearLogs: () => void;
  debugInterface: (
    uri: string,
    result: TResult | JSONObject | string,
    details?: TResult | JSONObject | string,
    providerName?: string
  ) => void;
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

  const { enqueueSnackbar } = useSnackbar();

  useEffect(() => {
    setDebugByUri(settingsCtx.get("debugByUri") as string[]);
  }, [settingsCtx, settingsCtx.changed]);

  function createLog(
    level: LoggingLevel,
    description: string,
    details: string,
    showSnackbar: boolean,
    snackbarVariant: VariantType
  ): void {
    // add new log event
    setLogs((prevLogs) => [new LogEvent(level, description, details), ...prevLogs]);
    // check settings logger level
    const value = settingsCtx.get("guiLogLevel");
    if (Array.isArray(value) && !value.includes(level)) return;

    if (level === LoggingLevel.ERROR) {
      console.error(level, description, details);
      setCountErrors(countErrors + 1);
    } else if (level === LoggingLevel.WARN) console.warn(level, description, details);
    else console.info(level, description, details);

    if (showSnackbar) {
      enqueueSnackbar(`${description}`, {
        TransitionComponent: Fade,
        content: (key, message) => (
          <LoggingDetailsComponent
            id={key}
            message={message}
            details={details}
            variant={snackbarVariant}
            onDetailsClick={() => {
              emitCustomEvent(
                EVENT_OPEN_COMPONENT,
                eventOpenComponent(LAYOUT_TABS.LOGGING, "Logging Panel", <></>, false, LAYOUT_TAB_SETS.BORDER_BOTTOM)
              );
            }}
          />
        ),
        variant: snackbarVariant,
      });
    }
  }

  function debug(description: string, details: string = "", showSnackbar: boolean = false): void {
    createLog(LoggingLevel.DEBUG, description, details, showSnackbar, "default");
  }

  function info(description: string, details: string = "", showSnackbar: boolean = true): void {
    createLog(LoggingLevel.INFO, description, details, showSnackbar, "info");
  }

  function success(description: string, details: string = "", showSnackbar: boolean = true): void {
    createLog(LoggingLevel.SUCCESS, description, details, showSnackbar, "success");
  }

  function warn(description: string, details: string = "", showSnackbar: boolean = true): void {
    createLog(LoggingLevel.WARN, description, details, showSnackbar, "warning");
  }

  function error(description: string, details: string = "", showSnackbar: boolean = true): void {
    createLog(LoggingLevel.ERROR, description, details, showSnackbar, "error");
  }

  function clearLogs(): void {
    setLogs(() => []);
  }

  function debugInterface(
    uri: string,
    msg: TResult | JSONObject | string,
    details?: TResult | JSONObject | string,
    providerName?: string
  ): void {
    // check settings to debug by uri

    let parsedMsg: TResult | JSONObject | string = {};
    try {
      if (msg && typeof msg === "string" && msg.length > 0) parsedMsg = JSON.parse(msg);
      else parsedMsg = msg;
    } catch (errorParse: unknown) {
      warn(`[URI] ${providerName} (${uri}): error while read debug message`, JSON.stringify(errorParse));
      parsedMsg = msg;
    }

    let parseDetails: TResult | JSONObject | string = {};
    if (details) {
      try {
        if (typeof details === "string" && details.length > 0) {
          parseDetails = JSON.parse(details);
        } else parseDetails = details;
      } catch (errorParse: unknown) {
        warn(`[URI] ${providerName} (${uri}): error while read debug details`, JSON.stringify(errorParse));
        parseDetails = details;
      }
    }

    if (Array.isArray(debugByUri) && debugByUri.includes(uri)) {
      debug(`[URI] ${providerName} (${uri}): ${JSON.stringify(parseDetails)}}`, JSON.stringify(parsedMsg));
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
