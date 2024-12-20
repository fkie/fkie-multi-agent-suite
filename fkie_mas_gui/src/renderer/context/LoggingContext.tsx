import { JSONObject, TResult } from "@/types";
import Fade from "@mui/material/Fade";
import { VariantType, useSnackbar } from "notistack";
import React, { createContext, useContext, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import LoggingDetailsComponent from "../components/UI/LoggingDetailsComponent";
import { LogEvent, LoggingLevel } from "../models";
import { LAYOUT_TABS, LAYOUT_TAB_SETS } from "../pages/NodeManager/layout/LayoutDefines";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../pages/NodeManager/layout/events";
import { SettingsContext } from "./SettingsContext";

export interface ILoggingContext {
  logs: LogEvent[];
  countErrors: number;
  debug: (description: string, details: string, showSnackbar?: boolean) => void;
  info: (description: string, details: string, showSnackbar?: boolean) => void;
  success: (description: string, details: string, showSnackbar?: boolean) => void;
  warn: (description: string, details: string, showSnackbar?: boolean) => void;
  error: (description: string, details: string, showSnackbar?: boolean) => void;
  clearLogs?: () => void;
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
  debug: () => {},
  info: () => {},
  success: () => {},
  warn: () => {},
  error: () => {},
  debugInterface: () => {},
};

// TODO Add valid github issue repository

// This text should appear every time a "bug" is found (Things that the user can not control/configure)
export const DEFAULT_BUG_TEXT = "A bug occurred, please consider report it as an issue on our github repository";

export const LoggingContext = createContext<ILoggingContext>(DEFAULT_LOGGING);

export function LoggingProvider({ children }: ILoggingProvider): ReturnType<React.FC<ILoggingProvider>> {
  const settingsCtx = useContext(SettingsContext);
  const [logs, setLogs] = useState<LogEvent[]>([]);
  const [countErrors, setCountErrors] = useState<number>(0);
  const { enqueueSnackbar } = useSnackbar();

  const createLog = (
    level: LoggingLevel,
    description: string,
    details: string,
    showSnackbar: boolean,
    snackbarVariant: VariantType
  ) => {
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
  };

  const debug = (description: string, details = "", showSnackbar = false) => {
    createLog(LoggingLevel.DEBUG, description, details, showSnackbar, "default");
  };

  const info = (description: string, details = "", showSnackbar = true) => {
    createLog(LoggingLevel.INFO, description, details, showSnackbar, "info");
  };

  const success = (description: string, details = "", showSnackbar = true) => {
    createLog(LoggingLevel.SUCCESS, description, details, showSnackbar, "success");
  };

  const warn = (description: string, details = "", showSnackbar = true) => {
    createLog(LoggingLevel.WARN, description, details, showSnackbar, "warning");
  };

  const error = (description: string, details = "", showSnackbar = true) => {
    createLog(LoggingLevel.ERROR, description, details, showSnackbar, "error");
  };

  const clearLogs = () => {
    setLogs(() => []);
  };

  const debugInterface = (
    uri: string,
    msg: TResult | JSONObject | string,
    details?: TResult | JSONObject | string,
    providerName?: string
  ) => {
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

    const value = settingsCtx.get("debugByUri");
    if (Array.isArray(value) && value.includes(uri)) {
      debug(`[URI] ${providerName} (${uri}): ${JSON.stringify(parseDetails)}}`, JSON.stringify(parsedMsg));
    }
  };

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
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [logs, countErrors]
  );

  return <LoggingContext.Provider value={attributesMemo}>{children}</LoggingContext.Provider>;
}

export default LoggingContext;
