import { useContext } from "react";
import LoggingContext, { ILoggingContext } from "../context/LoggingContext";


export function useLoggingContext(): ILoggingContext {
  const context = useContext(LoggingContext);

  if (!context) {
    throw new Error("useLoggingContext must be used inside LoggingProvider");
  }

  return context;
}
