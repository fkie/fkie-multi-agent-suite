import { generateUniqueId, tsStr } from "../utils";

export enum LoggingLevel {
  DEBUG = "DEBUG",
  INFO = "INFO",
  WARN = "WARN",
  SUCCESS = "SUCCESS",
  ERROR = "ERROR",
}

/**
 * LogEvent Class: Storage logging info
 */
export default class LogEvent {
  /**
   * Unique log item identifier
   */
  id: string;

  /**
   * Log level: INFO, SUCCESS, WARN and ERROR
   */
  level: LoggingLevel;

  /**
   * Datum of the log event
   */
  date: Date;

  /** string of the timestamp */
  timestamp: string;

  /**
   * Description of the log (usually concise)
   */
  description: string;

  /**
   * Detailed log info
   */
  details: string | undefined;

  constructor(level: LoggingLevel, description: string, details = "") {
    this.id = generateUniqueId();
    this.level = level;
    this.date = new Date();
    this.timestamp = tsStr(this.date);
    this.description = description;
    this.details = details;
  }
}
