import { generateUniqueId } from "../utils";

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
    const hours = String(this.date.getHours()).padStart(2, '0'); // Stunden (00-23)
    const minutes = String(this.date.getMinutes()).padStart(2, '0'); // Minuten (00-59)
    const seconds = String(this.date.getSeconds()).padStart(2, '0'); // Sekunden (00-59)
    const ms = String(this.date.getMilliseconds()).padStart(3, '0'); // Millisekunden (00-999)
    this.timestamp = `${hours}:${minutes}:${seconds}.${ms}`;
    this.description = description;
    this.details = details;
  }
}
