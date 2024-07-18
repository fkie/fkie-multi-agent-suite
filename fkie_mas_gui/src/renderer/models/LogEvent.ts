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
class LogEvent {
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
  datum: string;

  /**
   * Description of the log (usually concise)
   */
  description: string;

  /**
   * Detailed log info
   */
  details: string;

  /**
   * Class Constructor
   *
   * @param {LoggingLevel} level - Logging level
   * @param {string} description - Description of the log (usually concise)
   * @param {string} details - Detailed log info
   * @param {string} datum - Datum of the log event
   */
  constructor(level: LoggingLevel, description: string, details = "", datum = new Date().toLocaleString()) {
    this.id = generateUniqueId();
    this.level = level;
    this.datum = datum;
    this.description = description;
    this.details = details;
  }
}

export default LogEvent;
