export enum LogLevelType {
  DEBUG = "DEBUG",
  INFO = "INFO",
  WARN = "WARN",
  ERROR = "ERROR",
  FATAL = "FATAL",
}

export class LoggerConfig {
  level: LogLevelType;

  name: string;

  constructor(level: LogLevelType, name: string) {
    this.level = level;
    this.name = name;
  }
}

export default LoggerConfig;
