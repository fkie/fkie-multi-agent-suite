export enum InfoStateLevel {
  INFO = "INFO",
  SUCCESS = "SUCCESS",
  WARN = "WARN",
  ERROR = "ERROR",
}

export type TInfoState = {
  level: InfoStateLevel;
  message: string;
};
