/* eslint-disable max-classes-per-file */
/* eslint-disable camelcase */

import { getDiagnosticColor } from "../components/UI/Colors";

const DIAGNOSTIC_HISTORY_LENGTH = 10;

export enum DiagnosticLevel {
  OK = 0,
  WARN = 1,
  ERROR = 2,
  STALE = 3,
}

export function getMaxDiagnosticLevel(
  lvl1: DiagnosticLevel | undefined,
  lvl2: DiagnosticLevel | undefined
): DiagnosticLevel | undefined {
  if (lvl1 === undefined && lvl2 === undefined) {
    return DiagnosticLevel.OK;
  }
  if (lvl1 === undefined) {
    return lvl2;
  }
  if (lvl2 === undefined) {
    return lvl1;
  }
  const maxLvl = Math.max(lvl1, lvl2);
  if (maxLvl === DiagnosticLevel.STALE) {
    const minLvl = Math.min(lvl1, lvl2);
    if (minLvl !== DiagnosticLevel.OK) {
      return minLvl;
    }
  }
  return maxLvl;
}

export function getDiagnosticLevelName(lvl: DiagnosticLevel): string {
  switch (lvl) {
    case 0:
      return `OK[${lvl}]`;
    case 1:
      return `WARN[${lvl}]`;
    case 2:
      return `ERROR[${lvl}]`;
    case 3:
      return `STALE[${lvl}]`;
    default:
      return `UNKNOWN[${lvl}]`;
  }
}

export class DiagnosticKeyValue {
  key: string;

  value: string;

  constructor(key: string, value: string) {
    this.key = key;
    this.value = value;
  }
}

export class DiagnosticStatus {
  level: DiagnosticLevel;

  name: string;

  message: string;

  hardware_id: string | undefined;

  values: DiagnosticKeyValue[] | undefined = [];

  constructor(
    level: DiagnosticLevel,
    name: string,
    message: string,
    hardware_id: string,
    values: DiagnosticKeyValue[] = []
  ) {
    this.level = level;
    this.name = name;
    this.message = message;
    this.hardware_id = hardware_id;
    this.values = values;
  }
}

export class DiagnosticArray {
  timestamp: number;

  status: DiagnosticStatus[] | undefined;

  constructor(timestamp: number, status: DiagnosticStatus[]) {
    this.timestamp = timestamp;
    this.status = status;
  }
}

export class DiagnosticNodeInfo {
  name: string;
  diagnosticArray: DiagnosticArray[] = [];

  /** Calculated values of the last diagnosticArray message. */
  diagnosticLevel: DiagnosticLevel = DiagnosticLevel.OK;
  diagnosticMessage: string = "";
  diagnosticColor: string = "";

  constructor(name: string, status: DiagnosticStatus, timestamp: number) {
    this.name = name;
    this.addDiagnosticStatus(status, timestamp);
  }

  public addDiagnosticStatus(ds: DiagnosticStatus, timestamp: number): void {
    if (this.diagnosticArray.length === 0 || this.diagnosticArray[0].timestamp !== timestamp) {
      const da = new DiagnosticArray(timestamp, [ds]);
      this.diagnosticArray = [da, ...this.diagnosticArray.slice(0, DIAGNOSTIC_HISTORY_LENGTH - 1)];
    } else {
      this.diagnosticArray[0].status?.push(ds);
    }

    let diagnosticMessage = "";
    let diagnosticColor = "";
    this.diagnosticLevel = DiagnosticLevel.OK;
    for (const status of this.diagnosticArray[0].status || []) {
      for (const value of status.values || []) {
        if (value.key === "color") {
          diagnosticColor = value.value;
        }
      }
      diagnosticMessage += status.message ? `${status.message} ` : "";
      this.diagnosticLevel = getMaxDiagnosticLevel(this.diagnosticLevel, status.level) || this.diagnosticLevel;
    }
    this.diagnosticMessage = diagnosticMessage;
    this.diagnosticColor = diagnosticColor;
  }

  public getColor(isDarkMode: boolean) {
    return this.diagnosticColor ? this.diagnosticColor : getDiagnosticColor(this.diagnosticLevel, isDarkMode);
  }
}

export default class DiagnosticInfo {
  diagnosticNodes: DiagnosticNodeInfo[] = [];

  public add(diags: DiagnosticArray) {
    // update the screens
    const timestamp = Date.now();
    const diagStatus: DiagnosticStatus[] = diags.status || [];
    for (const status of diagStatus) {
      // match the name without leading slash
      // match the name with dots instead of slashes
      // match the name with trailing logger name
      const statusName = `/${status.name.replace(/^\/+/, "").replaceAll(".", "/")}`;
      const statusNodeIndex: number = this.diagnosticNodes.findIndex((node) => node.name === statusName);
      if (statusNodeIndex === -1) {
        this.diagnosticNodes.push(new DiagnosticNodeInfo(statusName, status, timestamp));
      } else {
        this.diagnosticNodes[statusNodeIndex].addDiagnosticStatus(status, timestamp);
      }
    }
  }

  public get(nodeName: string): DiagnosticNodeInfo | undefined {
    return this.diagnosticNodes.find((node) => node.name === nodeName || node.name.startsWith(`${nodeName}/`));
  }
}
