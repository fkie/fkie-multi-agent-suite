/* eslint-disable max-classes-per-file */
/* eslint-disable camelcase */
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

export default class DiagnosticArray {
  timestamp: number;

  status: DiagnosticStatus[] | undefined;

  constructor(timestamp: number, status: DiagnosticStatus[]) {
    this.timestamp = timestamp;
    this.status = status;
  }
}
