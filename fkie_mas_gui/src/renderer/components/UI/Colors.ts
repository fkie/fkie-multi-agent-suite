import { DiagnosticLevel } from "../../models/Diagnostics";

export const levelColors = {
  debug: {
    color: "#053232", // darkgrey
  },
  default: {
    color: "#053232", // darkgrey
  },
  success: {
    color: "#43a047", // green
  },
  error: {
    color: "#d32f2f", // dark red
  },
  info: {
    color: "#2196f3", // nice blue
  },
  warning: {
    color: "#ff9800", // amber
  },
};

export const levelColorsWbg = {
  debug: {
    backgroundColor: levelColors.debug.color, // darkgrey
    color: "#fff",
  },
  default: {
    backgroundColor: levelColors.default.color, // darkgrey
    color: "#fff",
  },
  success: {
    backgroundColor: levelColors.success.color, // green
    color: "#fff",
  },
  error: {
    backgroundColor: levelColors.error.color, // dark red
    color: "#fff",
  },
  info: {
    backgroundColor: levelColors.info.color, // nice blue
    color: "#fff",
  },
  warning: {
    backgroundColor: levelColors.warning.color, // amber
    color: "#fff",
  },
};

export function getDiagnosticStyle(lvl: DiagnosticLevel): object {
  switch (lvl) {
    case DiagnosticLevel.OK:
      return {
        backgroundColor: "#43a047", // green
        color: "#2b2b2c",
      };
    case DiagnosticLevel.WARN:
      return {
        backgroundColor: "#ff9800", // amber
        color: "#8B4513",
      };
    case DiagnosticLevel.ERROR:
      return {
        backgroundColor: "#d32f2f", // dark red
        color: "#ffffeb",
      };
    case DiagnosticLevel.STALE:
      return {
        backgroundColor: "#FFD700", // yellow
        color: "#8B4513",
      };
    default:
      return {
        backgroundColor: "#2196f3", // nice blue
        color: "#2b2b2c",
      };
  }
}

export const HostColors = [
  "#191f45",
  "#1f4788",
  "#5a4f74",
  "#317589",
  "#4d8fac",
  "#86aba5",
  "#a5ba93",
  "#749f8d",
  "#6b9362",
  "#407a52",
  "#006442",
  "#203838",
  "#52593b",
  "#5b8930",
  "#7a942e",
  "#bda928",
  "#ceb579",
  "#e8dfae",
  "#fce655",
  "#ffb61e",
  "#e08a1e",
  "#bb8141",
  "#785e49",
  "#4c3d30",
  "#824b35",
  "#ca6924",
  "#ff8936",
  "#ffa565",
  "#ff7952",
  "#ed4236",
  "#b14a30",
  "#672422",
  "#913228",
  "#c3272b",
  "#f8674f",
  "#fa7b62",
  "#ffb3a7",
  "#f47983",
  "#f2666c",
  "#db5a6b",
  "#c93756",
  "#763568",
  "#755d5b",
  "#8d608c",
  "#bb7796",
];

export function colorFromHostname(hostname: string): string {
  if (!hostname) return HostColors[0];
  // TODO get color from manually assigned list
  // generate hashCode from hostname
  let hash = 0;
  for (let i = 0; i < hostname.length; i++) {
    hash = (hash << 5) - hash + hostname.charCodeAt(i);
    hash |= 0; // Convert to 32bit integer
  }
  // determine a color (map 31 unsigned bits to color list)
  const index = Math.floor((Math.abs(hash) / 0x80000000) * HostColors.length);
  return HostColors[index];
}

export const TagColors = [
  // 'red', // Reserve for errors
  "cool-gray",
  "grey",
  "green",
  "teal",
  "outline",
  "cyan",
  "high-contrast",
  "magenta",
  "purple",
];
