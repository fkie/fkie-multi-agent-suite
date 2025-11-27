import { DiagnosticLevel } from "@/renderer/models/Diagnostics";

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
  warn: {
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
  warn: {
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
        backgroundColor: "#2196f3", // nice blue
        color: "#2b2b2c",
      };
    default:
      return {
        backgroundColor: "#FFD700", // yellow
        color: "#8B4513",
      };
  }
}

export function getDiagnosticColor(lvl: DiagnosticLevel, isDarkMode: boolean): string {
  switch (lvl) {
    case DiagnosticLevel.OK:
      return isDarkMode ? "#43a047" : "#43a047"; // green
    case DiagnosticLevel.WARN:
      return isDarkMode ? "#fb8c00" : "#ff9800"; // amber
    case DiagnosticLevel.ERROR:
      return isDarkMode ? "#e53935" : "#d32f2f"; // dark red
    case DiagnosticLevel.STALE:
      return isDarkMode ? "#1e88e5" : "#2196f3"; // nice blue
    default:
      return isDarkMode ? "#fbc02d" : "#FFD700"; // yellow
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


export function averageColor(colors: string[], isDarkMode: boolean): string {
  let totalR = 0;
  let totalG = 0;
  let totalB = 0;
  let totalA = 0;
  let validCount = 0;

  for (const color of colors) {
    const rgbaMatch = color.match(
      /rgba?\s*\(\s*([0-9.]+)[,\s]+([0-9.]+)[,\s]+([0-9.]+)(?:[,\s/]+([0-9.]+))?\s*\)/
    );

    const hexMatch = color.match(/^#([0-9a-fA-F]{6})$/);

    if (rgbaMatch) {
      const [, r, g, b, a] = rgbaMatch;
      totalR += Number.parseFloat(r);
      totalG += Number.parseFloat(g);
      totalB += Number.parseFloat(b);
      totalA += a !== undefined ? Number.parseFloat(a) : 1;
      validCount++;
    } else if (hexMatch) {
      const hex = hexMatch[1];
      const r = Number.parseInt(hex.substring(0, 2), 16);
      const g = Number.parseInt(hex.substring(2, 4), 16);
      const b = Number.parseInt(hex.substring(4, 6), 16);
      totalR += r;
      totalG += g;
      totalB += b;
      totalA += 1; // default alpha for hex
      validCount++;
    } else {
      console.warn(`⚠️ Invalid color skipped: ${color}`);
    }
  }

  if (validCount === 0) {
    return isDarkMode ? "#43a047" : "#43a047"; // fallback (green)
  }

  const avgR = Math.round(totalR / validCount);
  const avgG = Math.round(totalG / validCount);
  const avgB = Math.round(totalB / validCount);
  const avgA = Number.parseFloat((totalA / validCount).toFixed(3));

  return `rgba(${avgR}, ${avgG}, ${avgB}, ${avgA})`;
}
