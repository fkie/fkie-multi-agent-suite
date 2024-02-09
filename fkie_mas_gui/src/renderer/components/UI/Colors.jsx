import { DiagnosticLevel } from '../../models/Diagnostics';

export const levelColors = {
  debug: {
    color: '#053232', // darkgrey
  },
  default: {
    color: '#053232', // darkgrey
  },
  success: {
    color: '#43a047', // green
  },
  error: {
    color: '#d32f2f', // dark red
  },
  info: {
    color: '#2196f3', // nice blue
  },
  warning: {
    color: '#ff9800', // amber
  },
};

export const levelColorsWbg = {
  debug: {
    backgroundColor: levelColors.debug.color, // darkgrey
    color: '#fff',
  },
  default: {
    backgroundColor: levelColors.default.color, // darkgrey
    color: '#fff',
  },
  success: {
    backgroundColor: levelColors.success.color, // green
    color: '#fff',
  },
  error: {
    backgroundColor: levelColors.error.color, // dark red
    color: '#fff',
  },
  info: {
    backgroundColor: levelColors.info.color, // nice blue
    color: '#fff',
  },
  warning: {
    backgroundColor: levelColors.warning.color, // amber
    color: '#fff',
  },
};

export const getDiagnosticStyle = (lvl) => {
  switch (lvl) {
    case DiagnosticLevel.OK:
      return {
        backgroundColor: '#43a047', // green
        color: '#2b2b2c',
      };
    case DiagnosticLevel.WARN:
      return {
        backgroundColor: '#ff9800', // amber
        color: '#8B4513',
      };
    case DiagnosticLevel.ERROR:
      return {
        backgroundColor: '#d32f2f', // dark red
        color: '#ffffeb',
      };
    case DiagnosticLevel.STALE:
      return {
        backgroundColor: '#FFD700', // yellow
        color: '#8B4513',
      };
    default:
      return {
        backgroundColor: '#2196f3', // nice blue
        color: '#2b2b2c',
      };
  }
};

export const HostColors = [
  '#ffffeb',
  '#575d5e',
  '#cdba88',
  '#f9a800',
  '#e88c00',
  '#af804f',
  '#ddaf27',
  '#e3d9c6',
  '#ba481b',
  '#f67828',
  '#ff4d06',
  '#59191f',
  '#d8a0a6',
  '#816183',
  '#00387b',
  '#0f4c64',
  '#0089b6',
  '#637d96',
  '#058b8c',
  '#222d5a',
  '#3c7460',
  '#366735',
  '#50533c',
  '#114232',
  '#6c7c59',
  '#61993b',
  '#b9ceac',
  '#008351',
  '#7ebab5',
  '#00b51a',
  '#7a888e',
  '#6c6e6b',
  '#766a5e',
  '#383e42',
  '#808076',
  '#c5c7c4',
  '#89693e',
  '#70452a',
  '#8d4931',
  '#5a3826',
  '#e9e0d2',
  '#ecece7',
  '#2b2b2c',
  '#797b7a',
  '#c4618c',
  '#76689a',
  '#bc4077',
];

export const colorFromHostname = (hostname) => {
  if (!hostname) return HostColors[0];
  // TODO get color from manually assigned list
  // generate hashCode from hostname
  let hash = 0;
  for (let i = 0; i < hostname.length; i++) {
    hash = (hash << 5) - hash + hostname.charCodeAt(i);
    hash |= 0; // Convert to 32bit integer
  }
  // determine a color
  const index = Math.abs(hash) % HostColors.length;
  return HostColors[index];
};

export const TagColors = [
  // 'red', // Reserve for errors
  'cool-gray',
  'grey',
  'green',
  'teal',
  'outline',
  'cyan',
  'high-contrast',
  'magenta',
  'purple',
];
