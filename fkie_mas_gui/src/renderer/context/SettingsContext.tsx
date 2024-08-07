import React, { createContext, useMemo, useReducer } from "react";
import useLocalStorage from "../hooks/useLocalStorage";
import URI from "../models/uris";

export const getDefaultPortFromRos: (connectionType: string, rosVersion: string) => number = (
  connectionType,
  rosVersion
) => {
  if (connectionType === "crossbar-wamp") {
    return rosVersion === "2" ? 11811 : 11911;
  }
  return rosVersion === "2" ? 35430 : 35685;
};

export interface ISettingsContext {
  MIN_VERSION_DAEMON: string;
  changed: number;
  get: (attribute: string) => any;
  set: (attribute: string, value: any, settingsCtx?: ISettingsContext) => void;
  getParamList?: () => void;
}

export const LOG_LEVEL_LIST = ["DEBUG", "INFO", "SUCCESS", "WARN", "ERROR"];

export const LAUNCH_FILE_EXTENSIONS = [".launch", "launch.xml", "launch.py", "launch.yaml", "launch.yml"];

export const DEFAULT_SETTINGS = {
  MIN_VERSION_DAEMON: "3.0.5",
  fgColor: "#1a73e8",
  bgColor: "#fafafa",
  fgColorForDarkMode: "#B8E7FB",
  bgColorForDarkMod: "#424242",

  changed: 0,
  get: () => {
    return undefined;
  },
  set: () => {},
  getParamList: () => {
    return [];
  },
};

export interface ISettingsParam {
  label?: string;
  default: any;
  type: string;
  placeholder?: string;
  options?: string | any[];
  freeSolo?: boolean;
  readOnly?: boolean;
  description?: string;
  group?: string;
  min?: number;
  max?: number;
  cb?: (get: (attribute: string) => any, set: (attribute: string, value: any) => void) => void;
}

export const SETTINGS_DEF: { [id: string]: ISettingsParam } = {
  useDarkMode: {
    label: "Dark mode",
    default: false,
    type: "boolean",
    description: "",
    cb: (get: (attribute: string) => any, set: (attribute: string, value: any) => void) => {
      const newValue = get("useDarkMode");
      set("color", newValue ? DEFAULT_SETTINGS.fgColorForDarkMode : DEFAULT_SETTINGS.fgColor);
      set("backgroundColor", newValue ? DEFAULT_SETTINGS.bgColorForDarkMod : DEFAULT_SETTINGS.bgColor);
    },
  },
  colorizeHosts: {
    label: "Colorize hosts",
    default: true,
    type: "boolean",
    description: "",
  },
  showFloatingButtons: {
    label: "Show floating buttons",
    default: false,
    type: "boolean",
    description: "",
  },
  checkForUpdates: {
    label: "Check for updates on start",
    default: true,
    type: "boolean",
    description: "",
  },
  resetLayout: {
    label: "Reset layout",
    type: "button",
    default: false,
    description: "",
  },
  rosVersion: {
    label: "Default ROS version",
    default: "1",
    type: "string",
    options: ["1", "2"],
    readOnly: false,
    description: "default ROS version used to start remote daemon and discovery nodes.",
  },
  guiLogLevel: {
    label: "Log Level",
    type: "string[]",
    default: ["INFO", "SUCCESS", "WARN", "ERROR"],
    options: LOG_LEVEL_LIST,
    description: "displayed log levels",
  },
  debugByUri: {
    label: "Interface URIs",
    type: "string[]",
    default: [URI.ROS_PROVIDER_GET_LIST, URI.ROS_DAEMON_READY, URI.ROS_DISCOVERY_READY],
    options: Object.values(URI).sort(),
    description: "",
  },
  groupParameters: {
    label: "Group parameters",
    placeholder: "...",
    freeSolo: true,
    type: "string[]",
    default: ["/capability_group"],
    options: ["/capability_group"],
    description: "parameter specified the group of the node",
  },
  launchHistoryLength: {
    label: "Launch History Length",
    type: "number",
    default: 5,
    min: 0,
    max: 15,
  },
  ntpServer: {
    label: "NTP Server",
    placeholder: "...",
    freeSolo: true,
    type: "string[]",
    default: ["ntp.ubuntu.com"],
    options: ["ntp.ubuntu.com"],
  },
  color: {
    label: "Color",
    type: "none",
    default: "#1a73e8",
  },
  backgroundColor: {
    label: "Color",
    type: "none",
    default: "#fafafa",
  },
  timeDiffThreshold: {
    label: "Time Diff Threshold [ms]",
    type: "numberX",
    default: 500,
    min: 0,
  },
  namespaceSystemNodes: {
    label: "Namespace for system nodes",
    type: "none",
    default: "/{SYSTEM}",
  },
  tooltipEnterDelay: {
    label: "The number of milliseconds to wait before showing the tooltip.",
    type: "none",
    default: 1500,
  },
  actionOnChangeLaunch: {
    label: "Action on loaded launch file change detection",
    type: "string",
    default: "ASK",
    options: ["ASK", "DISMISS", "RELOAD"],
    description: "",
  },
  fontSizeTerminal: {
    label: "Font size in terminal",
    type: "number",
    default: 14,
    min: 2,
  },
  fontSize: {
    label: "Font size",
    type: "number",
    default: 14,
    min: 2,
  },
  editorOpenLocation: {
    label: "Location to open new editor",
    type: "string",
    default: "CENTER",
    options: ["BORDER_TOP", "CENTER", "BORDER_BOTTOM"],
    description: "",
  },
  showRemoteNodes: {
    label: "Show remote nodes",
    default: false,
    type: "none",
    description: "Each host shows all nodes visible to it",
  },
};

interface ISettingProvider {
  children: React.ReactNode;
}

export const SettingsContext = createContext<ISettingsContext>(DEFAULT_SETTINGS);

export function SettingsProvider({ children }: ISettingProvider): ReturnType<React.FC<ISettingProvider>> {
  const { MIN_VERSION_DAEMON } = DEFAULT_SETTINGS;
  const [changed, forceUpdate] = useReducer((x) => x + 1, 0);
  const [config, setConfig] = useLocalStorage<any>("SettingsContext:config", {});

  const get = (attribute: string) => {
    if (attribute in config) {
      return config[attribute];
    }
    if (attribute in SETTINGS_DEF) {
      return SETTINGS_DEF[attribute]?.default;
    }
    throw new Error(`Configuration attribute ${attribute} not found!`);
  };

  const set = (attribute: string, value: any) => {
    if (SETTINGS_DEF[attribute]) {
      // TODO: check for valid value
      config[attribute] = value;
      setConfig(config);
      if (SETTINGS_DEF[attribute].cb) {
        SETTINGS_DEF[attribute].cb?.(get, set);
      }
      forceUpdate();
    } else {
      throw new Error(`Configuration attribute ${attribute} while set() not found!`);
    }
  };

  const getParamList = () => {
    const params: { name: string; param: ISettingsParam }[] = [];
    Object.keys(SETTINGS_DEF).forEach(function (key) {
      params.push({ name: key, param: SETTINGS_DEF[key] });
    });
    return params;
  };

  const attributesMemo = useMemo(
    () => ({
      MIN_VERSION_DAEMON,
      changed,
      get,
      set,
      getParamList,
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [changed]
  );

  return <SettingsContext.Provider value={attributesMemo}>{children}</SettingsContext.Provider>;
}

export default SettingsContext;
