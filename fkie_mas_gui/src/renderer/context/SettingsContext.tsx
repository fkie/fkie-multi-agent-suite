import { JSONObject, JSONValue } from "@/types";
import React, { createContext, useMemo, useReducer } from "react";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import URI from "@/renderer/models/uris";

export const getDefaultPortFromRos: (
  connectionType: string,
  rosVersion: string,
  ros1MasterUri: string,
  networkId: number
) => number = (connectionType, rosVersion, ros1MasterUri, networkId) => {
  if (connectionType === "crossbar-wamp") {
    return rosVersion === "2" ? 11811 + networkId : 11911 + networkId;
  }
  let uriShift = 0;
  if (ros1MasterUri && ros1MasterUri !== "default") {
    // shift port if ROS_MASTER_URI has not a default port
    uriShift = (Number.parseInt(ros1MasterUri.split(":").slice(-1)[0]) - 11311) * 101;
  }
  return rosVersion === "2" ? 35430 + networkId : 35685 + uriShift + networkId;
};

export interface ISettingsContext {
  MIN_VERSION_DAEMON: string;
  changed: number;
  get: (attribute: string) => JSONValue | undefined;
  getDefault: (attribute: string) => JSONValue | undefined;
  set: (attribute: string, value: JSONValue, settingsCtx?: ISettingsContext) => void;
  getParamList: () => { name: string; param: ISettingsParam }[];
}

export const LOG_LEVEL_LIST = ["DEBUG", "INFO", "SUCCESS", "WARN", "ERROR"];

export const LAUNCH_FILE_EXTENSIONS = [".launch", "launch.xml", "launch.py", "launch.yaml", "launch.yml"];

export const DEFAULT_SETTINGS = {
  MIN_VERSION_DAEMON: "4.4.8",
  fgColor: "#1a73e8",
  bgColor: "#fafafa",
  fgColorForDarkMode: "#B8E7FB",
  bgColorForDarkMod: "#424242",

  changed: 0,
  get: (): JSONValue | undefined => {
    return undefined;
  },
  getDefault: (): JSONValue | undefined => {
    return undefined;
  },
  set: (): void => {},
  getParamList: (): { name: string; param: ISettingsParam }[] => {
    return [];
  },
};

export interface ISettingsParam {
  label?: string;
  default: JSONValue;
  type: string;
  placeholder?: string;
  options?: string | JSONValue[];
  freeSolo?: boolean;
  readOnly?: boolean;
  description?: string;
  group?: string;
  min?: number;
  max?: number;
  cb?: (get: (attribute: string) => JSONValue | undefined, set: (attribute: string, value: JSONValue) => void) => void;
  validate?: (value: JSONValue) => JSONValue;
  isValid?: (value: JSONValue) => boolean;
}

export const SETTINGS_DEF: { [id: string]: ISettingsParam } = {
  useDarkMode: {
    label: "Dark mode",
    default: false,
    type: "boolean",
    description: "",
    cb: (get: (attribute: string) => JSONValue | undefined, set: (attribute: string, value: JSONValue) => void) => {
      const newValue = get("useDarkMode");
      set("color", newValue ? DEFAULT_SETTINGS.fgColorForDarkMode : DEFAULT_SETTINGS.fgColor);
      set("backgroundColor", newValue ? DEFAULT_SETTINGS.bgColorForDarkMod : DEFAULT_SETTINGS.bgColor);
    },
    group: "Appearance",
  },
  colorizeHosts: {
    label: "Colorize hosts",
    default: true,
    type: "boolean",
    description: "Each host is assigned a color. Everything related to this host is marked with this color.",
    group: "Appearance",
  },
  showButtonsForKeyModifiers: {
    label: "Show buttons for key modifiers",
    default: false,
    type: "boolean",
    description: "Display buttons for additional functions that are otherwise accessible via key modifiers",
    group: "Appearance",
  },
  checkForUpdates: {
    label: "Check for updates on start",
    default: true,
    type: "boolean",
    description: "",
  },
  fontSizeTerminal: {
    label: "Font size in terminal",
    type: "number",
    default: 14,
    min: 2,
    description: "This font size only affects the terminal tab (e.g. for screen and log)",
    group: "Appearance",
  },
  fontSize: {
    label: "Font size",
    type: "number",
    default: 14,
    min: 2,
    description: "Global font size except in the terminal",
    group: "Appearance",
  },
  resetLayout: {
    label: "Reset layout",
    type: "button",
    default: false,
    description: "Restores default sizes and positions of the tabs and main window",
    group: "Appearance",
  },
  rosVersion: {
    label: "Default ROS version",
    default: import.meta.env.VITE_ROS_VERSION ? import.meta.env.VITE_ROS_VERSION : "2",
    type: "string",
    options: ["1", "2"],
    readOnly: false,
    description:
      "Standard ROS version used to start remote daemon and discovery nodes. Only if automatic detection has failed.",
  },
  guiLogLevel: {
    label: "Log Level",
    type: "string[]",
    default: ["INFO", "SUCCESS", "WARN", "ERROR"],
    options: LOG_LEVEL_LIST,
    description: "Messages that are displayed on the console. This has no effect on the output in the ‘Logging’ tab.",
    group: "Logging",
  },
  debugByUri: {
    label: "Interface URIs",
    type: "string[]",
    default: [URI.ROS_PROVIDER_GET_LIST, URI.ROS_DAEMON_READY, URI.ROS_DISCOVERY_READY],
    options: Object.values(URI).sort(),
    description:
      "When communicating with the MAS daemon, the messages from the listed URIs are output as debug messages.",
    group: "Logging",
  },
  capabilityGroupParameter: {
    label: "Capability Group parameter",
    freeSolo: true,
    type: "string",
    default: "capability_group",
    description:
      "ROS1 parameter that specifies the group of the node. If the ROS node does not have this parameter it use a global one or group according to the namespace.",
    validate: (value: JSONValue) => {
      if ((value as string).startsWith("/")) {
        return (value as string).substring(1);
      }
      return value;
    },
  },
  launchHistoryLength: {
    label: "Launch History Length",
    type: "number",
    default: 5,
    min: 0,
    max: 15,
    description: "Number of recently loaded files displayed in the Package Explorer tab.",
  },
  ntpServer: {
    label: "NTP Server",
    placeholder: "@/renderer.",
    freeSolo: true,
    type: "string[]",
    default: ["ntp.ubuntu.com"],
    options: ["ntp.ubuntu.com"],
    group: "Parametrization",
  },
  logCommand: {
    label: "Log command prefix",
    description: "Terminal command to display the log file. The file name is appended.",
    type: "string",
    default: "/usr/bin/less -fLQR +G",
    group: "Parametrization",
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
    default: 500,
  },
  actionOnChangeLaunch: {
    label: "Action on loaded launch file change detection",
    type: "string",
    default: "ASK",
    options: ["ASK", "DISMISS", "RELOAD"],
    description: "",
  },
  editorOpenLocation: {
    label: "Location to open editor tab",
    type: "string",
    default: "CENTER",
    options: ["BORDER_TOP", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  nodeLoggerOpenLocation: {
    label: "Location to open log level tab",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  nodeParamOpenLocation: {
    label: "Location to open node parameter tab",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  publisherOpenLocation: {
    label: "Location to open topic publisher tab",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "BORDER_LEFT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  subscriberOpenLocation: {
    label: "Location to open topic subscriber tab",
    type: "string",
    default: "BORDER_RIGHT",
    options: ["BORDER_RIGHT", "BORDER_LEFT", "CENTER", "BORDER_BOTTOM"],
    description: "",
    group: "Window behavior",
  },
  avoidGroupWithOneItem: {
    label: "Avoid groups with one item",
    default: true,
    type: "boolean",
    description: "Do not create a collapsible group with an element in it. Use name with namespace instead.",
  },
  tabFullName: {
    label: "Show tab names with namespace",
    default: true,
    type: "boolean",
    description: "",
  },
  spamNodes: {
    label: "Spam Nodes",
    freeSolo: true,
    type: "string",
    default: ".*_impl_,/*_ros2cli",
    description: "Nodes to be placed in a {SPAM} group.",
    isValid: (value: JSONValue) => {
      let result = true;
      const splits: string[] = (value as string).split(",");
      for (const item of splits) {
        try {
          new RegExp(`/(${item})/`);
          return true;
        } catch (error) {
          result = false;
          return false;
        }
      }
      return result;
    },
    validate: (value: JSONValue) => {
      const splits: string[] = (value as string).split(",");
      const validEntries = splits.filter((item) => {
        try {
          new RegExp(`/(${item})/`);
          return true;
        } catch (error) {
          console.log("error while test");
        }
        return false;
      });
      return validEntries.join(",");
    },
  },
  editorOpenExternal: {
    label: "Open editor in external window by default",
    default: false,
    type: window.commandExecutor ? "boolean" : "none",
    description: "",
    group: "Window behavior",
  },
  logOpenExternal: {
    label: "Open logs in external window by default",
    default: false,
    type: window.commandExecutor ? "boolean" : "none",
    description: "",
    group: "Window behavior",
  },
  screenOpenExternal: {
    label: "Open screen in external window by default",
    default: false,
    type: window.commandExecutor ? "boolean" : "none",
    description: "",
    group: "Window behavior",
  },
  subscriberOpenExternal: {
    label: "Open subscriber in external window by default",
    default: false,
    type: window.commandExecutor ? "boolean" : "none",
    description: "",
    group: "Window behavior",
  },
};

interface ISettingProvider {
  children: React.ReactNode;
}

export const SettingsContext = createContext<ISettingsContext>(DEFAULT_SETTINGS);

export function SettingsProvider({ children }: ISettingProvider): ReturnType<React.FC<ISettingProvider>> {
  const { MIN_VERSION_DAEMON } = DEFAULT_SETTINGS;
  const [changed, forceUpdate] = useReducer((x) => x + 1, 0);
  const [config, setConfig] = useLocalStorage<JSONObject>("SettingsContext:config", {});

  function get(attribute: string): JSONValue | undefined {
    if (attribute in config) {
      return config[attribute];
    }
    if (attribute in SETTINGS_DEF) {
      return SETTINGS_DEF[attribute]?.default;
    }
    throw new Error(`Configuration attribute ${attribute} not found!`);
  }

  function getDefault(attribute: string): JSONValue | undefined {
    if (attribute in SETTINGS_DEF) {
      return SETTINGS_DEF[attribute]?.default;
    }
    throw new Error(`Configuration attribute ${attribute} not found!`);
  }

  function set(attribute: string, value: JSONValue): void {
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
  }

  function getParamList(): { name: string; param: ISettingsParam }[] {
    const params: { name: string; param: ISettingsParam }[] = [];
    for (const key of Object.keys(SETTINGS_DEF)) {
      params.push({ name: key, param: SETTINGS_DEF[key] });
    }
    return params;
  }

  const attributesMemo = useMemo(
    () => ({
      MIN_VERSION_DAEMON,
      changed,
      get,
      getDefault,
      set,
      getParamList,
    }),
    [changed]
  );

  return <SettingsContext.Provider value={attributesMemo}>{children}</SettingsContext.Provider>;
}

export default SettingsContext;
