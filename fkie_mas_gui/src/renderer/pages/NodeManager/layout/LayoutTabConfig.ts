import { CmdType } from "@/renderer/providers";
import { TFileRange, TLaunchArg } from "@/types";

export interface IExtTerminalConfig {
  type: CmdType;
  providerId: string;
  nodeName: string;
  topicName: string;
  screen: string;
  cmd: string;
}

export interface IEditorConfig {
  id: string;
  host: string;
  port: number;
  rootLaunch: string;
  path: string;
  fileRange: TFileRange | null;
  launchArgs: TLaunchArg[];
}

export interface IPublisherConfig {
  id: string;
  host: string;
  port: number;
  topicName: string;
  topicType: string;
}

export interface ISubscriberConfig {
  id: string;
  host: string;
  port: number;
  topic: string;
  showOptions: boolean;
  noData: boolean;
}

export interface ITerminalConfig {
  id: string;
  host: string;
  port: number;
  cmdType: CmdType;
  node: string;
  screen: string;
  cmd: string;
}

export default class LayoutTabConfig {
  openExternal: boolean;

  tabType: CmdType;

  extTerminalConfig: IExtTerminalConfig | null;

  editorConfig: IEditorConfig | null;

  publisherConfig: IPublisherConfig | null;

  subscriberConfig: ISubscriberConfig | null;

  terminalConfig: ITerminalConfig | null;

  filterText: string;

  constructor(
    openExternal: boolean = false,
    tabType: CmdType = "",
    extTerminalConfig: IExtTerminalConfig | null = null,
    editorConfig: IEditorConfig | null = null,
    subscriberConfig: ISubscriberConfig | null = null,
    terminalConfig: ITerminalConfig | null = null,
    publisherConfig: IPublisherConfig | null = null,
    filterText: string = ""
  ) {
    this.openExternal = openExternal;
    this.tabType = tabType;
    this.extTerminalConfig = extTerminalConfig;
    this.editorConfig = editorConfig;
    this.publisherConfig = publisherConfig;
    this.subscriberConfig = subscriberConfig;
    this.terminalConfig = terminalConfig;
    this.filterText = filterText;
  }
}
