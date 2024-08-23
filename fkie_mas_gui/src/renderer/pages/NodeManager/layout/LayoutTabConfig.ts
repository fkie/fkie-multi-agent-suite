import CmdType from "../../../providers/CmdType";

export interface ITerminalConfig {
  type: CmdType | undefined;
  providerId: string;
  nodeName: string;
  topicName: string;
  screen: string;
  cmd: string;
}

export interface IEditorConfig {
  id: string,
  host: string;
  port: number;
  rootLaunch: string;
  path: string;
  fileRange: { startLineNumber: number; endLineNumber: number; startColumn: number; endColumn: number } | null;
}

export interface ISubscriberConfig {
  id: string,
  host: string;
  port: number;
  topic: string,
  showOptions: boolean,
  noData: boolean,
}

export default class LayoutTabConfig {
  openExternal: boolean;

  tabType: string;

  terminalConfig: ITerminalConfig | null;

  editorConfig: IEditorConfig | null;

  subscriberConfig: ISubscriberConfig | null;

  constructor(
    openExternal: boolean = false,
    tabType: string = "",
    terminalConfig: ITerminalConfig | null = null,
    editorConfig: IEditorConfig | null = null,
    subscriberConfig: ISubscriberConfig | null = null
  ) {
    this.openExternal = openExternal;
    this.tabType = tabType;
    this.terminalConfig = terminalConfig;
    this.editorConfig = editorConfig;
    this.subscriberConfig = subscriberConfig;
  }
}
