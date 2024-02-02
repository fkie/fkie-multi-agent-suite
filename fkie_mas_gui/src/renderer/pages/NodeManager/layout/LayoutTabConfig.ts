import { CmdType } from '../../../providers';

export default class LayoutTabConfig {
  openExternal: boolean;

  tabType: string;

  terminalConfig: {
    type: CmdType | undefined;
    providerId: string;
    nodeName: string;
    topicName: string;
    screen: string;
    cmd: string;
  };

  constructor(
    openExternal: boolean = false,
    tabType: string = '',
    terminalConfig: {
      type: CmdType | undefined;
      providerId: string;
      nodeName: string;
      topicName: string;
      screen: string;
      cmd: string;
    } = {
      type: undefined,
      providerId: '',
      nodeName: '',
      topicName: '',
      screen: '',
      cmd: '',
    },
  ) {
    this.openExternal = openExternal;
    this.tabType = tabType;
    this.terminalConfig = terminalConfig;
  }
}
