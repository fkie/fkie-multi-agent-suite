export default class CmdType extends String {
  static CMD = new CmdType('cmd');

  static SCREEN = new CmdType('screen');

  static LOG = new CmdType('log');

  static TERMINAL = new CmdType('terminal');

  static ECHO = new CmdType('echo');
}
