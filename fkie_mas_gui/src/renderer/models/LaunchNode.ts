/* eslint-disable camelcase */
/**
 * LaunchNode models the request to start a node.
 */
export default class LaunchNode {
  /**
   * Full name of the ros node exists in the launch file.
   */
  name: string;

  /**
   * The full path of the binary. Used in case of multiple binaries in the same package.
   */
  opt_binary: string | undefined;

  /**
   * Full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
   */
  opt_launch: string | undefined;

  /**
   * Log level
   */
  loglevel: string | undefined;

  /**
   * Log format
   */
  logformat: string | undefined;

  /**
   * Starts nodes with specified ROS_MASTER_URI.
   */
  masteruri: string | undefined;

  /**
   * Reload all global parameter if True.
   */
  reload_global_param: boolean;

  /**
   * Custom command prefix. It will be prepended before launch prefix.
   */
  cmd_prefix: string | undefined;

  ignore_timer: boolean = false;

  constructor(
    name: string,
    opt_binary: string,
    opt_launch: string,
    loglevel: string,
    logformat: string,
    masteruri: string,
    reload_global_param: boolean,
    cmd_prefix: string,
    ignore_timer: boolean
  ) {
    this.name = name;
    this.opt_binary = opt_binary;
    this.opt_launch = opt_launch;
    this.loglevel = loglevel;
    this.logformat = logformat;
    this.masteruri = masteruri;
    this.reload_global_param = reload_global_param;
    this.cmd_prefix = cmd_prefix;
    this.ignore_timer = ignore_timer;
  }
}
