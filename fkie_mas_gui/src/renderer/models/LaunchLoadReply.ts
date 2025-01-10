/* eslint-disable camelcase */
import LaunchArgument from "./LaunchArgument";
import LaunchReturnStatus from "./LaunchReturnStatus";

/**
 * LaunchLoadReply models the reply for load of the launch file.
 */
export default class LaunchLoadReply {
  /**
   * Return status with code and message.
   */
  status: LaunchReturnStatus;

  /**
   * All paths found for package/launch file combination.
   */
  paths: string[];

  /**
   * The list of arguments used or required for a load of the launch file.
   */
  args: LaunchArgument[];

  /**
   * The list of nodes with changed configuration after reload the launch file.
   */
  changed_nodes: string[];

  constructor(status: LaunchReturnStatus, paths: string[], args: LaunchArgument[], changed_nodes: string[]) {
    this.status = status;
    this.paths = paths;
    this.args = args;
    this.changed_nodes = changed_nodes;
  }
}
