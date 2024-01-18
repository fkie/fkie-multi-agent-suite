/* eslint-disable camelcase */
import LaunchArgument from './LaunchArgument';
import LaunchReturnStatus from './LaunchReturnStatus';

/**
 * LaunchLoadReply models the reply for load of the launch file.
 */
class LaunchLoadReply {
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

  /**
   * Class Constructor
   *
   * @param {LaunchReturnStatus} status - Return status with code and message.
   * @param {string[]} paths - All paths found for package/launch file combination.
   * @param {LaunchArgument[]} args - The list of arguments used or required for a load of the launch file.
   * @param {string[]} changed_nodes - The list of nodes with changed configuration after reload the launch file.
   */
  constructor(
    status: LaunchReturnStatus,
    paths: string[],
    args: LaunchArgument[],
    changed_nodes: string[],
  ) {
    this.status = status;
    this.paths = paths;
    this.args = args;
    this.changed_nodes = changed_nodes;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.status} - ${this.paths}`;
  };
}

export default LaunchLoadReply;
