/* eslint-disable camelcase */
import LaunchReturnStatus from './LaunchReturnStatus';

/**
 * LaunchNodeReply models the response for start of a node.
 */
class LaunchNodeReply {
  /**
   * Name of the node.
   */
  name: string;

  /**
   * Return status with code and message.
   */
  status: LaunchReturnStatus;

  /**
   * A list of paths with binaries for a node, only if MULTIPLE_BINARIES is returned.
   */
  paths: string[];

  /**
   * A list with names launch files, only if MULTIPLE_LAUNCHES is returned.
   */
  launch_files: string[];

  /**
   * Class Constructor
   *
   * @param {string} name - Name of the node.
   * @param {LaunchReturnStatus} status - Return status with code and message.
   * @param {string[]} paths - A list of paths with binaries for a node, only if MULTIPLE_BINARIES is returned.
   * @param {string[]} launch_files - A list with names launch files, only if MULTIPLE_LAUNCHES is returned.
   */
  constructor(
    name: string,
    status: LaunchReturnStatus,
    paths: string[],
    launch_files: string[],
  ) {
    this.name = name;
    this.status = status;
    this.paths = paths;
    this.launch_files = launch_files;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.status}`;
  };
}

export default LaunchNodeReply;
