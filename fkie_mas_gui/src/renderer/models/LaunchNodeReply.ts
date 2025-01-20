/* eslint-disable camelcase */
import LaunchReturnStatus from "./LaunchReturnStatus";

/**
 * LaunchNodeReply models the response for start of a node.
 */
export default class LaunchNodeReply {
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
  paths: string[] | undefined;

  /**
   * A list with names launch files, only if MULTIPLE_LAUNCHES is returned.
   */
  launch_files: string[] | undefined;

  constructor(name: string, status: LaunchReturnStatus, paths: string[], launch_files: string[]) {
    this.name = name;
    this.status = status;
    this.paths = paths;
    this.launch_files = launch_files;
  }
}
