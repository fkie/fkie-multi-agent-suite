/**
 * LaunchFile models a launch file.
 */
export default class LaunchFile {
  /**
   * Absolute path of the launch file.
   */
  path: string;

  /**
   * Starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
   * the nodes are started on the host specified by hostname of the masteruri.
   */
  masteruri: string | undefined;

  /**
   * Start nodes of this file on specified host.
   */
  host: string;

  constructor(path: string, masteruri: string, host: string) {
    this.path = path;
    this.masteruri = masteruri;
    this.host = host;
  }
}
