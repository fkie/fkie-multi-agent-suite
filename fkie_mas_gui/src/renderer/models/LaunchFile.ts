/**
 * LaunchFile models a launch file.
 */
class LaunchFile {
  /**
   * Absolute path of the launch file.
   */
  path: string;

  /**
   * Starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
   * the nodes are started on the host specified by hostname of the masteruri.
   */
  masteruri: string;

  /**
   * Start nodes of this file on specified host.
   */
  host: string;

  /**
   * Class Constructor
   *
   * @param {string} path - argument name.
   * @param {string} masteruri - argument value.
   * @param {string} host - argument value.
   */
  constructor(path: string, masteruri: string, host: string) {
    this.path = path;
    this.masteruri = masteruri;
    this.host = host;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.path} - ${this.masteruri} - ${this.host}`;
  };
}

export default LaunchFile;
