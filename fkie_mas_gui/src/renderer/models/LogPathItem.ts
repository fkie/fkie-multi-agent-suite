/* eslint-disable camelcase */
/**
 * LogPathItem models the paths for screen and ROS log files.
 */
class LogPathItem {
  /**
   * Unique log item identifier
   */
  id: string;

  node: string;

  screen_log: string;

  screen_log_exists: boolean;

  ros_log: string;

  ros_log_exists: boolean;

  /**
   * Class Constructor
   *
   * @param {string} node - complete node name
   * @param {string} screen_log - the absolute path to the screen log file.
   * @param {boolean} screen_log_exists - False if the file does not exists.
   * @param {string} ros_log - the absolute path to the ros log file.
   * @param {boolean} ros_log_exists - False if the file does not exists.
   */
  constructor(
    node: string,
    screen_log: string,
    screen_log_exists: boolean,
    ros_log: string,
    ros_log_exists: boolean,
  ) {
    this.id = this.guidGenerator();
    this.node = node;
    this.screen_log = screen_log;
    this.screen_log_exists = screen_log_exists;
    this.ros_log = ros_log;
    this.ros_log_exists = ros_log_exists;
  }

  /**
   * Generates an unique string Identifier
   *
   * @return {string} Returns a unique string identifier
   */
  guidGenerator: () => string = () => {
    return Date.now().toString(36) + Math.random().toString(36).substr(2);
  };
}

export default LogPathItem;
