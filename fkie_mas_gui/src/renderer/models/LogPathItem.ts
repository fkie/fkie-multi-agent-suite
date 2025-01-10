import { generateUniqueId } from "../utils";
/**
 * LogPathItem models the paths for screen and ROS log files.
 */
export default class LogPathItem {
  /**
   * Unique log item identifier
   */
  id: string;

  node: string;

  screen_log: string;

  screen_log_exists: boolean;

  ros_log: string;

  ros_log_exists: boolean;

  constructor(node: string, screen_log: string, screen_log_exists: boolean, ros_log: string, ros_log_exists: boolean) {
    this.id = generateUniqueId();
    this.node = node;
    this.screen_log = screen_log;
    this.screen_log_exists = screen_log_exists;
    this.ros_log = ros_log;
    this.ros_log_exists = ros_log_exists;
  }
}
