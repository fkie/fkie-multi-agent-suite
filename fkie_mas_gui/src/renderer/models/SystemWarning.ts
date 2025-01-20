/**
 * SystemWarning models a warning from system nodes on ROS side.
 */
export default class SystemWarning {
  /**
   * Short warning message.
   */
  msg: string;

  /**
   * Long description.
   */
  details: string;

  /**
   * Note on the possible solution.
   */
  hint: string | undefined;

  constructor(msg: string, details: string, hint: string) {
    this.msg = msg;
    this.details = details;
    this.hint = hint;
  }
}
