/**
 * SystemWarning models a warning from system nodes on ROS side.
 */
class SystemWarning {
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
  hint: string;

  /**
   * Class Constructor
   *
   * @param {string} msg - Short warning message.
   * @param {string} details - Long description.
   * @param {hint} details - Note on the possible solution.
   */
  constructor(msg: string, details: string, hint: string) {
    this.msg = msg;
    this.details = details;
    this.hint = hint;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.msg}`;
  };
}

export default SystemWarning;
