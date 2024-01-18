/**
 * LaunchReturnStatus models return status while handle launch tasks
 */
class LaunchReturnStatus {
  /**
   * Return code, one of follow stings
   * OK
   * ERROR
   * ALREADY_OPEN
   * MULTIPLE_BINARIES
   * MULTIPLE_LAUNCHES
   * PARAMS_REQUIRED
   * FILE_NOT_FOUND
   * NODE_NOT_FOUND
   * PACKAGE_NOT_FOUND
   * CONNECTION_ERROR
   */
  code: string;

  /**
   * Explaning optional message.
   */
  msg: string;

  /**
   * Class Constructor
   *
   * @param {string} code - Return code
   * @param {string} msg - Explaning optional message.
   */
  constructor(code: string, msg: string) {
    this.code = code;
    this.msg = msg;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.code}`;
  };
}

export default LaunchReturnStatus;
