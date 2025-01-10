/**
 * LaunchReturnStatus models return status while handle launch tasks
 */
export default class LaunchReturnStatus {
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
   * Explaining optional message.
   */
  msg: string;

  constructor(code: string, msg: string) {
    this.code = code;
    this.msg = msg;
  }
}
