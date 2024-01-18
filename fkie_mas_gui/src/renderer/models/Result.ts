/**
 * Result models return result with additional info message
 */
class Result {
  /**
   * Success or not
   */
  result: boolean;

  /**
   * Explaning optional message.
   */
  message: string;

  /**
   * Class Constructor
   *
   * @param {boolean} result - Success or not
   * @param {string} message - Explaning optional message.
   */
  constructor(result: boolean, message: string) {
    this.result = result;
    this.message = message;
  }
}

export default Result;
