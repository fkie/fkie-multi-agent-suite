import { TResult } from "@/types";

/**
 * Result models return result with additional info message
 */
class Result implements TResult {
  /**
   * Success or not
   */
  result: boolean;

  /**
   * Explaining optional message.
   */
  message: string;

  /**
   * Class Constructor
   *
   * @param {boolean} result - Success or not
   * @param {string} message - Explaining optional message.
   */
  constructor(result: boolean, message: string) {
    this.result = result;
    this.message = message;
  }
}

export default Result;
