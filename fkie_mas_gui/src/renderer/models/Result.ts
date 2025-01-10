import { TResult } from "@/types";

/**
 * Result models return result with additional info message
 */
export default class Result implements TResult {
  /**
   * Success or not
   */
  result: boolean;

  /**
   * Explaining optional message.
   */
  message: string;

  constructor(result: boolean, message: string) {
    this.result = result;
    this.message = message;
  }
}
