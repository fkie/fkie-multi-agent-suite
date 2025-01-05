/* eslint-disable camelcase */
import { TRosMessageStruct } from "./TRosMessageStruct";
/**
 * LaunchMessageStruct models the message struct for a ROS message type.
 */
class LaunchMessageStruct {
  /**
   * Type of the message.
   */
  msg_type: string;

  /**
   * Dictionary structure of the ROS message as JSON.
   */
  data: TRosMessageStruct;

  /**
   * True if the data for the message type was loaded successfully.
   */
  valid: boolean;

  /**
   * Error message if valid is False.
   */
  error_msg: string;

  /**
   * Class Constructor
   *
   * @param {string} msg_type - Type of the message.
   * @param {TRosMessageStruct} data - Dictionary structure of the ROS message as JSON string.
   * @param {boolean} valid - True if the data for the message type was loaded successfully.
   * @param {string} error_msg - Error message if valid is False.
   */
  constructor(msg_type: string, data: TRosMessageStruct, valid: boolean, error_msg: string) {
    this.msg_type = msg_type;
    this.data = data;
    this.valid = valid;
    this.error_msg = error_msg;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.msg_type}`;
  };
}

export default LaunchMessageStruct;
