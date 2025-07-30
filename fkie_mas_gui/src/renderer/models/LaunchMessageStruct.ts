/* eslint-disable camelcase */
import { TRosMessageStruct } from "../../types/TRosMessageStruct";
/**
 * LaunchMessageStruct models the message struct for a ROS message type.
 */
export default class LaunchMessageStruct {
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

  constructor(msg_type: string, data: TRosMessageStruct, valid: boolean, error_msg: string) {
    this.msg_type = msg_type;
    this.data = data;
    this.valid = valid;
    this.error_msg = error_msg;
  }
}
