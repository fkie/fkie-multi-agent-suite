/* eslint-disable camelcase */
import { JSONObject } from "@/types";

/**
 * LaunchPublishMessage models the message to publish to a ROS topic.
 */
class LaunchPublishMessage {
  /**
   * The ROS topic name.
   */
  topic_name: string;

  /**
   * Type of the message.
   */
  msg_type: string;

  /**
   * Dictionary structure of the ROS message as JSON string.
   */
  data: string;

  /**
   * Publishing rate (hz), only if once and latched is False.
   */
  rate: number;

  /**
   * Publish one message and exit.
   */
  once: boolean;

  /**
   * Enable latching.
   */
  latched: boolean;

  /**
   * Print verbose output.
   */
  verbose: boolean;

  /**
   * Use rostime for time stamps, else walltime is used.
   */
  use_rostime: boolean;

  /**
   * When publishing with a rate, performs keyword ('now' or 'auto') substitution for each message.
   */
  substitute_keywords: boolean;

  /**
   * Class Constructor
   *
   * @param {string} topic_name - The ROS topic name.
   * @param {string} msg_type - Type of the message.
   * @param {JSONObject | null} data - Dictionary structure of the ROS message as JSON string.
   * @param {number} rate - Publishing rate (hz), only if once and latched is False.
   * @param {boolean} once - Publish one message and exit.
   * @param {boolean} latched - Enable latching.
   * @param {boolean} verbose - Print verbose output.
   * @param {boolean} use_rostime - Use rostime for time stamps, else walltime is used.
   * @param {boolean} substitute_keywords - When publishing with a rate, performs keyword ('now' or 'auto') substitution for each message.
   */
  constructor(
    topic_name: string,
    msg_type: string,
    data: JSONObject | null,
    rate: number,
    once: boolean,
    latched: boolean,
    verbose: boolean,
    use_rostime: boolean,
    substitute_keywords: boolean
  ) {
    this.topic_name = topic_name;
    this.msg_type = msg_type;
    this.data = JSON.stringify(data);
    this.rate = rate;
    this.once = once;
    this.latched = latched;
    this.verbose = verbose;
    this.use_rostime = use_rostime;
    this.substitute_keywords = substitute_keywords;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.topic_name} [${this.msg_type}]`;
  };
}

export default LaunchPublishMessage;
