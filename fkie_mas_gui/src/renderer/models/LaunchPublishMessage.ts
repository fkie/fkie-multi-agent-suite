/* eslint-disable camelcase */
import { JSONObject } from "@/types";
import RosQos from "./RosQos";

/**
 * LaunchPublishMessage models the message to publish to a ROS topic.
 */
export default class LaunchPublishMessage {
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
   * Quality of service subscription options (Only ROS2).
   */
  qos: RosQos | undefined;

  constructor(
    topic_name: string,
    msg_type: string,
    data: JSONObject | null,
    rate: number,
    once: boolean,
    latched: boolean,
    verbose: boolean,
    use_rostime: boolean,
    substitute_keywords: boolean,
    qos: RosQos = new RosQos()
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
    this.qos = qos;
  }
}
