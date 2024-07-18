/* eslint-disable camelcase */
/**
 * SubscriberEvent models a received ROS message and statistics for a topic.
 */
class SubscriberEvent {
  /**
   * Name of the ROS topic to listen to (e.g. '/chatter').
   */
  topic: string;

  /**
   * Type of the ROS message (e.g. 'std_msgs/msg/String')
   */
  message_type: string;

  latched: boolean;

  /**
   * Dictionary of the ROS message as JSON string.
   */
  data: any | null;

  /**
   * Count of received messages since the start.
   */
  count: number;

  /**
   * Receive rate on the topic
   */
  rate: number;

  /**
   * Bandwidth
   */
  bw: number;

  bw_min: number;

  bw_max: number;

  /**
   * Delay
   */
  delay: number;

  delay_min: number;

  delay_max: number;

  /**
   * Message size in bytes
   */
  size: number;

  size_min: number;

  size_max: number;

  /**
   * Class Constructor
   *
   * @param {string} topic - Name of the ROS topic to listen to (e.g. '/chatter').
   * @param {string} message_type - Type of the ROS message (e.g. 'std_msgs/msg/String')
   * @param {any | null} data - Dictionary of the ROS message as JSON string.
   * @param {number} count - Count of received messages since the start.
   * @param {number} rate - Receive rate on the topic
   */
  constructor(
    topic: string,
    message_type: string,
    latched: boolean,
    data: any | null,
    count: number,
    rate: number,
    bw: number,
    bw_min: number,
    bw_max: number,
    delay: number,
    delay_min: number,
    delay_max: number,
    size: number,
    size_min: number,
    size_max: number
  ) {
    this.topic = topic;
    this.message_type = message_type;
    this.latched = latched;
    this.data = data;
    this.count = count;
    this.rate = rate;
    this.bw = bw;
    this.bw_min = bw_min;
    this.bw_max = bw_max;
    this.delay = delay;
    this.delay_min = delay_min;
    this.delay_max = delay_max;
    this.size = size;
    this.size_min = size_min;
    this.size_max = size_max;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.topic}`;
  };
}

export default SubscriberEvent;
