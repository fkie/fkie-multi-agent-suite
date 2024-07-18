/* eslint-disable camelcase */
/**
 * SubscriberNode creates a subscriber node which subscribes a given topic.
 */
class SubscriberFilter {
  /**
   * Report only statistics without message content.
   */
  no_data: boolean;

  /**
   * exclude arrays.
   */
  no_arr: boolean;

  /**
   * exclude string fields.
   */
  no_str: boolean;

  /**
   * rate to forward messages. Ignored on latched topics. Disabled by 0. Default: 1
   */
  hz: number;

  /**
   * window size, in # of messages, for calculating rate
   */
  window: number;

  /**
   * Class Constructor
   *
   * @param {boolean} no_data - Report only statistics without message content.
   * @param {boolean} no_arr - exclude arrays.
   * @param {boolean} no_str - exclude string fields.
   * @param {number} hz - rate to forward messages. Ignored on latched topics. Disabled by 0. Default: 1
   * @param {number} window - window size, in # of messages, for calculating rate
   */
  constructor(no_data = false, no_arr = false, no_str = false, hz = 1, window = 0) {
    this.no_data = no_data;
    this.no_arr = no_arr;
    this.no_str = no_str;
    this.hz = hz;
    this.window = window;
  }
}

export default SubscriberFilter;
