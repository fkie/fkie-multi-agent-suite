/* eslint-disable camelcase */
/**
 * SubscriberNode creates a subscriber node which subscribes a given topic.
 */
export default class SubscriberFilter {
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

  arrayItemsCount: number;

  constructor(no_data = false, no_arr = false, no_str = false, hz = 1, window = 0, arrayItemsCount = 15) {
    this.no_data = no_data;
    this.no_arr = no_arr;
    this.no_str = no_str;
    this.hz = hz;
    this.window = window;
    this.arrayItemsCount = arrayItemsCount;
  }
}
