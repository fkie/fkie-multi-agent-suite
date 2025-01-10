/* eslint-disable camelcase */
import RosQos from "./RosQos";
import SubscriberFilter from "./SubscriberFilter";

/**
 * SubscriberNode creates a subscriber node which subscribes a given topic.
 */
export default class SubscriberNode {
  /**
   * Name of the ROS topic to listen to (e.g. '/chatter').
   */
  topic: string;

  /**
   * Type of the ROS message (e.g. 'std_msgs/msg/String')
   */
  message_type: string;

  /**
   * use the TCP_NODELAY transport hint when subscribing to topics (Only ROS1)
   */
  tcp_no_delay: boolean;

  /**
   * Enable ROS simulation time (Only ROS2)
   */
  use_sim_time: boolean;

  /**
   * Filter class
   */
  filter: SubscriberFilter;

  /**
   * Quality of service subscription options (Only ROS2).
   */
  qos: RosQos;

  constructor(
    topic: string,
    message_type: string,
    tcp_no_delay = false,
    use_sim_time = false,
    filter = new SubscriberFilter(),
    qos: RosQos = new RosQos()
  ) {
    this.topic = topic;
    this.message_type = message_type;
    this.tcp_no_delay = tcp_no_delay;
    this.use_sim_time = use_sim_time;
    this.filter = filter;
    this.qos = qos;
  }
}
