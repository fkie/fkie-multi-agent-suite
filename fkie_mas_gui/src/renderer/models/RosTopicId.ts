/**
 * RosTopic models topics in a ROS system
 */
export default class RosTopicId {
  /**
   * Topic name including namespace
   */
  name: string;

  /**
   * List of types, in case of ROS1 the list only contains one element.
   */
  msg_type: string;

  constructor(name: string, msg_type: string) {
    this.name = name;
    this.msg_type = msg_type;
  }
}
