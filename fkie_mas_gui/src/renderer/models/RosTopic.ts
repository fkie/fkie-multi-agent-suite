import { generateUniqueId } from "../utils";
import RosQos from "./RosQos";

type IncompatibleQos = {
  node_id: string;
  compatibility: string;
  reason: string;
};

type EndpointInfo = {
  node_id: string;
  qos: RosQos | undefined;
  incompatible_qos: IncompatibleQos[] | undefined;
};

/**
 * RosTopic models topics in a ROS system
 */
class RosTopic {
  /**
   * Topic unique ID
   */
  id: string;

  /**
   * Topic name including namespace
   */
  name: string;

  /**
   * List of types, in case of ROS1 the list only contains one element.
   */
  msg_type: string;

  /**
   * List of ROS nodes publish to this topic.
   */
  publisher: EndpointInfo[];

  /**
   * List of ROS nodes subscribe to this topic.
   */
  subscriber: EndpointInfo[];

  /**
   * Class Constructor
   *
   * @param {string} name - Topic name including namespace
   * @param {string} msg_type - Message type.
   * @param {string[]} publisher - List of ROS nodes publish to this topic.
   * @param {string[]} subscriber - List of ROS nodes subscribe to this topic.
   */
  constructor(
    name: string,
    msg_type: string,
    publisher: EndpointInfo[] = [],
    subscriber: EndpointInfo[] = [],
    id: string | undefined = undefined
  ) {
    this.id = id || generateUniqueId();
    this.name = name;
    this.msg_type = msg_type;
    this.publisher = publisher;
    this.subscriber = subscriber;
  }

  /**
   * Generates a string representation of the node
   *
   * @return {string} Node description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.msg_type}`;
  };
}

export default RosTopic;
