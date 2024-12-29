import { generateUniqueId } from "../utils";
import RosQos from "./RosQos";

type IncompatibleQos = {
  node_id: string;
  compatibility: string;
  reason: string;
}

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
  msgtype: string;

  /**
   * List of ROS nodes publish to this topic.
   */
  publisher: string[];

  /**
   * List of ROS nodes subscribe to this topic.
   */
  subscriber: string[];

  qos: RosQos;

  incompatible_qos: IncompatibleQos[];

  /**
   * Class Constructor
   *
   * @param {string} name - Topic name including namespace
   * @param {string} msgtype - Message type.
   * @param {string[]} publisher - List of ROS nodes publish to this topic.
   * @param {string[]} subscriber - List of ROS nodes subscribe to this topic.
   */
  constructor(
    name: string,
    msgtype: string,
    publisher: string[] = [],
    subscriber: string[] = [],
    qos: RosQos = new RosQos(),
    incompatibleQos: IncompatibleQos[] | undefined = undefined
  ) {
    this.id = generateUniqueId();
    this.name = name;
    this.msgtype = msgtype;
    this.publisher = publisher;
    this.subscriber = subscriber;
    this.qos = qos;
    this.incompatible_qos = incompatibleQos ? incompatibleQos : [];
  }

  /**
   * Generates a string representation of the node
   *
   * @return {string} Node description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.msgtype}`;
  };
}

export default RosTopic;
