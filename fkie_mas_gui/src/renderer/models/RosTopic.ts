import { generateUniqueId } from "../utils";
import RosQos from "./RosQos";

export type IncompatibleQos = {
  node_id: string;
  compatibility: string;
  reason: string;
};

export type EndpointInfo = {
  node_id: string;
  qos: RosQos | undefined;
  incompatible_qos: IncompatibleQos[] | undefined;
};

/**
 * RosTopic models topics in a ROS system
 */
export default class RosTopic {
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
}
