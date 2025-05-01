import RosNode from "./RosNode";
import RosTopic, { EndpointInfo } from "./RosTopic";

export type EndpointExtendedInfo = {
  info: EndpointInfo;
  providerId: string;
  providerName: string;
};

export default class TopicExtendedInfo {
  id: string;

  name: string;

  msgType: string = "";

  hasQos: boolean = false;

  hasIncompatibleQos: boolean = false;

  publishers: EndpointExtendedInfo[] = [];

  subscribers: EndpointExtendedInfo[] = [];

  constructor(topic: RosTopic, node: RosNode) {
    this.id = `${topic.name}-${topic.msg_type}`;
    this.name = topic.name;
    this.msgType = topic.msg_type;
    this.addPublishers(topic.publisher, node);
    this.addSubscribers(topic.subscriber, node);
  }

  addPublishers(publishers: EndpointInfo[] | undefined, node: RosNode): void {
    for (const pub of publishers || []) {
      if (
        this.publishers.filter((item) => item.info.node_id === pub.node_id && item.providerId === node.providerId)
          .length === 0
      ) {
        this.publishers.push({ info: pub, providerId: node.providerId, providerName: node.providerName });
      }
      if (pub.qos) {
        this.hasQos = true;
      }
      if (pub.incompatible_qos && pub.incompatible_qos.length > 0) {
        this.hasIncompatibleQos = true;
      }
    }
  }

  addSubscribers(subscribers: EndpointInfo[] | undefined, node: RosNode): void {
    for (const sub of subscribers || []) {
      if (
        this.subscribers.filter((item) => item.info.node_id === sub.node_id && item.providerId === node.providerId)
          .length === 0
      ) {
        this.subscribers.push({ info: sub, providerId: node.providerId, providerName: node.providerName });
      }
      if (sub.qos) {
        this.hasQos = true;
      }
      if (sub.incompatible_qos && sub.incompatible_qos.length > 0) {
        this.hasIncompatibleQos = true;
      }
    }
  }

  public add(topic: RosTopic, node: RosNode): void {
    this.addPublishers(topic.publisher, node);
    this.addSubscribers(topic.subscriber, node);
  }
}
