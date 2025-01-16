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

  publishers: EndpointExtendedInfo[] = [];

  subscribers: EndpointExtendedInfo[] = [];

  constructor(topic: RosTopic, node: RosNode) {
    this.id = `${topic.name}-${topic.msg_type}`;
    this.name = topic.name;
    this.msgType = topic.msg_type;
    this.addPublishers(topic.publisher, node);
    this.addSubscribers(topic.subscriber, node);
  }

  addPublishers(publishers: EndpointInfo[], node: RosNode): void {
    publishers.forEach((pub: EndpointInfo) => {
      if (
        this.publishers.filter((item) => item.info.node_id === pub.node_id && item.providerId === node.providerId)
          .length === 0
      ) {
        this.publishers.push({ info: pub, providerId: node.providerId, providerName: node.providerName });
      }
    });
  }

  addSubscribers(subscribers: EndpointInfo[], node: RosNode): void {
    subscribers.forEach((sub: EndpointInfo) => {
      if (
        this.subscribers.filter((item) => item.info.node_id === sub.node_id && item.providerId === node.providerId)
          .length === 0
      ) {
        this.subscribers.push({ info: sub, providerId: node.providerId, providerName: node.providerName });
      }
    });
  }

  public add(topic: RosTopic, node: RosNode): void {
    this.addPublishers(topic.publisher, node);
    this.addSubscribers(topic.subscriber, node);
  }
}
