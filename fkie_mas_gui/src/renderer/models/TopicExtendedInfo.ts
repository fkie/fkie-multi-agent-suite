import RosTopic, { EndpointInfo } from "./RosTopic";

export default class TopicExtendedInfo {
  id: string;

  name: string;

  msgType: string = "";

  providerId: string = "";

  providerName: string = "";

  publishers: EndpointInfo[] = [];

  subscribers: EndpointInfo[] = [];

  constructor(topic: RosTopic, providerId: string, providerName: string) {
    this.id = `${topic.name}/${providerName}`;
    this.name = topic.name;
    this.msgType = topic.msg_type;
    this.providerId = providerId;
    this.providerName = providerName;
    this.addPublishers(topic.publisher);
    this.addSubscribers(topic.subscriber);
  }

  addPublishers(publishers: EndpointInfo[]): void {
    publishers.forEach((pub: EndpointInfo) => {
      if (this.publishers.filter((item) => item.node_id === pub.node_id).length === 0) {
        this.publishers.push(pub);
      }
    });
  }

  addSubscribers(subscribers: EndpointInfo[]): void {
    subscribers.forEach((sub: EndpointInfo) => {
      if (this.subscribers.filter((item) => item.node_id === sub.node_id).length === 0) {
        this.subscribers.push(sub);
      }
    });
  }

  public add(topic: RosTopic): void {
    this.addPublishers(topic.publisher);
    this.addSubscribers(topic.subscriber);
  }
}
