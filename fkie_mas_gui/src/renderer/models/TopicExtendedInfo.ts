import RosNode from "./RosNode";
import RosTopic, { EndpointInfo } from "./RosTopic";

export type EndpointExtendedInfo = {
  info: EndpointInfo;
  providerId: string;
  providerName: string;
  localNode: boolean;
};

export default class TopicExtendedInfo {
  id: string;

  name: string;

  msgType: string = "";

  hasQos: boolean = false;

  hasIncompatibleQos: boolean = false;

  rosTopic: RosTopic;

  publishers: EndpointExtendedInfo[] = [];

  subscribers: EndpointExtendedInfo[] = [];

  constructor(topic: RosTopic, node?: RosNode) {
    this.id = `${topic.name}-${topic.msg_type}`;
    this.name = topic.name;
    this.msgType = topic.msg_type;
    this.rosTopic = topic;
    this._updateQos();
    if (node) {
      this.addPublishers(node);
      this.addSubscribers(node);
    }
  }

  _updateQos(): void {
    for (const pub of this.rosTopic.publisher || []) {
      if (pub.qos) {
        this.hasQos = true;
      }
      if (pub.incompatible_qos && pub.incompatible_qos.length > 0) {
        this.hasIncompatibleQos = true;
      }
    }
    for (const sub of this.rosTopic.subscriber || []) {
      if (sub.qos) {
        this.hasQos = true;
      }
      if (sub.incompatible_qos && sub.incompatible_qos.length > 0) {
        this.hasIncompatibleQos = true;
      }
    }
  }

  addPublishers(node: RosNode): void {
    for (const pub of this.rosTopic.publisher || []) {
      if (pub.node_id !== node.id) {
        continue;
      }
      if (node.isLocal) {
        // remove all not local nodes and add this node
        this.publishers = this.publishers.filter((item) => item.localNode);
      }
      const hasNode = this.publishers.filter((item) => item.info.node_id === node.id).length > 0;
      if (!hasNode || node.isLocal) {
        // if we already added a local node, we add only further local nodes
        this.publishers.push({
          info: pub,
          providerId: node.providerId,
          providerName: node.providerName,
          localNode: node.isLocal,
        });
      }
    }
  }

  addSubscribers(node: RosNode): void {
    for (const sub of this.rosTopic.subscriber || []) {
      if (sub.node_id !== node.id) {
        continue;
      }
      if (node.isLocal) {
        // remove all not local nodes and add this node
        this.subscribers = this.subscribers.filter((item) => item.localNode);
      }
      const hasNode = this.subscribers.filter((item) => item.info.node_id === node.id).length > 0;
      if (!hasNode || node.isLocal) {
        // if we already added a local node, we add only further local nodes
        this.subscribers.push({
          info: sub,
          providerId: node.providerId,
          providerName: node.providerName,
          localNode: node.isLocal,
        });
      }
    }
  }

  public add(node: RosNode): void {
    this.addPublishers(node);
    this.addSubscribers(node);
  }
}
