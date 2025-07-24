import RosNode from "./RosNode";
import RosService from "./RosService";

export type TServiceNodeInfo = {
  nodeName: string;
  nodeId: string;
  isLocal: boolean;
  providerId: string;
  providerName: string;
};

export default class ServiceExtendedInfo {
  id: string;

  name: string;

  srvType = "";

  rosService: RosService;

  nodeProviders: TServiceNodeInfo[] = [];

  nodeRequester: TServiceNodeInfo[] = [];

  constructor(service: RosService) {
    this.id = `${service.name}/`;
    this.name = service.name;
    this.srvType = service.srv_type;
    this.rosService = service;
  }

  public addProvider(node: RosNode): void {
    for (const provider of this.rosService.provider || []) {
      if (provider !== node.id) {
        continue;
      }
      if (node.isLocal) {
        // remove all not local nodes
        this.nodeProviders = this.nodeProviders.filter((item) => item.isLocal);
      }
      const hasNode = this.nodeProviders.filter((item) => item.nodeId === node.id).length > 0;
      if (!hasNode || node.isLocal) {
        // if we already added a local node, we add only further local nodes
        this.nodeProviders.push({
          nodeName: node.name,
          nodeId: node.id,
          isLocal: node.isLocal,
          providerId: node.providerId,
          providerName: node.providerName,
        });
      }
    }
  }
  public addRequester(node: RosNode): void {
    for (const requester of this.rosService.requester || []) {
      if (requester !== node.id) {
        continue;
      }
      if (node.isLocal) {
        // remove all not local nodes and add this node
        this.nodeRequester = this.nodeRequester.filter((item) => item.isLocal);
      }
      const hasNode = this.nodeRequester.filter((item) => item.nodeId === node.id).length > 0;
      if (!hasNode || node.isLocal) {
        // if we already added a local node, we add only further local nodes
        this.nodeRequester.push({
          nodeName: node.name,
          nodeId: node.id,
          isLocal: node.isLocal,
          providerId: node.providerId,
          providerName: node.providerName,
        });
      }
    }
  }

  public add(node: RosNode): void {
    this.addProvider(node);
    this.addRequester(node);
  }
}
