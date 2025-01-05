import RosService from "./RosService";

export type TServiceNodeInfo = { nodeName: string; nodeId: string };

export default class ServiceExtendedInfo {
  id: string;

  name: string;

  srvType = "";

  providerId = "";

  providerName = "";

  nodeProviders: TServiceNodeInfo[] = [];

  nodeRequester: TServiceNodeInfo[] = [];

  constructor(service: RosService, providerId: string, providerName: string) {
    this.id = `${service.name}/${providerName}`;
    this.name = service.name;
    this.srvType = service.srv_type;
    this.providerId = providerId;
    this.providerName = providerName;
  }

  addProvider(nodeName: string, nodeId: string) {
    this.nodeProviders.push({ nodeName, nodeId });
  }
  addRequester(nodeName: string, nodeId: string) {
    this.nodeRequester.push({ nodeName, nodeId });
  }
}
