/* eslint-disable camelcase */
/**
 * RosService models topics in a ROS system
 */
export default class RosService {
  /**
   * Topic name including namespace
   */
  name: string;

  /**
   * List of types, in case of ROS1 the list only contains one element.
   */
  srv_type: string;

  /**
   * ROS service API URI (where the service is running)
   */
  service_API_URI: string;

  /**
   * The ROS_MASTER_URI the service was originally registered
   */
  masteruri: string;

  /**
   * Describes whether the service is running on the same host as the ROS master. Possible values: local, remote
   */
  location: string;

  /**
   * List of ROS nodes provide this service.
   */
  provider: string[];

  /**
   * List of ROS nodes requested this service.
   */
  requester: string[];

  constructor(
    name: string,
    srv_type: string,
    service_API_URI: string,
    masteruri = "",
    provider: string[] = [],
    location = "unknown",
    requester: string[] = []
  ) {
    this.name = name;
    this.srv_type = srv_type;
    this.service_API_URI = service_API_URI;
    this.masteruri = masteruri;
    this.provider = provider;
    this.location = location;
    this.requester = requester;
  }
}
