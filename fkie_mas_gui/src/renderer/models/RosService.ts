/* eslint-disable camelcase */
/**
 * RosService models topics in a ROS system
 */
class RosService {
  /**
   * Topic name including namespace
   */
  name: string;

  /**
   * List of types, in case of ROS1 the list only contains one element.
   */
  srvtype: string;

  /**
   * ROS serice API URI (where the service is running)
   */
  service_API_URI: string;

  /**
   * The ROS_MASTER_URI the service was originaly registered
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
   * Class Constructor
   *
   * @param {string} name - Topic name including namespace
   * @param {string} srvtype - Service type.
   * @param {string} service_API_URI - ROS serice API URI (where the service is running)
   * @param {string} masteruri - The ROS_MASTER_URI the service was originaly registered
   * @param {string[]} provider - List of ROS nodes provide this service.
   * @param {string} location - Describes whether the service is running on the same host as the ROS master. Possible values: local, remote
   */
  constructor(
    name: string,
    srvtype: string,
    service_API_URI: string,
    masteruri = "",
    provider: string[] = [],
    location = "local"
  ) {
    this.name = name;
    this.srvtype = srvtype;
    this.service_API_URI = service_API_URI;
    this.masteruri = masteruri;
    this.provider = provider;
    this.location = location;
  }

  /**
   * Generates a string representation of the node
   *
   * @return {string} Node description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.srvtype}`;
  };
}

export default RosService;
