/* eslint-disable camelcase */
import { JSONObject } from "@/types";
/**
 * LaunchCallService models the service to call a ROS service.
 */
class LaunchCallService {
  /**
   * The ROS service name.
   */
  service_name: string;

  /**
   * Type of the request service.
   */
  srv_type: string;

  /**
   * Dictionary structure of the ROS request service as JSON string.
   */
  data: JSONObject | null;

  /**
   * Class Constructor
   *
   * @param {string} service_name - The ROS service name.
   * @param {string} srv_type - Type of the request service.
   * @param {JSONObject | null} data - Dictionary structure of the ROS request service as JSON string.
   */
  constructor(service_name: string, srv_type: string, data: JSONObject | null) {
    this.service_name = service_name;
    this.srv_type = srv_type;
    this.data = data;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.service_name} [${this.srv_type}]`;
  };
}

export default LaunchCallService;
