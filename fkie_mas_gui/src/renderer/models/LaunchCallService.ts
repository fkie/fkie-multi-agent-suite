/* eslint-disable camelcase */
import { TRosMessageStruct } from "./TRosMessageStruct";
/**
 * LaunchCallService models the service to call a ROS service.
 */
export default class LaunchCallService {
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
  data: string;

  constructor(service_name: string, srv_type: string, data: TRosMessageStruct | TRosMessageStruct[] | undefined) {
    this.service_name = service_name;
    this.srv_type = srv_type;
    this.data = JSON.stringify(data);
  }
}
