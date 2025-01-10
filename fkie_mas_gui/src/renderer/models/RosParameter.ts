import { generateUniqueId } from "../utils";

/**
 * RosParameter models parameters in a ROS system
 */
export default class RosParameter {
  id: string;

  name: string;

  value: string | number | boolean | string[];

  type: string;

  providerId: string;

  constructor(name: string, value: string | number | boolean | string[], type: string, providerId: string) {
    this.id = generateUniqueId();
    this.name = name;
    this.value = value;
    this.type = type;
    this.providerId = providerId;
  }
}
