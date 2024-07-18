import { generateUniqueId } from "../utils";

/**
 * RosParameter models parameters in a ROS system
 */
class RosParameter {
  id: string;

  name: string;

  value: string | number | boolean | string[];

  type: string;

  providerId: string;

  /**
   * Class Constructor
   *
   * @param {string} name - Parameter name
   * @param {string | number | boolean | string[]} value - Parameter value
   */
  constructor(name: string, value: string | number | boolean | string[], type: string, providerId: string) {
    this.id = generateUniqueId();
    this.name = name;
    this.value = value;
    this.type = type;
    this.providerId = providerId;
  }
}

export default RosParameter;
