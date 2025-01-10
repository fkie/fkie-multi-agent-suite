import { generateUniqueId } from "../utils";

/**
 * RosPackage models packages in a ROS system
 */
export default class RosPackage {
  id: string;

  name: string;

  path: string;

  constructor(name: string, path: string) {
    this.id = generateUniqueId();
    this.name = name;
    this.path = path;
  }
}
