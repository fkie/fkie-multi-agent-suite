import { generateUniqueId } from '../utils';

/**
 * RosPackage models packages in a ROS system
 */
class RosPackage {
  id: string;

  name: string;

  path: string;

  /**
   * Class Constructor
   *
   * @param {string} name - Package name
   * @param {string} path - Local absolute path of a package
   */
  constructor(name: string, path: string) {
    this.id = generateUniqueId();
    this.name = name;
    this.path = path;
  }

  /**
   * Generates an unique string Identifier
   *
   * @return {string} Returns a unique string identifier
   */
  guidGenerator: () => string = () => {
    // return Math.random()
    //   .toString(36)
    //   .replace(/[^a-z]+/g, '')
    //   .substr(2, 10);
    return Date.now().toString(36) + Math.random().toString(36).substr(2);
  };
}

export default RosPackage;
