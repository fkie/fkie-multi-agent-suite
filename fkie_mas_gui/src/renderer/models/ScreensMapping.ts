/**
 * ScreensMapping models multiple screen of a node.
 */
class ScreensMapping {
  /**
   * Name of the node.
   */
  name: string;

  /**
   * A list of screens detected for a node.
   */
  screens: string[];

  /**
   * Class Constructor
   *
   * @param {string} name - Name of the node.
   * @param {string[]} screens - A list of screens detected for a node.
   */
  constructor(name: string, screens: string[]) {
    this.name = name;
    this.screens = screens;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.screens}`;
  };
}

export default ScreensMapping;
