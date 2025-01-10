/**
 * ScreensMapping models multiple screen of a node.
 */
export default class ScreensMapping {
  /**
   * Name of the node.
   */
  name: string;

  /**
   * A list of screens detected for a node.
   */
  screens: string[];

  constructor(name: string, screens: string[]) {
    this.name = name;
    this.screens = screens;
  }
}
