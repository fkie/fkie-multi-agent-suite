/**
 * LaunchAssociations models assosiation used for depend start or stopn of the nodes.
 */
class LaunchAssociations {
  /**
   * node (full name)
   */
  node: string;

  /**
   * List with nodes (full name) controlled by nodelet manager.
   */
  nodes: string[];

  /**
   * Class Constructor
   *
   * @param {string} node - node (full name)
   * @param {string[]} nodes - list with associated nodes (full name).
   */
  constructor(manager: string, nodes: string[]) {
    this.node = manager;
    this.nodes = nodes;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.node} - ${this.nodes}`;
  };
}

export default LaunchAssociations;
