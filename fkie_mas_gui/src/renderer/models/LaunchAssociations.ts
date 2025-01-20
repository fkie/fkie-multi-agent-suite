/**
 * LaunchAssociations models assosiation used for depend start or stopn of the nodes.
 */
export default class LaunchAssociations {
  /**
   * node (full name)
   */
  node: string;

  /**
   * List with nodes (full name) controlled by nodelet manager.
   */
  nodes: string[] | undefined;

  constructor(manager: string, nodes: string[]) {
    this.node = manager;
    this.nodes = nodes;
  }
}
