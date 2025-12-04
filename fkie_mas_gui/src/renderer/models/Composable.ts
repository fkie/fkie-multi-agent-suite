
/**
 * Composable node.
 */
export default class Composable {
  /**
   * Unique identifier
   */
  nodeId: string;

  /**
   * ROS name (including namespace)
   */
  containerName: string;

  nodes: string[] = [];

  constructor(
    nodeId: string,
    containerName: string,
    nodes: string[],
  ) {
    this.nodeId = nodeId;
    this.containerName = containerName;
    this.nodes = nodes;
  }
}
