import LaunchArgument from "./LaunchArgument";
import LaunchAssociations from "./LaunchAssociations";
import LaunchNodeInfo from "./LaunchNodeInfo";
import RosParameter from "./RosParameter";

/**
 * LaunchContent models the launch file with all content with nodes or associations.
 */
export default class LaunchContent {
  /**
   * Full path of the launch file with contains the reported nodes.
   */
  path: string;

  /**
   * Arguments used to load the launch file.
   */
  args: LaunchArgument[] | undefined;

  /**
   * Starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
   * the nodes are started on the host specified by hostname of the masteruri.
   */
  masteruri: string | undefined;

  /**
   * Start nodes of this file on specified host.
   */
  host: string;

  /**
   * List of node names configured by this launch file.
   */
  nodes: LaunchNodeInfo[] | undefined;

  /**
   * List of parameters configured by this launch file.
   */
  parameters: RosParameter[] | undefined;

  /**
   * List of associations configured by this launch file.
   */
  associations: LaunchAssociations[] | undefined;

  warnings: string[];

  constructor(
    path: string,
    args: LaunchArgument[],
    masteruri: string,
    host: string,
    nodes: LaunchNodeInfo[],
    parameters: RosParameter[],
    associations: LaunchAssociations[],
    warnings: string[]
  ) {
    this.path = path;
    this.args = args;
    this.masteruri = masteruri;
    this.host = host;
    this.nodes = nodes;
    this.parameters = parameters;
    this.associations = associations;
    this.warnings = warnings;
  }
}
