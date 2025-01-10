/* eslint-disable camelcase */
import LaunchArgument from "./LaunchArgument";

/**
 * LaunchLoadRequest models load request of the launch file.
 */
export default class LaunchLoadRequest {
  /**
   * ROS package name
   */
  ros_package: string;

  /**
   * Launch file in the package path. If the package contains
   * more then one launch file with same name,
   * you have to specify the sub-path with launch file.
   */
  launch: string;

  /**
   * if set, this will be used instead of package/launch
   */
  path: string;

  /**
   * Arguments to load the launch file. If args are empty but the launch file needs them,
   * the reply status has code PARAMS_REQUIRED and args list will be filled with requested args.
   */
  args: LaunchArgument[];

  /**
   * If True, use first file if more than one was found in the package.
   */
  force_first_file: boolean;

  /**
   * If True, the launch file will not be loaded, only launch arguments are requested.
   */
  request_args: boolean;

  /**
   * Starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
   * the nodes are started on the host specified by hostname of the masteruri.
   */
  masteruri: string;

  /**
   * Start nodes of this file on specified host.
   */
  host: string;

  constructor(
    ros_package: string,
    launch: string,
    path: string,
    args: LaunchArgument[],
    force_first_file: boolean,
    request_args: boolean,
    masteruri: string,
    host: string
  ) {
    this.ros_package = ros_package;
    this.launch = launch;
    this.path = path;
    this.args = args;
    this.force_first_file = force_first_file;
    this.request_args = request_args;
    this.masteruri = masteruri;
    this.host = host;
  }
}
