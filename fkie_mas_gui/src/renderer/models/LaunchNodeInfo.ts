/* eslint-disable camelcase */

import { TFileRange, TLaunchArg } from "@/types";
import RosParameter from "./RosParameter";

/**
 * LaunchNodeInfo models info for ROS nodes that comes from launch file.
 */
export default class LaunchNodeInfo {
  unique_name: string | null;

  node_name: string | null;

  name_configured: string | null;

  node_namespace: string | null;

  package_name: string | null;

  executable: string | null;

  respawn: boolean | null;

  respawn_delay: number | null;

  args: string | null;

  remap_args: unknown[] | null;

  parameters: RosParameter[] | null;

  env: unknown[] | null;

  additional_env: unknown[] | null;

  launch_prefix: string | null;

  output: string | null;

  output_format: string | null;

  cmd: string | null;

  cwd: string | null;

  sigterm_timeout: number | null;

  sigkill_timeout: number | null;

  on_exit: unknown[] | null;

  required: boolean | null;

  file_name: string | null;

  file_range: TFileRange | null;

  launch_context_arg: TLaunchArg[] | null;

  launch_name: string | null;

  composable_container: string | null;

  associations: string[] = [];

  constructor(
    unique_name: string | null,
    node_name: string | null = null,
    name_configured: string | null,
    node_namespace: string | null = null,
    package_name: string | null = null,
    executable: string | null = null,
    respawn: boolean | null = null,
    respawn_delay: number | null = null,
    args: string | null = null,
    remap_args: unknown[] | null = null,
    parameters: RosParameter[] | null = null,
    env: unknown[] | null = null,
    additional_env: unknown[] | null = null,
    launch_prefix: string | null = null,
    output: string | null = null,
    output_format: string | null,
    cmd: string | null,
    cwd: string | null,
    sigterm_timeout: number | null,
    sigkill_timeout: number | null,
    on_exit: unknown[] | null,
    required: boolean | null = null,
    file_name: string | null = null,
    file_range: TFileRange | null = null,
    launch_context_arg: TLaunchArg[] | null = null,
    launch_name: string | null = null,
    composable_container: string | null
  ) {
    this.unique_name = unique_name;
    this.node_name = node_name;
    this.name_configured = name_configured;
    this.node_namespace = node_namespace;
    this.package_name = package_name;
    this.executable = executable;
    this.respawn = respawn;
    this.respawn_delay = respawn_delay;
    this.args = args;
    this.remap_args = remap_args;
    this.parameters = parameters;
    this.env = env;
    this.additional_env = additional_env;
    this.launch_prefix = launch_prefix;
    this.output = output;
    this.output_format = output_format;
    this.cmd = cmd;
    this.cwd = cwd;
    this.sigterm_timeout = sigterm_timeout;
    this.sigkill_timeout = sigkill_timeout;
    this.on_exit = on_exit;
    this.required = required;
    this.file_name = file_name;
    this.file_range = file_range;
    this.launch_context_arg = launch_context_arg;
    this.launch_name = launch_name;
    this.composable_container = composable_container;
  }
}
