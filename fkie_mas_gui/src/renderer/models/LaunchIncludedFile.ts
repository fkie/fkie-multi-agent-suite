/* eslint-disable camelcase */
import LaunchArgument from "./LaunchArgument";

/**
 * Representation of an included file found in given string or path of a file.
 */
export default class LaunchIncludedFile {
  /**
   * host of the file.
   */
  host: string;

  /**
   * Current reading file.
   */
  path: string;

  /**
   * Line number of the occurrence. If requested `unique` is True the line number is zero.
   */
  line_number: number;

  /**
   * Resolved path.
   */
  inc_path: string;

  /**
   * True if resolved path exists.
   */
  exists: boolean;

  /**
   * Representation of included file without resolved arg and find statements.
   */
  raw_inc_path: string;

  /**
   * Depth of recursion. If `unique` is True the depth is zero.
   */
  rec_depth: number;

  /**
   * A list with arguments forwarded within include tag for 'inc_path'.
   */
  args: LaunchArgument[];

  /**
   * A list with default arguments defined in 'inc_path'.
   */
  default_inc_args: LaunchArgument[];

  /**
   * Size of the included file in bytes.
   */
  size: number;

  /** if True the included file is not loaded be current configuration. */
  conditional_excluded: boolean;

  constructor(
    host: string,
    path: string,
    line_number: number,
    inc_path: string,
    exists: boolean,
    raw_inc_path: string,
    rec_depth: number,
    args: LaunchArgument[],
    default_inc_args: LaunchArgument[],
    size: number,
    conditional_excluded: boolean
  ) {
    this.host = host;
    this.path = path;
    this.line_number = line_number;
    this.inc_path = inc_path;
    this.exists = exists;
    this.raw_inc_path = raw_inc_path;
    this.rec_depth = rec_depth;
    this.args = args;
    this.default_inc_args = default_inc_args;
    this.size = size;
    this.conditional_excluded = conditional_excluded;
  }
}
