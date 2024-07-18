/* eslint-disable camelcase */
import LaunchArgument from "./LaunchArgument";

/**
 * Request to parse the given file for included files.
 */
class LaunchIncludedFilesRequest {
  /**
   * File to parse.
   */
  path: string;

  /**
   * True to read recursive.
   */
  recursive: boolean;

  /**
   * True to ignore files included multiple times.
   */
  unique: boolean;

  /**
   * Pattern to change include detection. Empty for default.
   */
  pattern: string[];

  /**
   * Search only for files with given extensions. Empty for default.
   */
  search_in_ext: string[];

  /**
   * Arguments to load the launch file.
   */
  args: LaunchArgument[];

  /**
   * Class Constructor
   *
   * @param {string} path - File to parse.
   * @param {boolean} recursive - True to read recursive.
   * @param {boolean} unique - True to ignore files included multiple times.
   * @param {string[]} pattern - Pattern to change include detection. Empty for default.
   * @param {string[]} search_in_ext - Search only for files with given extensions. Empty for default.
   * @param {LaunchArgument[]} args - Arguments used to load the launch file.
   */
  constructor(
    path = "",
    recursive = true,
    unique = false,
    pattern: string[] = [],
    search_in_ext: string[] = [],
    args: LaunchArgument[] = []
  ) {
    this.path = path;
    this.recursive = recursive;
    this.unique = unique;
    this.pattern = pattern;
    this.search_in_ext = search_in_ext;
    this.args = args;
  }
}

export default LaunchIncludedFilesRequest;
