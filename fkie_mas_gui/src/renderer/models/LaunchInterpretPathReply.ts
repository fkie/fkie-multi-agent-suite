import LaunchArgument from "./LaunchArgument";
import LaunchReturnStatus from "./LaunchReturnStatus";

/**
 * Response for a request to parse the text for included paths.
 */
export default class LaunchInterpretPathReply {
  /**
   * Requested line or part associated with detected include path.
   */
  text: string;

  /**
   * The status of the parsing. One of the codes of LaunchReturnStatus.
   */
  status: LaunchReturnStatus;

  /**
   * The path of the configuration file containing the text.
   */
  path: string;

  /**
   * True if detected include path exists.
   */
  exists: boolean;

  /**
   * Arguments used to load the launch file.
   */
  args: LaunchArgument[] | undefined;

  constructor(text: string, status: LaunchReturnStatus, path: string, exists: boolean, args: LaunchArgument[]) {
    this.text = text;
    this.status = status;
    this.path = path;
    this.exists = exists;
    this.args = args;
  }
}
