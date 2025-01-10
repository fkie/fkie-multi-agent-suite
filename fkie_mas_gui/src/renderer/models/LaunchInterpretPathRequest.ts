import LaunchArgument from "./LaunchArgument";

/**
 * Request to parse the text for included paths.
 */
export default class LaunchInterpretPathRequest {
  /**
   * Line in the launch config.
   */
  text: string;

  /**
   * Arguments used to load the launch file.
   */
  args: LaunchArgument[];

  constructor(text: string, args: LaunchArgument[]) {
    this.text = text;
    this.args = args;
  }
}
