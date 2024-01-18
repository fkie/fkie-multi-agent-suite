import LaunchArgument from './LaunchArgument';

/**
 * Request to parse the text for included paths.
 */
class LaunchInterpretPathRequest {
  /**
   * Line in the launch config.
   */
  text: string;

  /**
   * Arguments used to load the launch file.
   */
  args: LaunchArgument[];

  /**
   * Class Constructor
   *
   * @param {string} text - Line in the launch config.
   * @param {LaunchArgument[]} args - Arguments used to load the launch file
   */
  constructor(text: string, args: LaunchArgument[]) {
    this.text = text;
    this.args = args;
  }
}

export default LaunchInterpretPathRequest;
