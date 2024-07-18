import log from "electron-log";
import { ARGUMENTS, getArgument, hasArgument } from "../CommandLineInterface";
import { ICredential } from "../models/ICredential";
import CommandExecutor from "./CommandExecutor";
import { ROSInfo } from "./ROSInfo";

/**
 * Class TerminalManager: Allows to create terminal objects to interact with console
 */
class TerminalManager {
  commandExecutor: CommandExecutor;

  rosInfo: ROSInfo;

  constructor() {
    this.commandExecutor = new CommandExecutor();
    this.rosInfo = new ROSInfo();
  }

  /**
   * Creates a new terminal using given credentials and execute bash
   *
   * @param {ICredential} credential - Credentials to connect with
   * @param {number} port - TTYD Port
   * @return {string} Returns result
   */
  public spawnTerminal: (
    rosVersion?: string | null,
    credential?: ICredential | null,
    port?: number
  ) => Promise<{ result: boolean; message: string }> = (rosVersion = null, credential = null, port = undefined) => {
    let version = rosVersion;
    if (!version) {
      version = this.rosInfo.version === "1" ? "1" : "2";
      log.debug(`use ROS version: ${version}`);
    }
    const portNumber = port || getArgument(ARGUMENTS.TTYD_PORT);

    const ttydCmd = `${this.getPathTTY()} --writable --port ${portNumber} bash`;

    let cmd = "";
    if (version === "1") {
      cmd = `rosrun fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else if (version === "2") {
      cmd = `ros2 run fkie_mas_daemon mas-remote-node.py --respawn --name=ttyd-${portNumber} --command=${ttydCmd} --pre_check_binary=true; `;
    } else {
      return Promise.resolve({
        result: false,
        message: "Could not start [ttyd], ROS is not available",
      });
    }

    log.info(`Starting [ttyd-${portNumber}]: [${cmd}]`);
    return this.commandExecutor.exec(credential, cmd);
  };

  /**
   * Returns the path of the suitable executable for the current platform
   *
   * @return {string} Path to the executable
   */
  private getPathTTY: () => string = () => {
    if (hasArgument(ARGUMENTS.TTYD_PATH)) {
      const path = getArgument(ARGUMENTS.TTYD_PATH);
      if (path) return path;
    }

    // TODO Add executables and paths for windows and MAC
    return "ttyd";
  };
}

export default TerminalManager;
