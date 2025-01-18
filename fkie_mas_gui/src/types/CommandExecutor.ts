import { ConnectConfig } from "ssh2";

export const CommandExecutorEvents = {
  exec: "commandExecutor:exec",
  execTerminal: "commandExecutor:execTerminal",
};

export type TCommandExecutor = {
  /**
   * Executes a command using a SSH connection
   * @param credential - SSH credential, null for local host.
   * @param command - Remote directory path
   * @return Returns response
   */
  exec: (
    credential: ConnectConfig | null,
    command: string
  ) => Promise<{ result: boolean; message: string; command: string; connectConfig?: ConnectConfig }>;

  /**
   * Executes a command in an external Terminal (using a SSH connection on remote hosts)
   * @param credential - SSH credential, null for local host
   * @param title - Remote directory path
   * @param command - Remote directory path
   * @return Returns response
   */
  execTerminal: (
    credential: ConnectConfig | null,
    title: string,
    command: string
  ) => Promise<{ result: boolean; message: string; command: string }>;
};
