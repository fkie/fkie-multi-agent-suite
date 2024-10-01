import { TCredential } from "./TCredential";

export const CommandExecutorEvents = {
  exec: "commandExecutor:exec",
  execTerminal: "commandExecutor:execTerminal",
};

export type TCommandExecutor = {
  /**
   * Executes a command using a SSH connection
   * @param {TCredential} credential - SSH credential, null for local host.
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  exec: (credential: TCredential | null, command: string) => Promise<{ result: boolean; message: string }>;

  /**
   * Executes a command in an external Terminal (using a SSH connection on remote hosts)
   * @param {TCredential} credential - SSH credential, null for local host
   * @param {string} title - Remote directory path
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  execTerminal: (
    credential: TCredential | null,
    title: string,
    command: string
  ) => Promise<{ result: boolean; message: string }>;
};
