import { spawn, StdioOptions } from "child_process";
import log from "electron-log";
import fs from "fs";
import os from "os";
import path from "path";
import { Client, ConnectConfig } from "ssh2";
import { ARGUMENTS, getArgument } from "../CommandLineInterface";
import { ICredential } from "../models/ICredential";
import PasswordManager from "./PasswordManager";
import { ISystemInfo, SystemInfo } from "./SystemInfo";

const textDecoder = new TextDecoder();
/**
 * Class CommandExecutor: Execute commands locally or remote using SSH2 interface
 */
class CommandExecutor {
  localCredential: ICredential;

  pm: PasswordManager;

  systemInfo?: ISystemInfo;

  terminalOptions: {
    terminals: string[];
    exec: string;
    noClose: string;
    title: string;
  } = {
    terminals: ["/usr/bin/x-terminal-emulator", "/usr/bin/xterm", "/opt/x11/bin/xterm"],
    exec: "e",
    noClose: "",
    title: "-T",
  };

  constructor() {
    this.pm = new PasswordManager();

    const fetchSystemInfo = async (): Promise<void> => {
      this.systemInfo = await new SystemInfo().getInfo();
    };
    fetchSystemInfo();

    // create local credential
    this.localCredential = {
      id: "",
      host: os.hostname(),
      port: 0,
      username: "",
      password: "",
      service: "",
      account: "",
    };
  }

  /**
   * Executes a command using a SSH connection
   * @param {ICredential} credential - SSH credential, null for local host.
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  public exec: (credential: ICredential | null, command: string) => Promise<{ result: boolean; message: string }> =
    async (credential: ICredential | null, command: string) => {
      let c = credential;

      // if no credential is given, assumes local host
      if (!c) c = this.localCredential;

      // Set the STDIO config: Ignore or redirect STDOUT/STDERR to current console
      let stdioOptions: StdioOptions | undefined = ["ignore", "pipe", "pipe"];
      const parentOut = getArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES) === "true";
      if (parentOut) {
        stdioOptions = ["inherit", "pipe", "pipe"];
      }

      const localIps = ["localhost", "127.0.0.1", os.hostname()];

      if (this.systemInfo) {
        this.systemInfo.networkInterfaces?.forEach((ni) => {
          localIps.push(ni.ip4);
        });
      }

      if (localIps.includes(c.host)) {
        // log.debug(`CommandExecutor exec: Using local process for: [${command}]`);

        // if local command, do not use SSH but child process instead
        return new Promise((resolve, _reject) => {
          try {
            let errorString = "";
            log.info(`<cmd>${command}`);
            const child = spawn(command, [], {
              shell: true,
              stdio: stdioOptions,
              detached: false,
            });

            child.on("close", (code) => {
              if (code !== 0) {
                resolve({
                  result: false,
                  message: errorString,
                });
              } else {
                resolve({
                  result: true,
                  message: "",
                });
                // resolve(`Closed with code: ${code}`);
              }
            });
            child.stdout?.on("data", function (data) {
              if (parentOut) {
                console.log(`${data}`);
                `${data}`.split("\n").forEach((item) => {
                  if (
                    item.includes("[rosrun] Couldn't find executable") ||
                    item.includes("[ERROR]") ||
                    item.includes("[error]")
                  ) {
                    errorString += item;
                  }
                });
              }
            });
            child.stderr?.on("data", function (data) {
              if (parentOut) {
                console.error(`${data}`);
              }
              errorString += data;
            });

            child.on("error", (error) => {
              resolve({
                result: false,
                message: error.message,
              });
            });
          } catch (error) {
            resolve({
              result: false,
              message: `Catch error ${error}`,
            });
          }
        });
      }

      // command must be executed remotely
      const connectionConfig = await this.generateConfig(c);

      return new Promise((resolve, _reject) => {
        if (!command)
          resolve({
            result: false,
            message: "Invalid empty command",
          });
        const conn = new Client();
        try {
          conn
            .on("ready", () => {
              conn.exec(command, (err: Error | undefined, sshStream: any) => {
                if (c) log.info(`<ssh:${c.username}@${c.host}:${c.port}>${command}`);
                if (err) {
                  resolve({
                    result: false,
                    message: err?.message,
                  });
                }
                let errorString = "";
                // .on('close', (code: string, signal: string) => {
                sshStream
                  .on("close", (code: number, _signal: string) => {
                    // TODO: Check code/signal to validate response or errors
                    // command executed correctly and no response
                    if (code !== 0) {
                      resolve({
                        result: false,
                        message: errorString,
                      });
                    } else {
                      resolve({
                        result: true,
                        message: "",
                      });
                    }
                    conn.end();
                  })
                  .stdout.on("data", (data: Buffer) => {
                    if (parentOut) {
                      console.log(`${textDecoder.decode(data)}`);
                    }
                    resolve({
                      result: true,
                      message: textDecoder.decode(data),
                    });
                  })
                  .stderr.on("data", (data: Buffer) => {
                    if (parentOut) {
                      console.error(`${textDecoder.decode(data)}`);
                    }
                    errorString += textDecoder.decode(data);
                    resolve({
                      result: false,
                      message: textDecoder.decode(data),
                    });
                  });
              });
            })
            .connect(connectionConfig);
          conn.on("error", (error: any) => {
            log.warn("CommandExecutor - connect error: ", error);
            resolve({
              result: false,
              message: error.message,
            });
          });
        } catch (error: any) {
          log.info("CommandExecutor - exec error: ", error);
          resolve({
            result: false,
            message: error.message,
          });
        }
      });
    };

  /**
   * Executes a command in an external Terminal (using a SSH connection on remote hosts)
   * @param {ICredential} credential - SSH credential, null for local host
   * @param {string} title - Remote directory path
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  public execTerminal: (
    credential: ICredential | null,
    title: string,
    command: string
  ) => Promise<{ result: boolean; message: string }> = async (credential, title, command) => {
    let terminalEmulator = "";
    let terminalTitleOpt = this.terminalOptions.title;
    let noCloseOpt = this.terminalOptions.noClose;
    let terminalExecOpt = this.terminalOptions.exec;
    // eslint-disable-next-line no-restricted-syntax
    for (const t of this.terminalOptions.terminals) {
      // eslint-disable-next-line no-loop-func
      try {
        fs.accessSync(t, fs.constants.X_OK);
        // workaround to support the command parameter in different terminal
        const resolvedPath = fs.realpathSync(t, null);
        const basename = path.basename(resolvedPath);
        if (["terminator", "gnome-terminal", "xfce4-terminal"].includes(basename)) {
          terminalExecOpt = "-x";
        } else {
          terminalExecOpt = "-e";
        }
        if (["terminator", "gnome-terminal", "gnome-terminal.wrapper"].includes(basename)) {
          // If your external terminal close after the execution, you can change this behavior in profiles.
          // You can also create a profile with name 'hold'. This profile will be then load by node_manager.
          noCloseOpt = "--profile hold";
        } else if (["xfce4-terminal", "xterm", "lxterm", "uxterm"].includes(basename)) {
          noCloseOpt = "";
          terminalTitleOpt = "-T";
        }
        terminalEmulator = t;
        break;
      } catch (error) {
        // continue with next terminal
      }
    }

    if (!terminalEmulator) {
      return new Promise((resolve) => {
        resolve({
          result: false,
          message: `No Terminal found! Please install one of ${this.terminalOptions.terminals}`,
        });
      });
    }

    let terminalTitle = "";
    if (title && terminalTitleOpt) {
      terminalTitle = `${terminalTitleOpt} ${title}`;
    }
    let sshCmd = "";
    if (credential) {
      // generate string for SSH command
      sshCmd = [
        "/usr/bin/ssh",
        "-aqtxXC",
        "-oClearAllForwardings=yes",
        "-oConnectTimeout=5",
        "-oStrictHostKeyChecking=no",
        "-oVerifyHostKeyDNS=no",
        "-oCheckHostIP=no",
        [credential.username, credential.host].join("@"),
      ].join(" ");
    }
    const cmd = `${terminalEmulator} ${terminalTitle} ${noCloseOpt} ${terminalExecOpt} ${sshCmd} ${command}`;
    return this.exec(null, cmd);
  };

  /**
   * Generate configuration file for SSH connection
   * @param {ICredential} credential - SSH credential
   * @return {object} Returns a configuration file
   */
  private generateConfig = async (credential: ICredential): Promise<ConnectConfig> => {
    // try to get password from password manager
    let pwd: string | null = null;
    try {
      pwd = await this.pm.getPassword(credential.service, credential.account);
    } catch (error) {
      log.info("CommandExecutor - generateConfig error: ", error);
    }

    // TODO: Check for inexistent passwords
    const config: ConnectConfig = {
      host: credential.host,
      port: credential.port,
      username: credential.username,
      password: pwd || undefined,
    };

    return config;
  };
}

export default CommandExecutor;
