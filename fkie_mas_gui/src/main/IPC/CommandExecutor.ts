import { CommandExecutorEvents, TCommandExecutor, TSystemInfo } from "@/types";
import { spawn, StdioOptions } from "child_process";
import { ipcMain } from "electron";
import log from "electron-log";
import fs from "fs";
import os from "os";
import path from "path";
import { Client, ClientChannel, ClientErrorExtensions, ConnectConfig } from "ssh2";
import { ARGUMENTS, getArgument } from "../CommandLineInterface";
import PasswordManager from "./PasswordManager";
import { SystemInfo } from "./SystemInfo";

const textDecoder = new TextDecoder();
/**
 * Class CommandExecutor: Execute commands locally or remote using SSH2 interface
 */
class CommandExecutor implements TCommandExecutor {
  localCredential: ConnectConfig;

  // TODO: read ssh config to get username for a given host
  privateSshKeys: Buffer[] = [];

  pm: PasswordManager;

  systemInfo?: TSystemInfo;

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

    const sshPath = `${os.homedir()}/.ssh/`;
    const files = fs.readdirSync(sshPath);
    files.filter((item) => {
      if (item.startsWith("id_")) {
        const content = fs.readFileSync(`${sshPath}${item}`);
        if (content.includes("PRIVATE KEY---")) {
          this.privateSshKeys.push(content);
          return true;
        }
      }
      return false;
    });

    log.info(`found ${this.privateSshKeys.length} ssh keys`);
    const fetchSystemInfo = async (): Promise<void> => {
      this.systemInfo = await new SystemInfo().getInfo();
    };
    fetchSystemInfo();

    // create local credential
    this.localCredential = {
      host: os.hostname(),
      port: 0,
      username: "",
      password: "",
      privateKey: "",
    };
  }

  public registerHandlers: () => void = () => {
    ipcMain.handle(CommandExecutorEvents.exec, (_event, credential: ConnectConfig, command: string) => {
      return this.exec(credential, command);
    });

    ipcMain.handle(
      CommandExecutorEvents.execTerminal,
      (_event, credential: ConnectConfig, title: string, command: string) => {
        return this.execTerminal(credential, title, command);
      }
    );
  };

  /**
   * Executes a command using a SSH connection
   * @param {ConnectConfig} credential - SSH credential, null for local host.
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  public exec: (
    credential: ConnectConfig | null,
    command: string
  ) => Promise<{ result: boolean; message: string; command: string; connectConfig?: ConnectConfig }> = async (
    credential: ConnectConfig | null,
    command: string
  ) => {
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

    if (c.host === undefined || localIps.includes(c.host)) {
      // log.debug(`CommandExecutor exec: Using local process for: [${command}]`);
      // if local command, do not use SSH but child process instead
      return new Promise((resolve) => {
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
                command,
              });
            } else {
              resolve({
                result: true,
                message: "",
                command,
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
              command,
            });
          });
        } catch (error) {
          resolve({
            result: false,
            message: `Catch error ${error}`,
            command,
          });
        }
      });
    }

    // command must be executed remotely

    return this.execRemote(c, command, 0);
  };

  private execRemote: (
    credential: ConnectConfig,
    command: string,
    keyIndex: number
  ) => Promise<{ result: boolean; message: string; command: string; connectConfig?: ConnectConfig }> = async (
    credential,
    command,
    keyIndex = 0
  ) => {
    const parentOut = getArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES) === "true";
    const connectionConfig = await this.generateConfig(credential, keyIndex);
    return new Promise((resolve) => {
      if (!command)
        resolve({
          result: false,
          message: "Invalid empty command",
          command,
          connectConfig: connectionConfig,
        });
      const conn: Client = new Client();
      try {
        conn
          .on("ready", () => {
            conn.exec(command, (err: Error | undefined, sshStream: ClientChannel) => {
              if (credential) log.info(`<ssh:${credential.username}@${credential.host}:${credential.port}>${command}`);
              if (err) {
                resolve({
                  result: false,
                  message: err?.message,
                  command,
                });
              }
              let errorString = "";
              // .on('close', (code: string, signal: string) => {
              sshStream
                .on("close", (code: number) => {
                  // TODO: Check code/signal to validate response or errors
                  // command executed correctly and no response
                  if (code !== 0) {
                    resolve({
                      result: false,
                      message: errorString,
                      command,
                    });
                  } else {
                    resolve({
                      result: true,
                      message: "",
                      command,
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
                    command,
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
                    command,
                  });
                });
            });
          })
          .connect(connectionConfig);
        conn.on("error", async (error: Error & ClientErrorExtensions) => {
          log.warn("CommandExecutor - connect error: ", error);
          connectionConfig.password = undefined;
          connectionConfig.privateKey = undefined;
          if (keyIndex + 1 < this.privateSshKeys.length) {
            const result = await this.execRemote(connectionConfig, command, keyIndex + 1);
            resolve(result);
          } else {
            resolve({
              result: false,
              message: error.message,
              command,
              connectConfig: connectionConfig,
            });
          }
        });
      } catch (error: any) {
        log.info("CommandExecutor - exec error: ", error);
        resolve({
          result: false,
          message: error.message,
          command,
        });
      }
    });
  };

  /**
   * Executes a command in an external Terminal (using a SSH connection on remote hosts)
   * @param {TCredenConnectConfigtial} credential - SSH credential, null for local host
   * @param {string} title - Remote directory path
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  public execTerminal: (
    credential: ConnectConfig | null,
    title: string,
    command: string
  ) => Promise<{ result: boolean; message: string; command: string }> = async (credential, title, command) => {
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
      } catch {
        // continue with next terminal
      }
    }

    if (!terminalEmulator) {
      return new Promise((resolve) => {
        resolve({
          result: false,
          message: `No Terminal found! Please install one of ${this.terminalOptions.terminals}`,
          command,
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
   * @param {ConnectConfig} credential - SSH credential
   * @return {object} Returns a configuration file
   */
  private generateConfig = async (credential: ConnectConfig, keyIndex: number): Promise<ConnectConfig> => {
    // try to get password from password manager
    let pwd: string | null = null;
    try {
      const account = `${credential.username}:${credential.host}`;
      pwd = await this.pm.getPassword("MAS_PASSWORDS", account);
    } catch (error) {
      log.info("CommandExecutor - generateConfig error: ", error);
    }

    let privateKey: Buffer | undefined;
    if (!credential.password && keyIndex < this.privateSshKeys.length) {
      privateKey = this.privateSshKeys[keyIndex];
    }

    const config: ConnectConfig = {
      host: credential.host,
      port: credential.port,
      username: credential.username,
      password: credential.password || pwd || undefined,
      privateKey: privateKey,
    };

    return config;
  };
}

export default CommandExecutor;
