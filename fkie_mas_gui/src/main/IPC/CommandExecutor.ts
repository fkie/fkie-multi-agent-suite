import { CommandExecutorEvents, TCommandExecutor, TSystemInfo } from "@/types";
import { ipcMain } from "electron";
import log from "electron-log";
import { spawn, spawnSync, StdioOptions } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { Client, ClientChannel, ClientErrorExtensions, ConnectConfig } from "ssh2";
import CommandLine from "./CommandLine";
import { SystemInfo } from "./SystemInfo";

type TTerminalSpec = {
  bin: string;
  /** args before the command, e.g. title/hold */
  preArgs: (title: string, hold: boolean) => string[];
  /** separator + command args; MUST be last */
  execArgs: (script: string) => string[];
  /** true for D-Bus based terminals: exit code says nothing about success */
  detached: boolean;
};

const textDecoder = new TextDecoder();

function terminalSpec(bin: string, hasHoldProfile: boolean): TTerminalSpec {
  let real = bin;
  try {
    // may throw on a dangling /etc/alternatives symlink
    real = fs.realpathSync(bin);
  } catch {
    log.warn(`cannot resolve symlink of ${bin}, using path as is`);
  }
  const base = path.basename(real);

  // "-ic" instead of "-lc": ~/.bashrc guards like [ -z "$PS1" ] && return
  // would skip the ROS setup in a non-interactive shell.
  const bash = (s: string): string[] => ["/bin/bash", "-ic", s];

  switch (base) {
    case "gnome-terminal":
      return {
        bin,
        preArgs: (t, hold) => [
          ...(t ? [`--title=${t}`] : []),
          ...(hold && hasHoldProfile ? ["--profile", "hold"] : []),
        ],
        execArgs: (s) => ["--", ...bash(s)],
        detached: true,
      };

    case "gnome-terminal.wrapper":
      // the Debian wrapper only understands the legacy syntax: -t TITLE -e "STRING"
      return {
        bin,
        preArgs: (t) => (t ? ["-t", t] : []),
        execArgs: (s) => ["-e", `/bin/bash -ic ${shq(s)}`],
        detached: true,
      };

    case "ptyxis":
    case "kgx":
      // neither supports a title option -> omit it
      return { bin, preArgs: () => [], execArgs: (s) => ["--", ...bash(s)], detached: true };

    case "tilix":
      // -e takes ONE string -> quote the whole command line
      return {
        bin,
        preArgs: (t) => (t ? ["-t", t] : []),
        execArgs: (s) => ["-e", `/bin/bash -ic ${shq(s)}`],
        detached: true,
      };

    case "terminator":
      return {
        bin,
        // "-u" (--no-dbus) prevents the handoff to a running instance
        preArgs: (t) => [...(t ? ["-T", t] : []), "-u"],
        // "-x" is argparse REMAINDER -> must be last
        execArgs: (s) => ["-x", ...bash(s)],
        detached: false,
      };

    case "xfce4-terminal":
      return {
        bin,
        // --disable-server: do not hand the window over to a running instance
        preArgs: (t) => [...(t ? ["-T", t] : []), "--disable-server"],
        execArgs: (s) => ["-x", ...bash(s)],
        detached: false,
      };

    case "konsole":
      return {
        bin,
        // --nofork keeps the exit code meaningful (konsole forks by default)
        preArgs: (t) => [...(t ? ["-p", `tabtitle=${t}`] : []), "--nofork"],
        execArgs: (s) => ["-e", ...bash(s)],
        detached: false,
      };

    case "kitty":
      return { bin, preArgs: (t) => (t ? ["--title", t] : []), execArgs: (s) => bash(s), detached: false };

    case "alacritty":
      // "-e" consumes all remaining args -> must be last
      return { bin, preArgs: (t) => (t ? ["-t", t] : []), execArgs: (s) => ["-e", ...bash(s)], detached: false };

    case "foot":
      // foot takes the program as positional argument, "-e" is not portable
      return { bin, preArgs: (t) => (t ? ["-T", t] : []), execArgs: (s) => bash(s), detached: false };

    case "wezterm":
      // wezterm needs the "start" subcommand and has no --title
      return { bin, preArgs: () => ["start"], execArgs: (s) => ["--", ...bash(s)], detached: false };

    case "lxterminal":
    case "qterminal":
      return {
        bin,
        preArgs: (t) => (t ? ["-T", t] : []),
        execArgs: (s) => ["-e", `/bin/bash -ic ${shq(s)}`],
        detached: true,
      };

    case "mate-terminal":
      return { bin, preArgs: (t) => (t ? [`--title=${t}`] : []), execArgs: (s) => ["--", ...bash(s)], detached: true };

    default: // xterm, uxterm, lxterm, urxvt, st, ...
      return { bin, preArgs: (t) => (t ? ["-T", t] : []), execArgs: (s) => ["-e", ...bash(s)], detached: false };
  }
}

/**
 * Class CommandExecutor: Execute commands locally or remote using SSH2 interface
 */
export default class CommandExecutor implements TCommandExecutor {
  commandLine: CommandLine | null = null;
  localCredential: ConnectConfig;

  // TODO: read ssh config to get username for a given host
  sshUsers: { [id: string]: string } = {}; // host: user
  sshPorts: { [id: string]: number } = {}; // host: port
  sshKeys: { [id: string]: Buffer } = {}; // host: privateKeys

  privateSshKeys: Buffer[] = [];

  systemInfo?: TSystemInfo;

  /** Cached path of the detected terminal emulator ("" = none found yet). */
  private detectedTerminal: string | null = null;

  /**
   * Terminal configuration and detection options.
   *
   * terminals   -> candidate terminal binaries, in priority order
   * exec        -> option used to execute a command (e.g. "-x /bin/bash -c" or "-e /bin/bash -c")
   * noClose     -> option used to keep terminal open after command (depends on terminal)
   * title       -> option used to set terminal title (depends on terminal)
   */
  terminalOptions: {
    terminals: string[];
    exec: string;
    noClose: string;
    title: string;
  } = {
      terminals: ["/usr/bin/x-terminal-emulator", "/usr/bin/xterm", "/opt/x11/bin/xterm"],
      exec: "-e /bin/bash -c",
      noClose: "",
      title: "",
    };

  constructor(commandLine: CommandLine) {
    this.commandLine = commandLine;
    const sshPath = `${os.homedir()}/.ssh`;
    let currentHost: string | null = null;

    const readSshLine = async (line: string): Promise<void> => {
      const nLine = line.trim();
      if (nLine.startsWith("Host ")) {
        currentHost = nLine.split(" ")[1];
      } else if ((nLine.startsWith("User ") || nLine.startsWith("user ")) && currentHost) {
        const username = nLine.split(" ")[1];
        this.sshUsers[currentHost] = username;
      } else if ((nLine.startsWith("Port ") || nLine.startsWith("port ")) && currentHost) {
        const port = nLine.split(" ")[1];
        this.sshPorts[currentHost] = Number.parseInt(port);
      } else if (nLine.startsWith("IdentityFile ") && currentHost) {
        const identPath: string = nLine.split(" ")[1].replace("~", os.homedir());
        try {
          this.sshKeys[currentHost] = fs.readFileSync(identPath);
        } catch (error) {
          console.error(`error while read specified IdentityFile "${identPath}": ${error}`);
        }
      }
    };

    const readSshConfig = async (): Promise<void> => {
      try {
        // read host/user configuration from ssh config
        fs.readFile(`${sshPath}/config`, "utf8", async (err, data) => {
          if (err) {
            log.warn(`error while read ${sshPath}/config`);
            return;
          }
          const configLines = data.split("\n");

          for (const line of configLines) {
            await readSshLine(line);
          }
          log.info(`found ${Object.keys(this.sshUsers).length} host configurations`);
        });
      } catch (error) {
        console.error(`error while read ssh configuration file "${sshPath}/config": ${error}`);
      }
    };

    readSshConfig();

    try {
      // read private ssh keys
      const files = fs.readdirSync(sshPath);
      files.filter((item) => {
        if (item.startsWith("id_")) {
          const content = fs.readFileSync(`${sshPath}/${item}`);
          if (content.includes("PRIVATE KEY---")) {
            this.privateSshKeys.push(content);
            return true;
          }
        }
        return false;
      });
    } catch (error) {
      console.error(`error while search .ssh directory for private keys: ${error}`);
    }

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
   * Executes a command using a SSH connection or locally via child_process.
   * @param credential - SSH credential, null for local host.
   * @param command - Command to execute
   * @return Returns response
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
      const parentOut = !this.commandLine?.getArg("hide-output-from-background-processes");
      if (parentOut) {
        stdioOptions = ["inherit", "pipe", "pipe"];
      }

      const localIps = ["localhost", "127.0.0.1", os.hostname()];

      if (this.systemInfo) {
        for (const ni of this.systemInfo.networkInterfaces || []) {
          localIps.push(ni.ip4);
        }
      }

      if (c.host === undefined || localIps.includes(c.host)) {
        // local command: do not use SSH but child process instead
        return new Promise((resolve) => {
          try {
            let errorString = "";
            let resultString = "";
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
                  message: resultString,
                  command,
                });
              }
            });

            child.stdout?.on("data", (data) => {
              if (parentOut) {
                console.log(`${data}`);
                resultString += `${data}`;
                for (const item of `${data}`.split("\n")) {
                  if (
                    item.includes("[rosrun] Couldn't find executable") ||
                    item.includes("[ERROR]") ||
                    item.includes("[error]")
                  ) {
                    errorString += item;
                  }
                }
              }
            });

            child.stderr?.on("data", (data) => {
              if (parentOut) {
                console.error(`${data}`);
              }
              errorString += data;
            });

            child.on("error", (error) => {
              if (parentOut) {
                console.error(`${error}`);
              }
              errorString += error;
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

  /**
   * Executes a command on a remote host via SSH.
   * Tries multiple private keys if authentication fails.
   */
  private execRemote: (
    credential: ConnectConfig,
    command: string,
    keyIndex: number
  ) => Promise<{ result: boolean; message: string; command: string; connectConfig?: ConnectConfig }> = async (
    credential,
    command,
    keyIndex = 0
  ) => {
      console.log(`exec on ${credential.host}: ${command}`);
      const parentOut = !this.commandLine?.getArg("hide-output-from-background-processes");
      const connectionConfig = this.generateConfig(credential, keyIndex);

      return new Promise((resolve) => {
        if (!command) {
          resolve({
            result: false,
            message: "Invalid empty command",
            command,
            connectConfig: connectionConfig,
          });
          return;
        }

        const conn: Client = new Client();
        try {
          conn
            .on("ready", () => {
              conn.exec(command, (err: Error | undefined, sshStream: ClientChannel) => {
                if (credential) {
                  log.info(`<ssh:${credential.username}@${credential.host}:${credential.port}>${command}`);
                }
                if (err) {
                  resolve({
                    result: false,
                    message: err?.message,
                    command,
                  });
                  return;
                }
                let errorString = "";

                sshStream
                  .on("close", (code: number) => {
                    // TODO: Check code/signal to validate response or errors
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
            log.warn("CommandExecutor - connect error: ", JSON.stringify(error));
            connectionConfig.password = undefined;
            connectionConfig.privateKey = undefined;
            if (error.level === "client-authentication") {
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
            } else {
              resolve({
                result: false,
                message: error.message,
                command,
              });
            }
          });
        } catch (error) {
          let errorMessage = "Failed to execute remote command";
          if (error instanceof Error) {
            errorMessage = error.message;
          }
          log.info("CommandExecutor - exec error: ", error);
          resolve({
            result: false,
            message: errorMessage,
            command,
          });
        }
      });
    };

  /**
   * Executes a command in an external Terminal (using a SSH connection on remote hosts)
   * @param credential - SSH credential, null for local host
   * @param title - Terminal title
   * @param command - Command to execute (will be passed to /bin/sh -c)
   */

  public async execTerminal(
    credential: ConnectConfig | null,
    title: string,
    command: string
  ): Promise<{ result: boolean; message: string; command: string }> {
    if (!command?.trim()) {
      return { result: false, message: "Refusing to open a terminal with an empty command", command };
    }
    const bin = this.findTerminal();
    if (!bin) return { result: false, message: "No terminal emulator found", command };

    const spec = terminalSpec(bin, this.hasGnomeHoldProfile());
    let script = command;
    if (credential) {
      const c = this.generateConfig(credential, 0);
      script = `/usr/bin/ssh -t -oStrictHostKeyChecking=no -oConnectTimeout=30 ${c.username}@${c.host} ${shq(command)}`;
    }

    const args = [...spec.preArgs(title.replaceAll('"', ""), true), ...spec.execArgs(script)];
    const cmdLine = cmdLineForLog(bin, args);
    log.info(`<terminal> ${cmdLine}`);

    return new Promise((resolve) => {
      // no shell: true -> args are passed verbatim, no quoting/whitespace issues
      const child = spawn(bin, args, { shell: false, detached: true, stdio: ["ignore", "pipe", "pipe"] });
      let out = "";
      child.stdout?.on("data", (d) => {
        out += `${d}`;
      });
      child.stderr?.on("data", (d) => {
        out += `${d}`;
      });

      let settled = false;
      const finish = (result: boolean, message: string): void => {
        if (settled) return;
        settled = true;
        if (!result) log.error(`<terminal> failed: ${message} | ${cmdLine}`);
        resolve({ result, message, command: cmdLine });
      };

      // assume success only if the process is still alive after the grace period
      const timer = setTimeout(() => finish(true, ""), 1500);

      child.on("error", (e) => {
        clearTimeout(timer);
        finish(false, `${e}`);
      });
      child.on("exit", (code, signal) => {
        // report the real reason instead of silently assuming success
        if (code === 0 && spec.detached) return; // window lives in another process
        clearTimeout(timer);
        if (code === 0) {
          finish(true, "");
          return;
        }
        finish(false, out.trim() || `terminal exited with code ${code}${signal ? ` (${signal})` : ""}`);
      });
      child.unref();
    });
  }

  /**
   * Find the first executable terminal emulator from the configured candidate list.
   * The result is cached; pass force=true to re-run the detection.
   *
   * @returns absolute path of the terminal binary or "" if none is available
   */
  public findTerminal(force: boolean = false): string {
    if (!force && this.detectedTerminal !== null) {
      return this.detectedTerminal;
    }

    const candidates = [
      ...this.terminalOptions.terminals,
      // additional fallbacks for systems without x-terminal-emulator alternatives
      "/usr/bin/gnome-terminal",
      "/usr/bin/konsole",
      "/usr/bin/xfce4-terminal",
      "/usr/bin/terminator",
      "/usr/bin/kitty",
      "/usr/bin/alacritty",
      "/usr/bin/foot",
      "/usr/bin/ptyxis",
      "/usr/bin/tilix",
    ];

    for (const t of candidates) {
      try {
        fs.accessSync(t, fs.constants.X_OK);
        this.detectedTerminal = t;
        log.info(`terminal emulator detected: ${t} -> ${fs.realpathSync(t)}`);
        return t;
      } catch {
        // not available, try next candidate
      }
    }

    log.warn(`no terminal emulator found, tried: ${candidates.join(", ")}`);
    this.detectedTerminal = "";
    return "";
  }

  private wildcardMatch(text: string | undefined, pattern: string) {
    if (text === undefined) {
      return undefined;
    }
    const regexPattern = new RegExp(`^${pattern.replace(/\?/g, ".").replace(/\*/g, ".*")}$`);
    return regexPattern.test(text);
  }

  /**
   * Generate configuration file for SSH connection
   * @param credential - SSH credential
   */
  private generateConfig(credential: ConnectConfig, keyIndex: number): ConnectConfig {
    let privateKey: Buffer | undefined;

    const matchedHosts = Object.keys(this.sshUsers).find((pattern) => {
      const matched = this.wildcardMatch(credential.host, pattern);
      return matched;
    });

    const sshUser: string | undefined = matchedHosts ? this.sshUsers[matchedHosts] : undefined;
    const sshPort: number | undefined = matchedHosts ? this.sshPorts[matchedHosts] : undefined;
    const sshKey: Buffer | undefined = matchedHosts ? this.sshKeys[matchedHosts] : undefined;

    if (!sshKey && !credential.password && keyIndex < this.privateSshKeys.length) {
      // no key in configuration and no password, try find key
      privateKey = this.privateSshKeys[keyIndex];
    }

    const config: ConnectConfig = {
      host: credential.host,
      port: sshPort || credential.port,
      username: sshUser || credential.username || os.userInfo().username,
      password: credential.password || undefined,
      privateKey: sshKey || privateKey,
    };

    return config;
  }

  /** Check whether a gnome-terminal profile named "hold" exists. */
  private hasGnomeHoldProfile(): boolean {
    try {
      const out = spawnSync("gsettings", ["get", "org.gnome.Terminal.ProfilesList", "list"], { encoding: "utf8" });
      if (out.status !== 0) return false;
      const ids = (out.stdout.match(/'([^']+)'/g) || []).map((s) => s.replaceAll("'", ""));
      return ids.some((id) => {
        const name = spawnSync(
          "gsettings",
          ["get", `org.gnome.Terminal.Legacy.Profile:/org/gnome/terminal/legacy/profiles:/:${id}/`, "visible-name"],
          { encoding: "utf8" }
        );
        return name.status === 0 && name.stdout.trim().replaceAll("'", "") === "hold";
      });
    } catch {
      return false;
    }
  }
}

/** Quote a single argument for POSIX shells (only for logging/copy&paste). */
function shq(arg: string): string {
  return /^[A-Za-z0-9_@%+=:,./-]+$/.test(arg) ? arg : `'${arg.replaceAll("'", `'\\''`)}'`;
}

/** Build a copy-pasteable command line for logs. */
export function cmdLineForLog(bin: string, args: string[]): string {
  return [bin, ...args].map(shq).join(" ");
}

/**
 * Downloads and runs the MAS debian install script.
 */
export async function updateDebianPackages(prerelease: boolean = false): Promise<boolean> {
  return new Promise((resolve) => {
    try {
      const stdioOptions: StdioOptions | undefined = ["inherit", "pipe", "pipe"];
      const child = spawn(
        "/usr/bin/wget",
        [
          `https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/${prerelease ? "devel" : "master"}/install_mas_debs.sh`,
          "-O",
          "/tmp/install_mas_debs.sh",
          "&&",
          "bash",
          "/tmp/install_mas_debs.sh",
        ],
        {
          shell: true,
          stdio: stdioOptions,
          detached: false,
        }
      );

      child.on("close", (code) => {
        resolve(code === 0);
      });

      child.stdout?.on("data", (data) => {
        log.info(`${data}`.trim());
      });

      child.stderr?.on("data", (data) => {
        log.info(`${data}`.trim());
      });

      child.on("error", (error) => {
        log.error(`${error}`);
        resolve(false);
      });
    } catch (error) {
      log.error(`${error}`);
      resolve(false);
    }
  });
}
