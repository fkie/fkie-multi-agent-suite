import { spawn, StdioOptions } from 'child_process';
import log from 'electron-log';
import os from 'os';
import { Client, ConnectConfig } from 'ssh2';

import { ARGUMENTS, getArgument } from '../CommandLineInterface';
import { ICredential } from '../models/ICredential';
import PasswordManager from './PasswordManager';
import { ISystemInfo, SystemInfo } from './SystemInfo';

const textDecoder = new TextDecoder();
/**
 * Class CommandExecutor: Execute commands locally or remote using SSH2 interface
 */
class CommandExecutor {
  localCredential: ICredential;

  pm: PasswordManager;

  systemInfo?: ISystemInfo;

  constructor() {
    this.pm = new PasswordManager();

    const fetchSystemInfo = async () => {
      this.systemInfo = await new SystemInfo().getInfo();
    };
    fetchSystemInfo();

    // create local credential
    this.localCredential = {
      id: '',
      host: os.hostname(),
      port: 0,
      username: '',
      password: '',
      service: '',
      account: '',
    };
  }

  /**
   * Executes a command using a SSH connection
   * @param {ICredential} credential - SSH credential
   * @param {string} command - Remote directory path
   * @return {Promise<{result: boolean, message: string}>} Returns response
   */
  public exec: (
    credential: ICredential | null,
    command: string,
  ) => Promise<{ result: boolean; message: string }> = async (
    credential: ICredential | null,
    command: string,
  ) => {
    let c = credential;

    // if no credential is given, assumes local host
    if (!c) c = this.localCredential;

    // Set the STDIO config: Ignore or redirect STDOUT/STDERR to current console
    let stdioOptions: StdioOptions | undefined = ['ignore', 'pipe', 'pipe'];
    const parentOut =
      getArgument(ARGUMENTS.SHOW_OUTPUT_FROM_BACKGROUND_PROCESSES) === 'true';
    if (parentOut) {
      stdioOptions = ['inherit', 'pipe', 'pipe'];
    }

    const localIps = ['localhost', '127.0.0.1', os.hostname()];

    if (this.systemInfo) {
      this.systemInfo.networkInterfaces?.forEach((ni) => {
        localIps.push(ni.ip4);
      });
    }

    if (localIps.includes(c.host)) {
      // log.debug(`CommandExecutor exec: Using local process for: [${command}]`);

      // if local command, do not use SSH but child process instead
      return new Promise((resolve, reject) => {
        try {
          let errorString = '';
          const child = spawn(command, [], {
            shell: true,
            stdio: stdioOptions,
            detached: false,
          });

          child.on('close', (code) => {
            if (code !== 0) {
              resolve({
                result: false,
                message: errorString,
              });
            } else {
              resolve({
                result: true,
                message: '',
              });
              // resolve(`Closed with code: ${code}`);
            }
          });
          child.stdout?.on('data', function (data) {
            if (parentOut) {
              console.log(`${data}`);
              if (data.includes("[rosrun] Couldn't find executable")) {
                errorString += data;
              }
            }
          });
          child.stderr?.on('data', function (data) {
            if (parentOut) {
              console.error(`${data}`);
            }
            errorString += data;
          });

          child.on('error', (error) => {
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

    return new Promise((resolve, reject) => {
      if (!command)
        resolve({
          result: false,
          message: 'Invalid empty command',
        });
      const conn = new Client();
      try {
        conn
          .on('ready', () => {
            conn.exec(command, (err: Error | undefined, sshStream: any) => {
              if (err) {
                resolve({
                  result: false,
                  message: err?.message,
                });
              }
              let errorString = '';
              // .on('close', (code: string, signal: string) => {
              sshStream
                .on('close', (code: number, signal: string) => {
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
                      message: '',
                    });
                  }
                  conn.end();
                })
                .stdout.on('data', (data: Buffer) => {
                  if (parentOut) {
                    console.log(`${textDecoder.decode(data)}`);
                  }
                  resolve({
                    result: true,
                    message: textDecoder.decode(data),
                  });
                })
                .stderr.on('data', (data: Buffer) => {
                  if (parentOut) {
                    console.error(`${textDecoder.decode(data)}`);
                  }
                  errorString += textDecoder.decode(data);
                  resolve({
                    result: true,
                    message: textDecoder.decode(data),
                  });
                });
            });
          })
          .connect(connectionConfig);
        conn.on('error', (error: any) => {
          log.warn('CommandExecutor - connect error: ', error);
          resolve({
            result: false,
            message: error.message,
          });
        });
      } catch (error: any) {
        log.info('CommandExecutor - exec error: ', error);
        resolve({
          result: false,
          message: error.message,
        });
      }
    });
  };

  /**
   * Generate configuration file for SSH connection
   * @param {ICredential} credential - SSH credential
   * @return {object} Returns a configuration file
   */
  private generateConfig = async (credential: ICredential) => {
    // try to get password from password manager
    let pwd = null;
    try {
      pwd = await this.pm.getPassword(credential.service, credential.account);
    } catch (error) {
      log.info('CommandExecutor - generateConfig error: ', error);
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
