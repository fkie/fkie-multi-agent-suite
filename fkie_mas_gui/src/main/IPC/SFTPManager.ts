import log from 'electron-log';
import Client from 'ssh2-sftp-client';
import { ICredential } from '../models/ICredential';
import PasswordManager from './PasswordManager';

/**
 * Class SFTPManager: SFTP interface
 */
class SFTPManager {
  pm: PasswordManager;

  constructor() {
    this.pm = new PasswordManager();
  }

  /**
   * Tests to see if remote file or directory exists.
   * @param {ICredential} credential - SSH credential
   * @param {string} path - Remote path
   * @return {boolean} Returns type of remote object if it exists or false if it does not.
   */
  public checkPassword = async (credential: ICredential) => {
    const sftp = new Client('sftp_check_password');
    let result: string | boolean = false;

    try {
      await sftp.connect(await this.generateConfig(credential));
      // result will be false or d, -, l (dir, file or link)
      result = await sftp.cwd();
      await sftp.end();
      result = !!result;
    } catch (error) {
      log.info('SFTPManager - check password error: ', error);
    }

    return result;
  };

  /**
   * Retrieves a directory listing.
   * This method returns a Promise, which once realized, returns an array of objects representing items in the remote directory.
   * @param {ICredential} credential - SSH credential
   * @param {string} path - Remote directory path
   * @param {string} filter - A function used to filter the items included in the returned array.
   * @return {Array} Returns an array of objects representing items in the remote directory.
   */
  // public list = async (
  //   credential: ICredential,
  //   path: string,
  //   filter?: string
  // ) => {
  //   if (!path) return [];
  //   if (path.length === 0) return [];

  //   const sftp = new Client('sftp_list');

  //   let fileList: Client.FileInfo[] = [];

  //   try {
  //     await sftp.connect(await this.generateConfig(credential));

  //     // TODO: Fix filter argument
  //     fileList = await sftp.list(path);
  //     await sftp.end();
  //   } catch (error) {
  //     log.info('SFTPManager - list error: ', error);
  //   }

  //   return fileList;
  // };

  /**
   * Tests to see if remote file or directory exists.
   * @param {ICredential} credential - SSH credential
   * @param {string} path - Remote path
   * @return {boolean} Returns type of remote object if it exists or false if it does not.
   */
  // public exist = async (credential: ICredential, path: string) => {
  //   const sftp = new Client('sftp_exist');
  //   let result: string | boolean = false;

  //   try {
  //     await sftp.connect(await this.generateConfig(credential));
  //     // result will be false or d, -, l (dir, file or link)
  //     result = await sftp.exists(path);
  //     await sftp.end();
  //     result = result !== false;
  //   } catch (error) {
  //     log.info('SFTPManager - exist error: ', error);
  //   }

  //   return result;
  // };

  /**
   * Returns the attributes associated with the object pointed to by path.
   * @param {ICredential} credential - SSH credential
   * @param {string} path - Remote path
   * @return {Promise<Client.FileStats>} Returns object attributes.
   */
  // public stat = async (credential: ICredential, path: string) => {
  //   const sftp = new Client('sftp_stat');
  //   let result: Client.FileStats | null = null;
  //   try {
  //     await sftp.connect(await this.generateConfig(credential));
  //     result = await sftp.stat(path);
  //     await sftp.end();
  //   } catch (error) {
  //     log.info('SFTPManager - stat error: ', error);
  //   }

  //   return result;
  // };

  /**
   * Retrieve a file from a remote SFTP server.
   * @param {ICredential} credential - SSH credential
   * @param {string} path - Remote file path
   * @return {Promise<string |  NodeJS.WritableStream | Buffer | null>} Returns file content
   */
  // public get = async (
  //   credential: ICredential,
  //   path: string
  // ): Promise<string | NodeJS.WritableStream | Buffer | null> => {
  //   const sftp = new Client('sftp_get');

  //   // TODO: Do we always work with UTF-8?
  //   // fix encoding to utf-8
  //   const options: Client.TransferOptions = {
  //     readStreamOptions: {
  //       flags: 'r',
  //       encoding: 'utf-8',
  //       mode: 0o666,
  //       autoClose: true,
  //     },
  //   };

  //   let fileContent = null;

  //   try {
  //     await sftp.connect(await this.generateConfig(credential));
  //     fileContent = await sftp.get(path, undefined, options);
  //     await sftp.end();
  //   } catch (error) {
  //     log.info('SFTPManager - get error: ', error);
  //   }

  //   return fileContent;
  // };

  /**
   * Upload data from local system to remote SFTP server.
   * @param {ICredential} credential - SSH credential
   * @param {string} content - Data source for data to copy to the remote server.
   * @param {string} path - Remote file path
   * @return {string} Returns file content
   */
  // public put = async (
  //   credential: ICredential,
  //   content: string,
  //   path: string
  // ) => {
  //   // TODO: Return error back to client
  //   if (path.length === 0) {
  //     log.info('SFTPManager - put:  Invalid empty path');
  //     return null;
  //   }

  //   if (content.length === 0) {
  //     log.info('SFTPManager - put:  Invalid empty content');
  //     return null;
  //   }

  //   const sftp = new Client('sftp_put');

  //   // TODO: Do we always work with UTF-8?
  //   // fix encoding to utf-8
  //   const options: Client.TransferOptions = {
  //     readStreamOptions: {
  //       flags: 'r',
  //       encoding: 'utf-8',
  //       mode: 0o666,
  //       autoClose: true,
  //     },
  //     writeStreamOptions: {
  //       flags: 'w', // w - write and a - append
  //       encoding: 'utf-8', // use null for binary files
  //       mode: 0o666, // mode to use for created file (rwx)
  //     },
  //   };

  //   let result = null;

  //   try {
  //     await sftp.connect(await this.generateConfig(credential));
  //     result = await sftp.put(Buffer.from(content), path, options);
  //     await sftp.end();
  //   } catch (error) {
  //     log.info('SFTPManager - put error: ', error);
  //   }

  //   return result;
  // };

  /**
   * Generate configuration file for SFTP connection
   * @param {ICredential} credential - SSH credential
   * @return {object} Returns a configuration file
   */
  private generateConfig = async (credential: ICredential) => {
    // try to get password from password manager
    let pwd = null;
    try {
      pwd = await this.pm.getPassword(credential.service, credential.account);
    } catch (error) {
      log.info('SFTPManager - generateConfig error: ', error);
    }

    // TODO: Check for inexistent passwords
    const config: Client.ConnectOptions = {
      host: credential.host,
      port: credential.port,
      username: credential.username,
      password: pwd || undefined,
    };

    return config;
  };
}

export default SFTPManager;
