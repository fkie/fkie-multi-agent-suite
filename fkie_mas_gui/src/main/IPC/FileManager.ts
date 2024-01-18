import log from 'electron-log';
import { promises as fsp } from 'fs';
import { IFileStats } from '../models/IFileStats';

/**
 * Class FileManager: SFTP interface
 */
class FileManager {
  /**
   * Tests to see if remote file or directory exists.
   * @param {string} path - Remote path
   * @return {Promise<boolean>} Returns type of remote object if it exists or false if it does not.
   */
  public exist = async (path: string): Promise<boolean> => {
    try {
      await fsp.access(path);
      return true;
    } catch {
      return false;
    }
  };

  /**
   * Returns the attributes associated with the object pointed to by path.
   * @param {string} path - Remote path
   * @return {Promise<IFileStats | null>} Returns object attributes.
   */
  public stat = async (path: string): Promise<IFileStats | null> => {
    let result: IFileStats | null = null;
    try {
      const fStats = await fsp.stat(path);
      result = { modifyTime: fStats.mtimeMs };
    } catch (error) {
      log.info('FileManager - stat error: ', error);
    }

    return result;
  };

  /**
   * Retrieve a file from a remote SFTP server.
   * @param {string} path - Remote file path
   * @return {Promise<Buffer | null>} Returns file content
   */
  public get = async (path: string): Promise<Buffer | null> => {
    let fileContent = null;
    try {
      fileContent = await fsp.readFile(path);
    } catch (error) {
      log.info('FileManager - get error: ', error);
    }

    return fileContent;
  };

  /**
   * Upload data from local system to remote SFTP server.
   * @param {string} content - Data source for data to copy to the remote server.
   * @param {string} path - Remote file path
   * @return {Promise<string | null>} Returns operation message
   */
  public put = async (
    content: string,
    path: string,
  ): Promise<string | null> => {
    // TODO: Return error back to client
    if (path.length === 0) {
      log.info('FileManager - put:  Invalid empty path');
      return null;
    }

    if (content.length === 0) {
      log.info('FileManager - put:  Invalid empty content');
      return null;
    }

    let result = null;

    try {
      await fsp.writeFile(path, content);
      result = `Updated the contents of ${path}`;
    } catch (error) {
      log.info('FileManager - put error: ', error);
    }

    return result;
  };
}

export default FileManager;
