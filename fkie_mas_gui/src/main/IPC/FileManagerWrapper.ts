import { ICredential } from '../models/ICredential';
import FileManager from './FileManager';
import SFTPManager from './SFTPManager';

/**
 * Class FileManagerWrapper: Wraps FileManager and SFTPManager together for unified file handling.
 */
class FileManagerWrapper {
  fm: FileManager;

  sftpm: SFTPManager;

  constructor() {
    this.fm = new FileManager();
    this.sftpm = new SFTPManager();
  }

  public checkPassword = async (credential: ICredential) => {
    if (credential) {
      return this.sftpm.checkPassword(credential);
    }
    return true;
  };

  // public exist = async (credential: ICredential | null, path: string) => {
  //   if (credential) {
  //     return this.sftpm.exist(credential, path);
  //   }
  //   return this.fm.exist(path);
  // };

  // public stat = async (credential: ICredential | null, path: string) => {
  //   if (credential) {
  //     return this.sftpm.stat(credential, path);
  //   }
  //   return this.fm.stat(path);
  // };

  // public get = async (credential: ICredential | null, path: string) => {
  //   if (credential) {
  //     return this.sftpm.get(credential, path);
  //   }
  //   return this.fm.get(path);
  // };

  // public put = async (
  //   credential: ICredential | null,
  //   content: string,
  //   path: string
  // ) => {
  //   if (credential) {
  //     return this.sftpm.put(credential, content, path);
  //   }
  //   return this.fm.put(content, path);
  // };
}

export default FileManagerWrapper;
