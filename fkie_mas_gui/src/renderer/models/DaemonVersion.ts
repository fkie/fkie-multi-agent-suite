import { generateUniqueId } from "../utils";

/**
 * DaemonVersion models version of the daemon node
 */
class DaemonVersion {
  id: string;

  version: string;

  date: string;

  /**
   * Class Constructor
   *
   * @param {string} version - Version
   * @param {string} date - Date
   */
  constructor(version: string, date: string) {
    this.id = generateUniqueId();
    this.version = version;
    this.date = date;
  }
}

export default DaemonVersion;
