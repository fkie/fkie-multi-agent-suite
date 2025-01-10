import { generateUniqueId } from "../utils";

/**
 * DaemonVersion models version of the daemon node
 */
export default class DaemonVersion {
  id: string;

  version: string;

  date: string;

  constructor(version: string, date: string) {
    this.id = generateUniqueId();
    this.version = version;
    this.date = date;
  }
}
