/**
 * PathItem models files in a package
 */
class PathItem {
  id: string;

  path: string;

  mtime: number;

  size: number;

  type: string;

  host: string;

  // these values are updated in package explorer after the item is received
  name?: string;
  package?: string;
  relativePath?: string;

  /**
   * Class Constructor
   *
   * @param {string} path - Absolute path of the file or directory
   * @param {number} mtime - Time of last modification of path. The return value is a number giving the number of seconds since the epoch
   * @param {number} size - Size, in bytes, of path
   * @param {string} type - One of types {file, dir, symlink, package}
   * @param {string} host - host IP of the owner
   */
  constructor(path: string, mtime: number, size: number, type: string, host: string) {
    this.id = this.guidGenerator();
    this.path = path;
    this.mtime = mtime;
    this.size = size;
    this.type = type;
    this.host = host;
  }

  /**
   * Generates an unique string Identifier
   *
   * @return {string} Returns a unique string identifier
   */
  guidGenerator: () => string = () => {
    return Date.now().toString(36) + Math.random().toString(36).substr(2);
  };
}

export default PathItem;
