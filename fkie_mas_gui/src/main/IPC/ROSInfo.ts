import { TRosInfo } from "@/types";
/**
 * Read ROS info stored in Environment variables
 */
export class ROSInfo {
  // both
  version: string | undefined; // ROS_VERSION

  pythonVersion: string | undefined; // ROS_PYTHON_VERSION

  packagePath: string | undefined; // ROS_PACKAGE_PATH

  etcDir: string | undefined; // ROS_ETC_DIR

  masterUri: string | undefined; // ROS_MASTER_URI

  root: string | undefined; // ROS_ROOT

  distro: string | undefined; // ROS_DISTRO

  // ROS1 specific
  lispPackageDirectories: string | undefined; // ROSLISP_PACKAGE_DIRECTORIES

  // ROS2 specific
  domainId: string | undefined; // ROS_DOMAIN_ID

  rmwImplementation: string | undefined;

  localhostOnly: string | undefined; // ROS_LOCALHOST_ONLY

  constructor() {
    // check if running in NodeJs Node
    if (!process || !process.env) return;

    // TODO: Shall we clean up / validate environment variables?
    this.version = process.env.ROS_VERSION;
    this.pythonVersion = process.env.ROS_PYTHON_VERSION;
    this.packagePath = process.env.ROS_PACKAGE_PATH;
    this.etcDir = process.env.ROS_ETC_DIR;
    this.masterUri = process.env.ROS_MASTER_URI;
    this.root = process.env.ROS_ROOT;
    this.distro = process.env.ROS_DISTRO;
    this.lispPackageDirectories = process.env.ROSLISP_PACKAGE_DIRECTORIES;
    this.domainId = process.env.ROS_DOMAIN_ID;
    this.rmwImplementation = process.env.RMW_IMPLEMENTATION;
    this.localhostOnly = process.env.ROS_LOCALHOST_ONLY;
  }

  public async getInfo(): Promise<TRosInfo> {
    return Promise.resolve({
      version: this.version,
      pythonVersion: this.pythonVersion,
      etcDir: this.etcDir,
      masterUri: this.masterUri,
      root: this.root,
      distro: this.distro,
      domainId: this.domainId,
      rmwImplementation: this.rmwImplementation,
      localhostOnly: this.localhostOnly,
    } as TRosInfo);
  }

  /**
   * Get a string representation of this object
   *
   */
  public toString(): string {
    return JSON.stringify(this.getInfo());
  }
}
