/* eslint-disable camelcase */
/**
 * RunNodeStandalone models the request to start a standalone node.
 */
export default class RunNodeStandalone {
  /** The ROS package */
  package_name: string;

  /** The binary file to execute */
  executable: string;

  /** The full path of the binary. Used in case of multiple binaries in the same package. */
  opt_binary: string | undefined;

  /** The name of the ros node. */
  name: string | undefined;

  /** namespace of the node, default: "/" */
  namespace: string | undefined;

  /** Log level */
  loglevel: string | undefined;

  /** Log format */
  logformat: string | undefined;

  /** Custom command prefix. It will be prepended before launch prefix. */
  prefix: string | undefined;

  /** parameters as string. */
  params: string | undefined;

  constructor(
    package_name: string,
    executable: string,
    opt_binary: string,
    name: string,
    namespace: string,
    loglevel: string,
    logformat: string,
    prefix: string,
    params: string
  ) {
    this.package_name = package_name;
    this.executable = executable;
    this.opt_binary = opt_binary;
    this.name = name;
    this.namespace = namespace;
    this.loglevel = loglevel;
    this.logformat = logformat;
    this.prefix = prefix;
    this.params = params;
  }
}
