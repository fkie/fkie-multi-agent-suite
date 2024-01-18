/**
 * ProviderLaunchConfiguration models launch configuration to start ROS system nodes
 */
class ProviderLaunchConfiguration {
  /** Name of the provider, usually set after the provider was launched. */
  providerName?: string;

  providerId?: string;

  host: string;

  /** If zero the port is depending on ROS version. */
  port: number = 0;

  /** Currently support only 'crossbar-wamp' */
  type: string = 'crossbar-wamp';

  /** Use secure connection */
  useSSL: boolean = false;

  /** ROS version as string of {'1', '2'} */
  rosVersion: string;

  daemon: {
    enable: boolean;
  } = { enable: false };

  discovery: {
    enable: boolean;
    networkId?: number;
    group?: string;
    heartbeatHz?: number;
    robotHosts?: string[];
  } = { enable: false };

  sync: {
    enable: boolean;
    doNotSync?: string[];
    syncTopics?: string[];
  } = { enable: false };

  terminal: {
    enable: boolean;
    port?: number;
  } = { enable: false };

  forceRestart = false;

  /** Try to connect on start. */
  autoConnect: boolean = true;

  /** Start system nodes on failed connection */
  autostart: boolean = false;

  /**
   * Class Constructor
   *
   * @param {string} host - Parameter name
   * @param {string | number | boolean | string[]} value - Parameter value
   */
  constructor(host: string, rosVersion: string, port: number = 0) {
    this.host = host;
    this.rosVersion = rosVersion;
    this.port = port;
  }

  name: () => string = () => {
    return this.providerName ? this.providerName : this.host;
  };
}

export default ProviderLaunchConfiguration;
