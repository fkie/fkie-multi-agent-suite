import { ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import { ConnectionState } from "./ConnectionState";
import Provider, { IConCallback } from "./Provider";

/**
 * Provider with reduced subscriptions to handle external file editor.
 */
export default class SubscriberProvider extends Provider {
  /**
   * constructor that initializes a new instance of a provider object.
   *
   * @param {ISettingsContext} settings - External settings
   * @param {string} host - IP address or hostname of a remote server on remote host.
   * @param {string} rosVersion - ROS version as string of {'1', '2'}
   * @param {number} port - Port of a remote server on remote host. If zero, it depends on the ros version.
   * @param {ILoggingContext | null} logger - External logger
   */
  constructor(
    settings: ISettingsContext,
    host: string,
    rosVersion: string,
    port: number = 0,
    useSSL: boolean = false,
    logger: ILoggingContext | null = null
  ) {
    super(settings, host, rosVersion, port, useSSL, logger);
  }

  public getCallbacks: () => IConCallback[] = () => {
    return [];
  };

  public updateDaemonInit: () => void = async () => {
    this.getDaemonVersion()
      .then((dv) => {
        if (this.isAvailable()) {
          this.daemonVersion = dv;
          this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          this.updateRosNodes({});
          this.updateProviderList();
          return true;
        }
        return false;
      })
      .catch((error) => {
        this.setConnectionState(ConnectionState.STATES.ERRORED, `Daemon no reachable: ${error}`);
        return false;
      });
  };
}
