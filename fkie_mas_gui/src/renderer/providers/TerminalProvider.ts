import { ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import { URI } from "../models";
import ConnectionState from "./ConnectionState";
import Provider, { TConCallback } from "./Provider";

/**
 * Provider with reduced subscriptions to handle external terminal needs.
 */
export default class TerminalProvider extends Provider {
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
    super(settings, host, rosVersion, port, 0, useSSL, logger);
    this.className = "TerminalProvider";
  }

  public getCallbacks: () => TConCallback[] = () => {
    return [
      { uri: URI.ROS_NODES_CHANGED, callback: this.updateRosNodes },
      { uri: URI.ROS_SCREEN_LIST, callback: this.callbackScreensUpdate },
    ];
  };

  public updateDaemonInit: () => void = async () => {
    this.getDaemonVersion()
      .then((dv) => {
        if (this.isAvailable()) {
          this.daemonVersion = dv;
          this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          this.updateRosNodes({});
          this.updateScreens();
          this.updateProviderList(true);
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
