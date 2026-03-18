import { ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import ConnectionState from "./ConnectionState";
import Provider, { TConCallback } from "./Provider";

/**
 * Provider with reduced subscriptions to handle external file editor.
 */
export default class SubscriberProvider extends Provider {
  /**
   * constructor that initializes a new instance of a provider object.
   *
   * @param host - IP address or hostname of a remote server on remote host.
   * @param rosVersion - ROS version as string of {'1', '2'}
   * @param port - Port of a remote server on remote host. If zero, it depends on the ros version.
   */
  constructor(
    logCtxRef: React.MutableRefObject<ILoggingContext>,
    settingsCtxRef: React.MutableRefObject<ISettingsContext>,
    host: string,
    rosVersion: string,
    port: number = 0,
    useSSL: boolean = false,
  ) {
    super(logCtxRef, settingsCtxRef, host, rosVersion, port, 0, useSSL);
    this.className = "SubscriberProvider";
  }

  public getCallbacks: () => TConCallback[] = () => {
    return [];
  };

  public updateDaemonInit: () => void = async () => {
    this.getDaemonVersion()
      .then((dv) => {
        if (this.isAvailable()) {
          this.daemonVersion = dv;
          this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
          this.updateRosNodes({});
          // this.updateProviderList(true);
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
