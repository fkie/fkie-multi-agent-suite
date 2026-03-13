import { JSONObject } from "@/types";
import { emitCustomEvent } from "react-custom-events";
import { ILoggingContext } from "../context/LoggingContext";
import { ISettingsContext } from "../context/SettingsContext";
import { URI } from "../models";
import ConnectionState from "./ConnectionState";
import { EventProviderLaunchLoaded } from "./events";
import { EVENT_PROVIDER_LAUNCH_LOADED } from "./eventTypes";
import Provider, { TConCallback } from "./Provider";

/**
 * Provider with reduced subscriptions to handle external file editor.
 */
export default class EditorProvider extends Provider {
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
    useSSL: boolean = false
  ) {
    super(logCtxRef, settingsCtxRef, host, rosVersion, port, 0, useSSL);
    this.className = "EditorProvider";
  }

  public getCallbacks: () => TConCallback[] = () => {
    return [
      { uri: URI.ROS_PATH_CHANGED, callback: this.callbackChangedFile },
      { uri: URI.ROS_LAUNCH_CHANGED, callback: this.updateRosNodes },
    ];
  };

  public updateDaemonInit: () => void = async () => {
    this.getDaemonVersion()
      .then((dv) => {
        if (this.isAvailable()) {
          this.daemonVersion = dv;
          this.setConnectionState(ConnectionState.STATES.CONNECTED, "");
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

  public updateRosNodes: (msg: JSONObject) => void = async (msg) => {
    this.log().debug(`Trigger update ros nodes for ${this.id}`, "");
    const msgObj = msg as unknown as { path: string; action: string; requester: string };
    if (msgObj?.path) {
      emitCustomEvent(EVENT_PROVIDER_LAUNCH_LOADED, new EventProviderLaunchLoaded(this, msgObj.path, msgObj.requester));
    }
  };
}
