/* eslint-disable max-classes-per-file */
import { JSONObject, TResult, TResultData } from "@/types";
import { ILoggingContext } from "../../context/LoggingContext";
import { getDefaultPortFromRos } from "../../context/SettingsContext";
import ProviderConnection from "../ProviderConnection";
import CrossbarIO from "./crossbar_io";

/**
 * CrossbarIOConnection class to connect with a running WAMP Router
 */
export default class CrossbarIOConnection extends ProviderConnection {
  static type = "crossbar-wamp";

  crossbar: CrossbarIO;

  /**
   * External logger
   */
  logger: ILoggingContext | null;

  constructor(
    host: string,
    rosVersion: string,
    port: number = 0,
    networkId: number = 0,
    useSSL: boolean = false,
    onClose: (reason: string, details: string) => void = () => {},
    onOpen: () => void = () => {},
    logger: ILoggingContext | null = null
  ) {
    super();
    this.logger = logger;
    const providerPort =
      port !== 0 ? port : getDefaultPortFromRos(CrossbarIOConnection.type, rosVersion, "", networkId);
    this.crossbar = new CrossbarIO(host, providerPort, onClose, onOpen, useSSL);
    this.uri = this.crossbar.wsURI;
    this.port = this.crossbar.port;
    this.host = this.crossbar.host;
  }

  open: () => Promise<boolean> = () => {
    return this.crossbar.open(this.timeout);
  };

  close: () => Promise<void> = () => {
    return this.crossbar.close();
  };

  connected: () => boolean = () => {
    return this.crossbar.connectionIsOpen;
  };

  /**
   * Subscribe a URI topic using current session
   *
   * @param {string} uri - URI to subscribe for. (ex. 'ros.system.pong')
   * @param {function} callback - Callback to be executed when new messages arrives.
   */
  subscribe: (uri: string, callback: (msg: JSONObject) => void) => Promise<TResult> = async (uri, callback) => {
    const result = await this.crossbar.subscribe(uri, (msg: string[]) => {
      if (typeof msg[0] === "string") {
        try {
          const msgObj = JSON.parse(msg[0]);
          callback(msgObj);
        } catch (error) {
          this.logger?.error(`Could not parse subscribed crossbar message ${msg}`, `Error: ${error}`);
        }
      } else {
        callback(msg[0]);
      }
    });
    const rval: TResult = {
      result: result[0],
      message: result[2],
    };
    return Promise.resolve(rval);
  };

  /**
   * Make a request to remote server.
   *
   * @param {string} uri - URI to call for. (ex. 'ros.system.ping')
   * @param {Object} args - Arguments passed to the call
   */
  call: (uri: string, params: unknown[]) => Promise<TResultData> = async (uri, params) => {
    const r = await this.crossbar.call(uri, params);
    if (r[0]) {
      return Promise.resolve({ result: true, message: "", data: JSON.parse(r[1]) } as TResultData);
    }
    throw Error(r[2]);
  };

  /**
   * publish to a URI topic using current session
   *
   * @param {string} uri - URI to publish. (ex. 'ros.remote.ping')
   * @param {object} payload - payload to be sent with request
   */
  publish: (uri: string, payload: JSONObject) => Promise<TResult> = async (uri, payload) => {
    const result = await this.crossbar.publish(uri, [payload]);
    const rval: TResult = {
      result: result[0],
      message: result[2],
    };
    return rval;
  };

  /**
   * Close all subscriptions
   */
  closeSubscriptions: () => Promise<void> = async () => {
    await this.crossbar.closeSubscriptions();
  };

  /**
   * Close given subscription
   */
  closeSubscription: (uri: string) => Promise<void> = async (uri) => {
    await this.crossbar.closeSubscription(uri);
  };
}
