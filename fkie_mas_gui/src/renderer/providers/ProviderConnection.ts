import { JSONObject, TResult, TResultData } from "@/types";

export type TProviderTimestamp = {
  timestamp: number;
  diff: number;
};

export type TResultClearPath = {
  node: string;
  result: boolean;
  message: string;
};

export type TResultStartNode = {
  success: boolean;
  message: string;
  details: string;
  response: object | null;
};

export default class ProviderConnection {
  host: string = "";

  port: number = -1;

  uri: string = "";

  useSSL: boolean = false;

  /** Default connection timeout in milliseconds */
  timeout: number = 5000;

  open: () => Promise<boolean> = () => {
    return Promise.resolve(false);
  };

  close: () => Promise<void> = () => {
    return Promise.resolve();
  };

  connected: () => boolean = () => {
    return false;
  };

  /**
   * Subscribe a URI topic using current session
   *
   * @param uri - URI to subscribe for. (ex. 'ros.system.pong')
   * @param callback - Callback to be executed when new messages arrives.
   */
  subscribe: (uri: string, callback: (msg: JSONObject) => void) => Promise<TResult> = async () => {
    throw Error("not implemented");
  };

  /**
   * Make a request to remote server.
   *
   * @param uri - URI to call for. (ex. 'ros.system.ping')
   * @param param - Arguments passed to the call
   */
  call: (uri: string, param: unknown[], timeout?: number) => Promise<TResultData> = async () => {
    throw Error("not implemented");
  };

  /**
   * publish to a URI topic using current session
   *
   * @param uri - URI to publish. (ex. 'ros.remote.ping')
   * @param payload - payload to be sent with request
   */
  publish: (uri: string, payload: JSONObject) => Promise<TResult> = async () => {
    throw Error("not implemented");
  };

  /**
   * Close all subscriptions
   */
  closeSubscriptions: () => Promise<void> = async () => {
    throw Error("not implemented");
  };

  /**
   * Close given subscription
   */
  closeSubscription: (uri: string) => Promise<void> = async () => {
    throw Error("not implemented");
  };
}
