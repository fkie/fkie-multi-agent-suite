import JSONObject from '../models/JsonObject';

export interface IProviderTimestamp {
  timestamp: number;
  diff: number;
}

/**
 * @param success: A boolean success
 * @param response: a incoming response object
 * @param error: a string error if failure.
 */
export interface ICallResult {
  success: boolean;
  response: string;
  error: string;
}

export interface IResultClearPath {
  node: string;
  result: boolean;
  message: string;
}

export interface IResult {
  result: boolean;
  message: string;
}

export interface IResultStartNode {
  success: boolean;
  message: string;
  details: string;
  response: object | null;
}

export default class ProviderConnection {
  host: string = '';

  port: number = -1;

  uri: string = '';

  useSSL: boolean = false;

  /** Default connection timeout in milliseconds */
  timeout: number = 5000;

  open: () => Promise<any> = () => {
    return Promise.resolve({});
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
   * @param {string} uri - URI to subscribe for. (ex. 'ros.system.pong')
   * @param {function} callback - Callback to be executed when new messages arrives.
   */
  subscribe: (
    uri: string,
    callback: (msg: JSONObject) => void,
  ) => Promise<IResult> = async () => {
    throw Error('not implemented');
  };

  /**
   * Make a request to remote server.
   *
   * @param {string} uri - URI to call for. (ex. 'ros.system.ping')
   * @param {Object} param - Arguments passed to the call
   */
  call: (uri: string, param: any[]) => Promise<JSONObject> = async () => {
    throw Error('not implemented');
  };

  /**
   * publish to a URI topic using current session
   *
   * @param {string} uri - URI to publish. (ex. 'ros.remote.ping')
   * @param {object} payload - payload to be sent with request
   */
  publish: (uri: string, payload: JSONObject) => Promise<IResult> =
    async () => {
      throw Error('not implemented');
    };

  /**
   * Close all subscriptions
   */
  closeSubscriptions: () => Promise<void> = async () => {
    throw Error('not implemented');
  };

  /**
   * Close given subscription
   */
  closeSubscription: (uri: string) => Promise<void> = async () => {
    throw Error('not implemented');
  };
}
