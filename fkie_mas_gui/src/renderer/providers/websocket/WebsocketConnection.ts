import JSON5 from "json5";
import { JSONObject } from "@/types";
import { ILoggingContext } from "../../context/LoggingContext";
import { getDefaultPortFromRos } from "../../context/SettingsContext";
import ProviderConnection, { IResult } from "../ProviderConnection";

interface IQueueItem {
  promise: [
    Parameters<ConstructorParameters<typeof Promise>[0]>[0],
    Parameters<ConstructorParameters<typeof Promise>[0]>[1],
  ];
  timeout?: ReturnType<typeof setTimeout>;
}

interface IQueue {
  [x: number]: IQueueItem;
}

interface ISubscriptions {
  [uri: string]: (msg: JSONObject) => void;
}

/**
 * WebsocketConnection class to connect with a running daemon with websocket
 */
export default class WebsocketConnection extends ProviderConnection {
  static type = "websocket";

  websocket: WebSocket | null = null;

  /**
   * External logger
   */
  logger: ILoggingContext | null;

  private subscriptions: ISubscriptions;

  private queue: IQueue;

  private rpcId: number = 0;

  private onClose: (reason: string, details: string) => void = () => {};

  private onOpen: () => void = () => {};

  constructor(
    host: string,
    rosVersion: string,
    port: number = 0,
    useSSL: boolean = false,
    onClose: (reason: string, details: string) => void = () => {},
    onOpen: () => void = () => {},
    logger: ILoggingContext | null = null
  ) {
    super();
    this.subscriptions = {};
    this.queue = {};
    this.rpcId = 0;
    this.logger = logger;
    const providerPort = port !== 0 ? port : getDefaultPortFromRos(WebsocketConnection.type, rosVersion, "");
    this.uri = `ws://${host}:${providerPort}`;

    this.port = providerPort;
    this.host = host;
    this.useSSL = useSSL;
    this.onClose = onClose;
    this.onOpen = onOpen;
  }

  generateRequestId: () => number = () => {
    this.rpcId += 1;
    return this.rpcId;
  };

  open: () => Promise<boolean> = () => {
    if (this.websocket !== null) return Promise.resolve(true);
    this.websocket = new WebSocket(this.uri);
    this.websocket.addEventListener("open", () => {
      this.logger?.info(`websocket connected to ${this.websocket?.url}`, "", false);
      this.onOpen();
    });
    this.websocket.addEventListener("close", (event: CloseEvent) => {
      this.logger?.info(`websocket disconnected from ${this.websocket?.url}`, "");
      this.websocket = null;
      this.onClose(event.reason, `${event.code}`);
    });
    this.websocket.addEventListener("error", (event) => {
      this.logger?.error(`error on connected to ${this.websocket?.url}`, `event.type: ${JSON.stringify(event.type)}`);
      this.websocket = null;
      return Promise.resolve(false);
    });
    this.websocket.addEventListener("message", (event: MessageEvent) => {
      this.handleMessage(event.data);
    });
    const start = Date.now();
    const waitForConnection = (resolve: (value: boolean) => void, reject: (reason?: boolean) => void) => {
      // connection available :)
      if (this.connected()) {
        resolve(true);
      }

      // Max Timeout passed
      else if (
        this.websocket?.readyState === WebSocket.CLOSED ||
        (this.timeout && Date.now() - start >= this.timeout)
      ) {
        resolve(false);
      }

      // reconnect
      else {
        setTimeout(waitForConnection.bind(this, resolve, reject), 500);
      }
    };
    return new Promise(waitForConnection);
  };

  close: () => Promise<void> = () => {
    return Promise.resolve(this.websocket?.close());
  };

  connected: () => boolean = () => {
    return this.websocket?.readyState === WebSocket.OPEN;
  };

  /**
   * Subscribe a URI topic using current session
   *
   * @param {string} uri - URI to subscribe for. (ex. 'ros.system.pong')
   * @param {function} callback - Callback to be executed when new messages arrives.
   */
  subscribe: (uri: string, callback: (msg: JSONObject) => void) => Promise<IResult> = async (uri, callback) => {
    this.subscriptions[uri] = callback;
    const result = await this.call("sub", [uri])
      .catch((err) => {
        return {
          result: false,
          message: err,
        };
      })
      .then((value) => {
        return {
          result: value,
          message: "",
        };
      });
    if (!result.result) {
      delete this.subscriptions[uri];
    }
    return Promise.resolve(result as unknown as IResult);
  };

  /**
   * Make a request to remote server.
   *
   * @param {string} uri - URI to call for. (ex. 'ros.system.ping')
   * @param {Object} params - Arguments passed to the call
   */
  call: (uri: string, params: unknown[]) => Promise<JSONObject> = async (uri, params) => {
    return new Promise((resolve, reject) => {
      if (!this.connected()) reject(new Error(`[${this.uri}] socket not ready`));

      const rpcId = this.generateRequestId();

      const message = {
        uri,
        id: rpcId,
        params,
      };

      this.websocket?.send(JSON.stringify(message));
      this.queue[rpcId] = { promise: [resolve as never, reject] };
      this.queue[rpcId].timeout = setTimeout(() => {
        delete this.queue[rpcId];
        reject(new Error(`[${this.uri}] reply timeout`));
      }, this.timeout);
    });
  };

  /**
   * publish to a URI topic using current session
   *
   * @param {string} uri - URI to publish. (ex. 'ros.remote.ping')
   * @param {object} payload - payload to be sent with request
   */
  publish: (uri: string, payload: JSONObject) => Promise<IResult> = async (uri, payload) => {
    if (!this.connected()) {
      return Promise.reject({
        result: false,
        message: "websocket not connected",
      });
    }

    const message = {
      uri,
      message: payload,
    };
    this.websocket?.send(JSON.stringify(message));
    return Promise.resolve({
      result: true,
      message: "",
    });
  };

  /**
   * Close all subscriptions
   */
  closeSubscriptions: () => Promise<void> = async () => {
    Object.keys(this.subscriptions).every(async (key) => {
      await this.call("unsub", [key]).catch((error) => {
        this.logger?.warn(`failed unregister ${key}`, `${error}`, false);
      });
    });
    this.subscriptions = {};
    return Promise.resolve();
  };

  /**
   * Close given subscription
   */
  closeSubscription: (uri: string) => Promise<void> = async (uri) => {
    await this.call("unsub", [uri]);
    delete this.subscriptions[uri];
  };

  handleMessage: (msg: string) => void = (msg) => {
    try {
      const message = JSON5.parse(msg);
      if ("id" in message) {
        // object with "id" means the response for an rpc request
        if (!this.queue[message.id]) {
          // timeouted or wrong id
          return;
        }
        // reject early since server's response is invalid
        if ("error" in message === "result" in message)
          this.queue[message.id].promise[1]({
            result: false,
            message: 'Server response malformed. Response must include either "result"' + ' or "error", but not both.',
          });

        if (this.queue[message.id].timeout) {
          clearTimeout(this.queue[message.id].timeout);
        }

        if (message.error) {
          this.queue[message.id].promise[1]({
            result: false,
            message: message.error,
          });
        } else {
          this.queue[message.id].promise[0]({
            result: true,
            message: message.result,
          });
        }

        delete this.queue[message.id];
      } else {
        // handle subscriptions
        const cbSub = this.subscriptions[message.uri];
        if (cbSub) {
          cbSub(message.message);
        }
      }
    } catch (error) {
      this.logger?.warn(`[${this.uri}] error while handle received message: ${error}`, `${JSON.stringify(msg)}`, false);
    }
  };
}
