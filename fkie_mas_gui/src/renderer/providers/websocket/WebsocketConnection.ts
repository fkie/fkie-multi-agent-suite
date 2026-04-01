import { JSONObject, TResult, TResultData } from "@/types";
import JSON5 from "json5";
import { ILoggingContext } from "../../context/LoggingContext";
import { getDefaultPortFromRos } from "../../context/SettingsContext";
import ProviderConnection from "../ProviderConnection";

type IQueueItem = {
  promise: [
    Parameters<ConstructorParameters<typeof Promise>[0]>[0],
    Parameters<ConstructorParameters<typeof Promise>[0]>[1],
  ];
  timeout?: ReturnType<typeof setTimeout>;
};

type IQueue = {
  [x: number]: IQueueItem;
};

type ISubscriptions = {
  [uri: string]: (msg: JSONObject) => void;
};

/**
 * WebsocketConnection class to connect with a running daemon via WebSocket

 */
export default class WebsocketConnection extends ProviderConnection {
  static type = "websocket";

  websocket: WebSocket | null = null;

  private subscriptions: ISubscriptions;
  private queue: IQueue;
  private rpcId: number = 0;

  private onClose: (reason: string, details: string) => void = () => {};
  private onOpen: () => void = () => {};
  private onReconnect: () => void = () => {};

  // --- reconnect configuration ---
  /** Whether the connection should auto-reconnect after unexpected close */
  private shouldReconnect = true;

  /** Current number of reconnect attempts */
  private reconnectAttempts = 0;

  /** Base delay (in ms) used for reconnect backoff */
  private reconnectDelayMs = 2000;

  /** Promise representing an in-flight connection attempt */
  private connectingPromise: Promise<boolean> | null = null;

  constructor(
    private logCtxRef: React.MutableRefObject<ILoggingContext>,
    host: string,
    rosVersion: string,
    port: number = 0,
    domainId: number = 0,
    useSSL: boolean = false,
    onClose: (reason: string, details: string) => void = () => {},
    onOpen: () => void = () => {},
    onReconnect: () => void = () => {}
  ) {
    super();
    this.subscriptions = {};
    this.queue = {};
    this.rpcId = 0;
    this.rosVersion = rosVersion;
    this.domainId = domainId;

    const providerPort = port !== 0 ? port : getDefaultPortFromRos(WebsocketConnection.type, rosVersion, "", domainId);

    this.port = providerPort;
    this.host = host;
    this.useSSL = useSSL;

    const scheme = this.useSSL ? "wss" : "ws";
    this.uri = `${scheme}://${host}:${providerPort}`;

    this.onClose = onClose;
    this.onOpen = onOpen;
    this.onReconnect = onReconnect;
  }

  private log(): ILoggingContext {
    return this.logCtxRef.current;
  }

  generateRequestId: () => number = () => {
    this.rpcId += 1;
    return this.rpcId;
  };

  /**
   * Try to open the WebSocket connection.
   * Resolves to true on successful connection, false on clean close, rejects on error/timeout.

   */
  open: () => Promise<boolean> = () => {
    // If already open, resolve immediately
    if (this.connected()) return Promise.resolve(true);

    // If a connection attempt is already in progress, return that promise
    if (this.websocket && this.websocket.readyState === WebSocket.CONNECTING && this.connectingPromise) {
      return this.connectingPromise;
    }

    // We want to auto-reconnect on unexpected closes
    this.shouldReconnect = true;

    const scheme = this.useSSL ? "wss" : "ws";
    this.uri = `${scheme}://${this.host}:${this.port}`;

    this.connectingPromise = new Promise<boolean>((resolve, reject) => {
      const ws = new WebSocket(this.uri);
      this.websocket = ws;

      const handleOpen = () => {
        // This is called once when the connection is successfully established
        this.log().success(`websocket connected to ${ws.url}`, "", "connected");
        this.onOpen();
        this.reconnectAttempts = 0;

        // Mark the "connecting" promise as done
        this.connectingPromise = null;
        resolve(true);
      };

      const handleClose = (event: CloseEvent) => {
        // This is called on *any* close (failed connect or later disconnect)
        this.log().info(`websocket disconnected from ${this.uri}`, "", "disconnected");
        this.websocket = null;
        this.onClose(event.reason, `${event.code}`);

        // If we were still in the "connecting" phase, finish that promise
        if (this.connectingPromise) {
          this.connectingPromise = null;
          // You can choose resolve(false) or reject(...) here
          resolve(false);
        }

        // For established connections we may want to auto-reconnect
        if (this.shouldReconnect) {
          this.scheduleReconnect();
        }
      };

      const handleError = (event: Event) => {
        // This is called on network errors etc.
        this.log().error(
          `error on connect to ${this.uri}`,
          `event.type: ${JSON.stringify(event.type)}\nIs the daemon running?\nIs the hostname being resolved to the correct IP address?\nPlease check the details in the console by pressing F12.`
        );

        // Ensure the socket is closed, which will also trigger handleClose
        try {
          ws.close();
        } catch {
          // ignore
        }

        this.websocket = null;

        // If we are still in the "connecting" phase, reject the promise
        if (this.connectingPromise) {
          this.connectingPromise = null;
          reject(new Error(`[${this.uri}] connection error`));
        }
      };

      const handleMessage = (event: MessageEvent) => {
        this.handleMessage(event.data);
      };

      // IMPORTANT: these listeners stay attached for the whole lifetime
      ws.addEventListener("open", handleOpen);
      ws.addEventListener("close", handleClose);
      ws.addEventListener("error", handleError);
      ws.addEventListener("message", handleMessage);

      // Optional: connection timeout for the initial connect
      if (this.timeout) {
        setTimeout(() => {
          if (!this.connected() && ws.readyState === WebSocket.CONNECTING) {
            this.log().warn(`[${this.uri}] connection timeout after ${this.timeout}ms`, "");
            try {
              ws.close(); // will trigger handleClose
            } catch {
              // ignore
            }
            this.websocket = null;

            if (this.connectingPromise) {
              this.connectingPromise = null;
              reject(new Error(`[${this.uri}] connection timeout`));
            }
          }
        }, this.timeout);
      }
    });

    return this.connectingPromise;
  };

  /**
   * Schedule an automatic reconnect with simple backoff.

   */
  private scheduleReconnect() {
    if (!this.shouldReconnect) {
      return;
    }

    const delay = this.reconnectDelayMs * (this.reconnectAttempts + 1);
    this.reconnectAttempts += 1;

    this.log().info(`[${this.uri}] trying to reconnect in ${delay}ms (attempt ${this.reconnectAttempts})`, "");

    setTimeout(() => {
      // Only attempt reconnect if still allowed
      if (!this.shouldReconnect) return;
      this.onReconnect();
      this.open().catch((err) => {
        this.log().warn(`[${this.uri}] reconnect attempt failed: ${err}`, "");
      });
    }, delay);
  }

  /**
   * Close the WebSocket connection and disable auto-reconnect.

   */
  close: () => Promise<void> = async () => {
    this.shouldReconnect = false;

    if (this.websocket) {
      this.websocket.close();
    }

    return Promise.resolve();
  };

  connected: () => boolean = () => {
    return this.websocket?.readyState === WebSocket.OPEN;
  };

  /**
   * Subscribe a URI topic using current session.

   *
   * @param uri - URI to subscribe to (e.g. 'ros.system.pong')
   * @param callback - Callback executed when new messages arrive

   */
  subscribe: (uri: string, callback: (msg: JSONObject) => void) => Promise<TResult> = async (uri, callback) => {
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

    return Promise.resolve(result as unknown as TResult);
  };

  /**
   * Make a request to remote server.

   *
   * @param uri - URI to call (e.g. 'ros.system.ping')
   * @param params - Arguments passed to the call
   * @param timeout - Per-call timeout in ms (falls back to this.timeout if not provided)

   */
  call: (uri: string, params: unknown[], timeout?: number) => Promise<TResultData> = (uri, params, timeout = 5000) => {
    return new Promise((resolve, reject) => {
      if (!this.connected()) {
        reject(new Error(`[${this.uri}] socket not ready`));
        return;
      }

      const rpcId = this.generateRequestId();

      const message = {
        uri,
        id: rpcId,
        params,
      };

      this.websocket?.send(JSON.stringify(message));

      this.queue[rpcId] = { promise: [resolve as never, reject] };

      const effectiveTimeout = timeout ?? this.timeout;

      if (effectiveTimeout) {
        this.queue[rpcId].timeout = setTimeout(() => {
          delete this.queue[rpcId];
          reject(new Error(`[${this.uri}] reply timeout`));
        }, effectiveTimeout);
      }
    });
  };

  /**
   * Publish to a URI topic using current session.

   *
   * @param uri - URI to publish to (e.g. 'ros.remote.ping')
   * @param payload - Payload to be sent with the request

   */
  publish: (uri: string, payload: JSONObject) => Promise<TResult> = async (uri, payload) => {
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
   * Close all subscriptions on the server and clear local subscription callbacks.

   */
  closeSubscriptions: () => Promise<void> = async () => {
    const keys = Object.keys(this.subscriptions);

    await Promise.all(
      keys.map((key) =>
        this.call("unsub", [key]).catch((error) => {
          this.log().warn(`failed unregister ${key}`, `${error}`);
        })
      )
    );

    this.subscriptions = {};
    return Promise.resolve();
  };

  /**
   * Close a specific subscription.

   */
  closeSubscription: (uri: string) => Promise<void> = async (uri) => {
    await this.call("unsub", [uri]).catch((error) => {
      this.log().warn(`failed unregister ${uri}`, `${error}`);
    });
    delete this.subscriptions[uri];
  };

  /**
   * Handle all incoming WebSocket messages.

   */
  handleMessage: (msg: string) => void = (msg) => {
    try {
      const message = JSON5.parse(msg);

      // RPC response: must have an "id"
      if ("id" in message) {
        const id = message.id;

        if (!this.queue[id]) {
          // Either already timed out or unknown id
          return;
        }

        const hasError = "error" in message;
        const hasResult = "result" in message;

        // Reject early if server response is malformed
        if (hasError === hasResult) {
          this.queue[id].promise[1]({
            result: false,
            message: 'Server response malformed. Response must include either "result" or "error", but not both.',
            data: null,
            error: "malformed response",
          } as TResultData);

          if (this.queue[id].timeout) {
            clearTimeout(this.queue[id].timeout);
          }
          delete this.queue[id];
          return;
        }

        if (this.queue[id].timeout) {
          clearTimeout(this.queue[id].timeout);
        }

        if (hasError && message.error) {
          this.queue[id].promise[1]({
            result: false,
            message: message.error,
            data: null,
            error: "error",
          } as TResultData);
        } else {
          this.queue[id].promise[0]({
            result: true,
            message: "",
            data: message.result,
          } as TResultData);
        }

        delete this.queue[id];
      } else {
        // Subscription message
        const cbSub = this.subscriptions[message.uri];
        if (cbSub) {
          cbSub(message.message);
        }
      }
    } catch (error) {
      this.log().warn(`[${this.uri}] error while handling received message: ${error}`, `${JSON.stringify(msg)}`);
    }
  };
}
