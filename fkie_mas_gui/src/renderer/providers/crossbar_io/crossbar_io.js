import autobahn from 'autobahn-browser';

/**
 * CrossbarIO class connects to a WAMP Application
 */
class CrossbarIO {
  /**
   * constructor
   *
   * @param {string} host - URL of the WAMP Router
   * @param {number} port - Port of the WAMP Router
   * @param {string} realm - WAMP Realm
   * @param {func} onClose - onClose callback
   * @param {func} onOpen - onOpen callback
   * @param {boolean} useWSS - Use WSS instead of WS
   */
  constructor(
    host = 'localhost',
    port = 8080,
    onClose = null,
    onOpen = null,
    useWSS = false,
    realm = 'ros',
  ) {
    this.session = null;
    this.details = null;
    this.connection = null;
    this.connectionIsOpen = false;
    this.wsUnreachable = false;
    this.onCloseCallback = onClose;
    this.onOpenCallback = onOpen;
    this.useWSS = useWSS;
    this.subscriptionList = [];
    this.registrationList = [];

    // the URL of the WAMP Router (Crossbar.io)
    const wsType = this.useWSS ? 'wss' : 'ws';
    const httpType = this.useWSS ? 'https' : 'http';
    this.wsURI = `${wsType}://${host}:${port}/${wsType}`;
    this.httpURI = `${httpType}://${host}:${port}/${wsType}`;
    this.host = host;
    this.port = port;
    this.realm = realm;
  }

  /**
   * Opens a new connection to a WAMP Router
   *
   * @param {number} timeout - Timeout for rejecting te connection attempt.
   * @return {Promise} Connection promise, true when connection succeeded
   */
  open = async (timeout = 5000) => {
    // the WAMP connection to the Router
    /*
    Configuration options:
    https://github.com/crossbario/autobahn-js/blob/master/doc/reference.md

    max_retries: integer - Maximum number of reconnection attempts. Unlimited if set to -1 (default: 15)
    max_retry_delay: float - Maximum delay for reconnection attempts in seconds (default: 300).
    */

    console.log(`create new connection: ${this.wsURI}`);
    this.connection = new autobahn.Connection({
      url: this.wsURI,
      max_retries: 0, // reconnection is handled independently
      // max_retries: 15, // reconnection is handled independently
      max_retry_delay: 300,
      realm: this.realm,
      transports: [
        {
          type: 'websocket',
          url: this.wsURI,
        },
        {
          type: 'longpoll',
          url: this.httpURI,
        },
      ],
      on_user_error: (error, customErrorMessage) => {
        // here comes your custom error handling, when a
        // something went wrong in a user defined callback.
        console.debug(`on_user_error: ${error} - ${customErrorMessage}`);
      },
      on_internal_error: (error, customErrorMessage) => {
        // here comes your custom error handling, when a
        // something went wrong in the autobahn core.
        console.debug(`on_internal_error: ${error} - ${customErrorMessage}`);
      },
    });

    // fired when connection is established and session attached
    this.connection.onopen = this.onOpen;

    // fired when connection was lost (or could not be established)
    this.connection.onclose = this.onClose;

    this.connection.open();

    const start = Date.now();
    const waitForConnection = (resolve, reject) => {
      // connection available :)
      if (this.connectionIsOpen) {
        resolve(true);
      }

      // Max Timeout passed
      else if (timeout && Date.now() - start >= timeout) {
        resolve(false);
      }

      // reconnect
      else {
        setTimeout(waitForConnection.bind(this, resolve, reject), 500);
      }
    };
    return new Promise(waitForConnection);
  };

  /**
   * Close all subscriptions
   */
  closeSubscriptions = async () => {
    try {
      await Promise.all(
        this.subscriptionList.map((subscription) => {
          console.log(
            `CROSSBAR_IO: (${this.host}:${this.port}) Closing subscription to: [${subscription.topic}]`,
          );
          return subscription.unsubscribe();
        }),
      ).catch((error) => {
        console.error(
          `CROSSBAR_IO (${this.host}:${this.port}): Error while unsubscribe: [${error}]`,
        );
      });
    } catch (error) {
      console.error(
        `CROSSBAR_IO (${this.host}:${this.port}): Error while unsubscribe: [${error}]`,
      );
    }
    this.subscriptionList = [];
  };

  /**
   * Close given subscription
   */
  closeSubscription = async (topic) => {
    try {
      const toClose = this.subscriptionList.filter(
        (subscription) => subscription.topic === topic,
      );
      await Promise.all(
        toClose.map((subscription) => {
          console.log(
            `CROSSBAR_IO: (${this.host}:${this.port}) Closing subscription to: [${subscription.topic}]`,
          );
          return subscription.unsubscribe();
        }),
      ).catch((error) => {
        console.error(
          `CROSSBAR_IO (${this.host}:${this.port}): Error while unsubscribe: [${error}]`,
        );
      });
      this.subscriptionList = this.subscriptionList.filter(
        (subscription) => subscription.topic !== topic,
      );
    } catch (error) {
      console.error(
        `CROSSBAR_IO (${this.host}:${this.port}): Error while unsubscribe: [${error}]`,
      );
    }
  };

  /**
   * Close all RPC registrations
   */
  closeRegistrations = async () => {
    try {
      await Promise.all(
        this.registrationList.map((registration) => {
          console.log(
            `CROSSBAR_IO (${this.host}:${this.port}): Closing registration`,
            registration,
          );
          return registration.unregister();
        }),
      ).catch((error) => {
        console.error(
          `CROSSBAR_IO (${this.host}:${this.port}): Error while unregister: [${error}]`,
        );
      });
    } catch (error) {
      console.error(
        `CROSSBAR_IO (${this.host}:${this.port}): Error while unsubscribe: [${error}]`,
      );
    }
    this.registrationList = [];
  };

  /**
   * Close the current connection if any.
   */
  close = async () => {
    // If connection not open, return
    if (!this.connectionIsOpen) return;

    this.connectionIsOpen = false;

    if (this.connection) {
      console.log(`Closing autobahn connection to ${this.wsURI}`);
      if (this.session.isOpen) {
        // close all subscriptions
        await this.closeSubscriptions();
        // close all registrations
        await this.closeRegistrations();
      }

      // close active connection
      await this.connection.close(
        'wamp.goodbye.normal',
        'Closing ROS provider',
      );
    }
    this.connection = null;
  };

  /**
   * Subscribe a WAMP URI topic using current session
   *
   * @param {string} uri - WAMP URI to subscribe for. (ex. 'ros.system.pong')
   * @param {function} callback - Callback to be executed when new messages arrives.
   * @return {List} List with three items: A boolean success, a incoming response object and a string error if failure.
   */
  subscribe = (uri, callback) => {
    if (!this.connectionIsOpen) return false;
    if (
      uri in this.subscriptionList.map((subscription) => subscription.topic)
    ) {
      console.log(`ignore subscription to ${uri}, already subscribed`);
      return false;
    }
    return this.session
      .subscribe(uri, callback)
      .then(
        (subscription) => {
          // subscription succeeded, subscription is an instance of autobahn.Subscription
          this.subscriptionList.push(subscription);
          return [true, subscription, ''];
        },
        (error) => {
          // subscription failed, error is an instance of autobahn.Error
          console.error(
            `crossbar.subscribe: Subscription to [${uri}] failed: `,
            error,
          );
          return [false, null, JSON.stringify(error)];
        },
      )
      .catch((error) => {
        console.error(
          `crossbar.subscribe: Subscription to [${uri}] failed: catch error: `,
          error,
        );
        return [false, null, JSON.stringify(error)];
      });
  };

  /**
   * publish to a WAMP URI topic using current session
   *
   * @param {string} uri - WAMP URI to publish. (ex. 'ros.remote.ping')
   * @param {object} payload - payload to be sent with request
   * @return {List} List with three items: A boolean success, a incoming response object and a string error if failure.
   */
  publish = (uri, payload) => {
    if (!this.connectionIsOpen) return Promise.resolve(false);

    // publish with broker acknowledge (if available)
    // https://github.com/crossbario/autobahn-js/blob/master/doc/reference.md#acknowledgement
    return this.session
      .publish(uri, payload, {}, { acknowledge: true })
      .then(
        (publication) => {
          // publish was successful
          return [true, publication, ''];
        },
        (error) => {
          // publish failed
          console.error(
            `crossbar.publish: Publication of [${uri}] failed: `,
            error,
          );
          return [false, null, JSON.stringify(error)];
        },
      )
      .catch((error) => {
        // crossbar publish failed
        console.error(
          `crossbar.publish: Publication of [${uri}] failed: Catch error: `,
          error,
        );
        return [false, null, JSON.stringify(error)];
      });
  };

  /**
   * register a function to a WAMP URI topic using current session
   *
   * @param {string} uri - WAMP URI of the function
   * @param {function} callback - function to be called when request arrives
   * @return {List} List with three items: A boolean success, a incoming response object and a string error if failure.
   */
  register = (uri, callback) => {
    // return if connection is not open
    if (!this.connectionIsOpen) return Promise.resolve(false);

    // return if UIR already registered
    if (this.session.registrations.includes(uri)) return Promise.resolve(true);

    return this.session
      .register(uri, callback)
      .then(
        (registration) => {
          // registration succeeded
          this.registrationList.push(registration);
          return [true, registration, ''];
        },
        (error) => {
          // registration failed, error is an instance of autobahn.Error
          console.error(
            `crossbar.register: Registration of [${uri}] failed: `,
            error,
          );
          return [false, null, JSON.stringify(error)];
        },
      )
      .catch((error) => {
        console.error(
          `crossbar.register: Registration of [${uri}] failed: catch error: `,
          error,
        );
        return [false, null, JSON.stringify(error)];
      });
  };

  /**
   * Make a request to an given WAMP URI using the current session
   *
   * @param {string} uri - WAMP URI to call for. (ex. 'ros.system.ping')
   * @param {Object} args - Arguments passed to the call
   * @return {List} List with three items: A boolean success, a incoming response object and a string error if failure.
   */
  call = async (uri, args = []) => {
    if (!this.connectionIsOpen) return [false, null, 'Connection is closed'];

    const rCall = await this.session
      .call(uri, args)
      .then(
        (result) => {
          // call was successful
          return [true, result, ''];
        },
        (error) => {
          // call failed
          return [false, null, JSON.stringify(error)];
        },
      )
      .catch((error) => {
        return [false, null, JSON.stringify(error)];
      });

    return rCall;
  };

  onOpen = (session, details) => {
    this.session = session;
    this.details = details;
    this.connectionIsOpen = true;

    if (this.onOpenCallback) this.onOpenCallback(session, details);
  };

  onClose = (reason, details) => {
    //   reason is a string with the possible values
    //   "closed": The connection was closed explicitly (by the application or server). No automatic reconnection will be tried.
    //   "lost": The connection had been formerly established at least once, but now was lost. Automatic reconnection will happen unless you return truthy from this callback.
    //   "unreachable": The connection could not be established in the first place. No automatic reattempt will happen, since most often the cause is fatal (e.g. invalid server URL or server unreachable)
    //   "unsupported": No WebSocket transport could be created. For security reasons the WebSocket spec states that there should not be any specific errors for network-related issues, so no details are returned in this case either.
    // details is an object containing the reason and message passed to autobahn.Connection.close(), and thus does not apply in case of "lost" or "unreachable".
    this.connectionIsOpen = false;
    this.wsUnreachable = reason === 'unreachable';

    if (this.onCloseCallback) this.onCloseCallback(reason, details);
  };
}

export default CrossbarIO;
