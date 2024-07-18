import ROSLIB from "roslib";
import { ILoggingContext } from "../../context/LoggingContext";
import { IRosProvider, RosNode, RosTopic } from "../../models";
import { generateUniqueId } from "../../utils";

/**
 * RosBridgeProvider class implements a IRosProvider to connect with ROS using ROSLIB
 */
class RosBridgeProvider implements IRosProvider {
  private rosClient?: ROSLIB.Ros; // The roslib client when we're connected.

  url: string; // WebSocket URL

  /**
   * Unique Identifier
   */
  id: string;

  /**
   * name of the provider
   */
  name: string;

  /**
   * True if a class initialized
   */
  initialized: boolean;

  /**
   * True if a connection to a web bridge server succeeded
   */
  connected: boolean;

  /**
   * Type of provider
   */
  type: string = "ros-bridge";

  /**
   * Host name
   */
  host: string;

  /**
   * Port number
   */
  port: number;

  /**
   * External logger
   */
  logger: ILoggingContext | null;

  /**
   * constructor
   *
   * @param {string} url - Web bridge URI
   */
  constructor(name = "ros-bridge-provider", host = "localhost", port = 9090, logger: ILoggingContext | null = null) {
    this.name = name;
    this.initialized = false;
    this.connected = false;
    this.host = host;
    this.port = port;
    this.url = `ws://${host}:${port}`;
    this.id = generateUniqueId();
    this.logger = logger;
  }

  /**
   * Initializes the ROS provider
   *
   * @return {Promise} True is a connection with RosBridge succeeded
   */
  public init = async () => {
    return new Promise<boolean>((resolve, reject) => {
      try {
        const rosClient = new ROSLIB.Ros({
          url: this.url,
          transportLibrary: "websocket",
        });

        rosClient.on("connection", () => {
          this.rosClient = rosClient;
          this.initialized = true;
          this.connected = true;
          resolve(true);
        });

        rosClient.on("error", (error: Error) => {
          this.initialized = false;
          this.connected = false;
          if (this.logger) {
            this.logger.error("RosBridge error (init on error)", JSON.stringify(error));
          }
          reject(error);
        });

        rosClient.on("close", () => {
          rosClient.close(); // ensure the underlying worker is cleaned up
          delete this.rosClient;
          this.initialized = false;
          this.connected = false;

          if (this.logger) {
            this.logger.error(`Provider [${this.name}]: connection closed`, "");
          }

          resolve(false);
        });
      } catch (error) {
        this.initialized = false;
        this.connected = false;

        if (this.logger) {
          this.logger.error(`Provider [${this.name}]: catch error`, JSON.stringify(error));
        }

        reject(error);
      }
    });
  };

  /**
   * Close the connection with RosBridge if any
   */
  public close = () => {
    if (this.rosClient) this.rosClient.close(); // ensure the underlying worker is cleaned up
    delete this.rosClient;
    this.initialized = false;
  };

  /**
   * Get list of available nodes using the methods [getTopicsAndRawTypes] and [getNodes]
   *
   * @return {Promise} Returns a list of ROS nodes
   */
  public getNodeList: () => Promise<RosNode[]> = async () => {
    const nodeList = new Map<string, RosNode>();
    const topicTypes = new Map<string, string>();

    // get list of topics and types
    try {
      const topicsTypesRaw = await new Promise<{
        topics: string[];
        types: string[];
        typedefs_full_text: string[];
      }>((resolve, reject) => this.rosClient?.getTopicsAndRawTypes(resolve, reject));

      for (let i = 0; i < topicsTypesRaw.topics.length; i += 1) {
        const topic = topicsTypesRaw.topics[i];
        const type = topicsTypesRaw.types[i];
        topicTypes.set(topic, type);
      }
    } catch (error) {
      if (this.logger) {
        this.logger.error(`Provider [${this.name}]: error on getNodeList()`, JSON.stringify(error));
      }
      return Promise.resolve([]);
    }

    return new Promise<RosNode[]>((outerResolve, outerReject) => {
      this.rosClient?.getNodes(
        async (nodes) => {
          /* eslint-disable no-await-in-loop */
          for (let indexNode = 0; indexNode < nodes.length; indexNode += 1) {
            const node = nodes[indexNode];

            await new Promise((innerResolve, innerReject) => {
              this.rosClient?.getNodeDetails(
                node,
                (subscriptions: string[], publications: string[], services: string[]) => {
                  const ns = this.getNamespace(node);
                  const nodeName = node; // node.replace(ns, '').replace('/', '');

                  if (!nodeList.has(node)) nodeList.set(node, new RosNode(node, nodeName, ns));

                  publications.forEach((pub) => {
                    if (!nodeList.get(node)?.publishers.has(pub)) {
                      nodeList.get(node)?.publishers.set(pub, new RosTopic(pub, [topicTypes.get(pub) ?? ""]));
                    }
                  });
                  subscriptions.forEach((sub) => {
                    if (!nodeList.get(node)?.subscribers.has(sub)) {
                      nodeList.get(node)?.subscribers.set(sub, new RosTopic(sub, [topicTypes.get(sub) ?? ""]));
                    }
                  });
                  services.forEach((srv) => {
                    if (!nodeList.get(node)?.services.has(srv)) {
                      nodeList.get(node)?.services.set(srv, new RosTopic(srv, [topicTypes.get(srv) ?? ""]));
                    }
                  });

                  innerResolve(undefined);
                },
                (error: Error) => {
                  innerReject(error);
                }
              );
            });
          }
          /* eslint-enable no-await-in-loop */

          outerResolve(Array.from(nodeList.values()));
        },
        (error) => {
          if (this.logger) {
            this.logger.error(`Provider [${this.name}]: error on getNodeList()`, JSON.stringify(error));
          }
          outerReject(error);
        }
      );
    });
  };

  /**
   * Return the current host, using the ros bridge URL
   *
   * @return {Promise} Returns a list of ROS nodes
   */
  public getSystemUri = async () => {
    return Promise.resolve(`${this.url.replaceAll("ws://", "")}`);
  };

  /**
   * Get the namespace of a given input string
   *
   * @param {string} name - Input string, usually a node name
   * @return {string} Namespace
   */
  public getNamespace = (name: string): string => {
    const arr = name.split("/");
    if (arr.length > 2) {
      return `/${arr[1]}`;
    }
    return "/";
  };

  /**
   * Check if the provider is available
   *
   * @return {boolean} true is available
   */
  public isAvailable: () => boolean = () => {
    return this.initialized && this.connected;
  };
}

export default RosBridgeProvider;
