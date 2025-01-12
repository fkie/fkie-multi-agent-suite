/* eslint-disable camelcase */
import { TTag } from "@/types";
import { DiagnosticLevel, DiagnosticStatus } from "./Diagnostics";
import LaunchNodeInfo from "./LaunchNodeInfo";
import RosService from "./RosService";
import RosTopic from "./RosTopic";

/**
 * RosNodeStatus Valid state of a ROS node:
 */
export enum RosNodeStatus {
  INACTIVE = "Inactive", // The node was loaded (in a launch file), but it has not been started yet
  RUNNING = "Running", // The node is running in the given host
  NOT_MONITORED = "Not Monitored", // The node runs on a different host and will be not monitored
  DEAD = "Dead", // The node was registered in ROS_MASTER, but it does not respond (probably was killed or it is over-loaded)
  ONLY_SCREEN = "Only screen", // we found only an active screen with correct name
  UNKNOWN = "Unknown", // Unknown status
}

export const RosNodeStatusInfo = {
  [RosNodeStatus.INACTIVE]: "The node was loaded (in a launch file), but it hasn't been started",

  [RosNodeStatus.RUNNING]: "The node is running in the given host",

  [RosNodeStatus.NOT_MONITORED]: "The node runs in a different host and its status will be not monitored",

  [RosNodeStatus.DEAD]:
    "The node was registered in ROS CORE, but it does not respond (probably was killed or it is over-loaded)",

  [RosNodeStatus.UNKNOWN]: "Unknown status",
};

/**
 * RosNode models the nodes running in a ROS system
 */
export default class RosNode {
  /** ID used on the UI side across all provider. idGlobal should be the same for life of the node on remote host */
  idGlobal: string = "";

  tags: TTag[] = []; // used on gui side

  /**
   * Unique identifier
   */
  id: string;

  /**
   * ROS name (including namespace)
   */
  name: string;

  /**
   * Node namespace
   */
  namespace: string;

  /**
   * ROS node API URI (where the node is running)
   */
  node_API_URI: string;

  /**
   * status: check [RosNodeStatus]
   */
  status: RosNodeStatus;

  /**
   * Process id
   */
  pid: number;

  /**
   * The ROS_MASTER_URI the node was originally registered
   */
  masteruri: string;

  /**
   * Describes whether the node is running on the same host as the ROS master. Possible values: local, remote
   */
  location: string | string[];

  /**
   * (optional) name of the provider that returned the node
   */
  providerName: string;

  /**
   * (optional) id of the provider that returned the node
   */
  providerId: string;

  /**
   * (optional) path of the launch file if available
   */
  launchPath: string;

  /**
   * List of topics subscribed by the node
   */
  subscribers: Map<string, RosTopic>;

  /**
   * List of topics published by the node
   */
  publishers: Map<string, RosTopic>;

  /**
   * List of services available in the node
   */
  services: Map<string, RosService>;

  /**
   * List of Active screens
   */
  screens: string[];

  /** List of diagnostic messages associated with this node. */
  diagnosticStatus: DiagnosticStatus[] = [];

  diagnosticLevel: DiagnosticLevel = DiagnosticLevel.OK;

  diagnosticMessage: string = "";

  /**
   * Flag to signal system node
   */
  system_node: boolean;

  /**
   * Map of launch files and list of parameters
   */
  // parameters: Map<string, RosParameter[]>;

  /**
   * group used for visualization
   */
  capabilityGroup: { namespace?: string; name?: string } = {};
  group: string = "";

  /**
   * Map of launch files and launch info
   */
  launchInfo: Map<string, LaunchNodeInfo> = new Map();

  /** Used to store by user changed logger configuration.
   * type: { name: string; level: string }
   */
  rosLoggers: { [id: string]: string } = {};

  guid?: string;

  /**
   * True if the node is a container
   */

  is_container?: boolean;
  /**
   * The node name of the composable/nodelet's parent
   */
  container_name?: string;

  dynamicReconfigureServices: string[] = [];

  lifecycle_state?: string;
  lifecycle_available_transitions?: { label: string; id: number }[];

  constructor(
    id = "",
    name = "",
    namespace = "",
    node_API_URI = "",
    status = RosNodeStatus.UNKNOWN,
    pid = -1,
    masteruri = "",
    location = "unknown",
    system_node = false,
    subscribers = new Map<string, RosTopic>(),
    publishers = new Map<string, RosTopic>(),
    services = new Map<string, RosService>(),
    screens = [],
    launchPath = ""
  ) {
    this.id = id;
    this.name = name;
    this.namespace = namespace;
    this.node_API_URI = node_API_URI;
    this.status = status;
    this.pid = pid;
    this.masteruri = masteruri;
    this.location = location;
    this.system_node = system_node;
    this.subscribers = subscribers;
    this.publishers = publishers;
    this.services = services;
    this.screens = screens;
    this.launchPath = launchPath;
    this.providerName = "";
    this.providerId = "";
  }

  public getRosLoggersCount(): number {
    return Object.keys(this.rosLoggers).length;
  }

  public getLaunchComposableContainer(): string | null {
    if (this.launchPath) {
      const launchInfo = this.launchInfo.get(this.launchPath);
      if (launchInfo?.composable_container) {
        return launchInfo?.composable_container;
      }
    } else if (this.launchInfo.size === 1) {
      const launchInfo = this.launchInfo.values().next().value;
      if (launchInfo?.composable_container) {
        return launchInfo?.composable_container;
      }
    }
    return null;
  }

  public getAllContainers(): string[] {
    const result: string[] = [];
    if (this.container_name) {
      result.push(this.container_name);
    }
    this.launchInfo.forEach((launchInfo) => {
      if (launchInfo.composable_container && !result.includes(launchInfo.composable_container)) {
        result.push(launchInfo.composable_container);
      }
    });
    return result;
  }
}
