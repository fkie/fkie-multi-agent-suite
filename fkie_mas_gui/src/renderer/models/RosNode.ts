/* eslint-disable camelcase */
import { DiagnosticLevel, DiagnosticStatus } from './Diagnostics';
import LaunchNodeInfo from './LaunchNodeInfo';
import RosService from './RosService';
import RosTopic from './RosTopic';

/**
 * RosNodeStatus Valid state of a ROS node:
 */
export enum RosNodeStatus {
  INACTIVE = 'Inactive', // The node was loaded (in a launch file), but it has not been started yet
  RUNNING = 'Running', // The node is running in the given host
  NOT_MONITORED = 'Not Monitored', // The node runs on a different host and will be not monitored
  DEAD = 'Dead', // The node was registered in ROS_MASTER, but it does not respond (probably was killed or it is over-loaded)
  UNKNOWN = 'Unknown', // Unknown status
}

export const RosNodeStatusInfo = {
  [RosNodeStatus.INACTIVE]:
    "The node was loaded (in a launch file), but it hasn't been started",

  [RosNodeStatus.RUNNING]: 'The node is running in the given host',

  [RosNodeStatus.NOT_MONITORED]:
    'The node runs in a different host and its status will be not monitored',

  [RosNodeStatus.DEAD]:
    'The node was registered in ROS CORE, but it does not respond (probably was killed or it is over-loaded)',

  [RosNodeStatus.UNKNOWN]: 'Unknown status',
};

/**
 * RosNode models the nodes running in a ROS system
 */
class RosNode {
  /** ID used on the UI side across all provider. idGlobal should be the same for life of the node on remote host */
  idGlobal: string = '';

  tags: any[] = []; // used on gui side

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
  location: string | any;

  /**
   * (optional) name of the provider that returned the node
   */
  providerName?: string;

  /**
   * (optional) id of the provider that returned the node
   */
  providerId?: string;

  /**
   * path of loaded the launch files
   */
  launchPaths: Set<string>;

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

  diagnosticMessage: string = '';

  /**
   * Flag to signal system node
   */
  system_node: boolean;

  /**
   * List of parameter and values
   */
  parameters?: Map<string, string | number | boolean | string[]>;

  /**
   * group used for visualization
   */
  group?: string;

  /**
   * Info given on launch file
   */
  launchInfo?: LaunchNodeInfo;

  associations: string[] = [];

  /**
   * ID of the composable/nodelet's parent
   */
  parent_id?: string;

  /**
   * Class Constructor
   *
   * @param {string} id - Unique identifier
   * @param {string} name - ROS name (including namespace)
   * @param {string} namespace - ROS namespace
   * @param {string} node_API_URI - ROS node API URI (where the node is running)
   * @param {RosNodeStatus} status - Node status: inactive (loaded but not stated), running (already started), unknown
   * @param {number} pid - process id
   * @param {string} masteruri - The ROS_MASTER_URI the node was originally registered
   * @param {string} location - Describes whether the node is running on the same host as the ROS master. Possible values: local, remote
   * @param {Map<string, RosTopic>} subscribers - List of topics subscribed by the node
   * @param {Map<string, RosTopic>} publishers - List of topics published by the node
   * @param {Map<string, RosTopic>} actions - List of actions registered by the node
   * @param {Map<string, RosService>} services - List of services available in the node
   * @param {string[]} screens - List of screens associated to the node
   * @param {Set<string>} launchPaths - A unique list of launch files associated to the node
   * @param {boolean} system_node - Flag to signal a system's relevant node
   *
   */
  constructor(
    id = '',
    name = '',
    namespace = '',
    node_API_URI = '',
    status = RosNodeStatus.UNKNOWN,
    pid = -1,
    masteruri = '',
    location = 'local',
    system_node = false,
    subscribers = new Map<string, RosTopic>(),
    publishers = new Map<string, RosTopic>(),
    services = new Map<string, RosService>(),
    screens = [],
    launchPaths = new Set<string>(),
    launchPath = '',
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
    this.launchPaths = launchPaths;
    this.launchPath = launchPath;
  }

  /**
   * Generates a string representation of the node
   *
   * @return {string} Node description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.namespace} - ${this.status}`;
  };
}

export const compareRosNodes = (a: RosNode, b: RosNode) => {
  const aValue = a.group + a.name.replace(a.namespace, '');
  const bValue = b.group + b.name.replace(a.namespace, '');
  if (aValue < bValue) {
    return -1;
  }
  if (aValue > bValue) {
    return 1;
  }
  return 0;
};

export default RosNode;
