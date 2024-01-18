/* eslint-disable camelcase */
import RosDuration from './RosDuration';

const RELIABILITY = {
  // Implementation specific default
  SYSTEM_DEFAULT: 0,
  // Guarantee that samples are delivered, may retry multiple times.
  RELIABLE: 1,
  // Attempt to deliver samples, but some may be lost if the network is not robust
  BEST_EFFORT: 2,
  // Reliability policy has not yet been set
  UNKNOWN: 3,
};

// QoS history enumerations describing how samples endure
const HISTORY = {
  // Implementation default for history policy
  SYSTEM_DEFAULT: 0,
  // Only store up to a maximum number of samples, dropping oldest once max is exceeded
  KEEP_LAST: 1,
  // Store all samples, subject to resource limits
  KEEP_ALL: 2,
  // History policy has not yet been set
  UNKNOWN: 3,
};

// QoS durability enumerations describing how samples persist
const DURABILITY = {
  // Implementation specific default
  SYSTEM_DEFAULT: 0,
  // The rmw publisher is responsible for persisting samples for “late-joining” subscribers
  TRANSIENT_LOCAL: 1,
  // Samples are not persistent
  VOLATILE: 2,
  // Durability policy has not yet been set
  UNKNOWN: 3,
};

// QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
// For a subscriber, these are its requirements for its topic's publishers.
const LIVELINESS = {
  // Implementation specific default
  SYSTEM_DEFAULT: 0,
  // The signal that establishes a Topic is alive comes from the ROS rmw layer.
  AUTOMATIC: 1,
  // :depricated: Explicitly asserting node liveliness is required in this case.
  MANUAL_BY_NODE: 2,
  // The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
  // on the Topic or an explicit signal from the application to assert liveliness on the Topic
  // will mark the Topic as being alive.
  // Using `3` for backwards compatibility.
  MANUAL_BY_TOPIC: 3,
  // Durability policy has not yet been set
  UNKNOWN: 4,
};

/**
 * Quality of service settings for a ros topic.
 */
class RosQos {
  /**
   * QoS durability enumerations describing how samples persist
   */
  durability: number;

  /**
   * QoS history enumerations describing how samples endure
   */
  history: number;

  /**
   * Queue size
   */
  depth: number;

  /**
   * QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
   */
  liveliness: number;

  /**
   * QoS reliability enumerations the guaranties.
   */
  reliability: number;

  deadline: RosDuration;

  lease_duration: RosDuration;

  lifespan: RosDuration;

  /**
   * Class Constructor
   *
   * @param {number} durability - QoS durability enumerations describing how samples persist
   * @param {number} history - QoS history enumerations describing how samples endure
   * @param {number} depth - Queue size
   * @param {number} liveliness - QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
   * @param {number} reliability - QoS reliability enumerations the guaranties.
   * @param {RosDuration} deadline - .
   * @param {RosDuration} lease_duration - .
   * @param {RosDuration} lifespan - .
   */
  constructor(
    durability: number = DURABILITY.VOLATILE,
    history: number = HISTORY.KEEP_LAST,
    depth = 10,
    liveliness: number = LIVELINESS.SYSTEM_DEFAULT,
    reliability: number = RELIABILITY.RELIABLE,
    deadline: RosDuration = new RosDuration(),
    lease_duration: RosDuration = new RosDuration(),
    lifespan: RosDuration = new RosDuration(),
  ) {
    this.durability = durability;
    this.history = history;
    this.depth = depth;
    this.liveliness = liveliness;
    this.reliability = reliability;
    this.deadline = deadline;
    this.lease_duration = lease_duration;
    this.lifespan = lifespan;
  }
}

export default RosQos;

export { DURABILITY, HISTORY, LIVELINESS, RELIABILITY };
