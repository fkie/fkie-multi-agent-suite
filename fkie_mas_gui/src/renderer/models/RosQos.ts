/* eslint-disable camelcase */
import RosDuration from "./RosDuration";

export const RELIABILITY = {
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
export const HISTORY = {
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
export const DURABILITY = {
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
export const LIVELINESS = {
  // Implementation specific default
  SYSTEM_DEFAULT: 0,
  // The signal that establishes a Topic is alive comes from the ROS rmw layer.
  AUTOMATIC: 1,
  // :deprecated: Explicitly asserting node liveliness is required in this case.
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
export default class RosQos {
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

  liveliness_lease_duration: RosDuration;

  lifespan: RosDuration;

  constructor(
    durability: number = DURABILITY.VOLATILE,
    history: number = HISTORY.KEEP_LAST,
    depth = 10,
    liveliness: number = LIVELINESS.SYSTEM_DEFAULT,
    reliability: number = RELIABILITY.RELIABLE,
    deadline: RosDuration = new RosDuration(),
    liveliness_lease_duration: RosDuration = new RosDuration(),
    lifespan: RosDuration = new RosDuration()
  ) {
    this.durability = durability;
    this.history = history;
    this.depth = depth;
    this.liveliness = liveliness;
    this.reliability = reliability;
    this.deadline = deadline;
    this.liveliness_lease_duration = liveliness_lease_duration;
    this.lifespan = lifespan;
  }

  toString(): string {
    let result = "";
    if (this.depth !== 10) {
      result += `--qos-depth ${this.depth} `;
    }
    if (this.durability < DURABILITY.UNKNOWN) {
      result += `--qos-durability ${durabilityToString(this.durability)} `;
    }
    if (this.reliability < RELIABILITY.UNKNOWN) {
      result += `--qos-reliability ${reliabilityToString(this.reliability)} `;
    }
    if (this.liveliness < LIVELINESS.UNKNOWN) {
      result += `--qos-liveliness ${livelinessToString(this.liveliness)} `;
    }
    if (this.history < HISTORY.UNKNOWN) {
      result += `--qos-history ${historyToString(this.history)} `;
    }
    return result;
  }
}

export function qosFromJson(obj: RosQos | undefined): RosQos {
  if (obj) {
    return new RosQos(
      obj.durability | DURABILITY.VOLATILE,
      obj.history | HISTORY.KEEP_LAST,
      obj.depth | 10,
      obj.liveliness | LIVELINESS.SYSTEM_DEFAULT,
      obj.reliability | RELIABILITY.RELIABLE
    );
  }
  return new RosQos();
}

export function reliabilityToString(value: number): string {
  switch (value) {
    case RELIABILITY.SYSTEM_DEFAULT:
      return "system_default";
    case RELIABILITY.RELIABLE:
      return "reliable";
    case RELIABILITY.BEST_EFFORT:
      return "best_effort";
  }
  return "unknown";
}

export function durabilityToString(value: number): string {
  switch (value) {
    case DURABILITY.SYSTEM_DEFAULT:
      return "system_default";
    case DURABILITY.TRANSIENT_LOCAL:
      return "transient_local";
    case DURABILITY.VOLATILE:
      return "volatile";
  }
  return "unknown";
}

export function livelinessToString(value: number): string {
  switch (value) {
    case LIVELINESS.SYSTEM_DEFAULT:
      return "system_default";
    case LIVELINESS.AUTOMATIC:
      return "automatic";
    case LIVELINESS.MANUAL_BY_NODE:
      return "manual_by_node";
    case LIVELINESS.MANUAL_BY_TOPIC:
      return "manual_by_topic";
  }
  return "unknown";
}

export function historyToString(value: number): string {
  switch (value) {
    case HISTORY.SYSTEM_DEFAULT:
      return "system_default";
    case HISTORY.KEEP_LAST:
      return "keep_last";
    case HISTORY.KEEP_ALL:
      return "keep_all";
  }
  return "unknown";
}
