/**
 * RosParameter models parameters in a ROS system
 */

import { JSONValue } from "@/types";

export type RosParameterValue = string | number | boolean | string[] | JSONValue;

export class RosParameterRange {
  from_value: number | undefined;
  to_value: number | undefined;
  step: number | undefined;

  constructor(from_value: number, to_value: number, step: number) {
    this.from_value = from_value;
    this.to_value = to_value;
    this.step = step;
  }
}

export default class RosParameter {
  id: string;

  node: string;

  name: string;

  value: RosParameterValue;

  type: string;

  // ros2 parameter description
  readonly: boolean | undefined = false;
  description: string | undefined;
  additional_constraints: string | undefined;
  floating_point_range: RosParameterRange[] | undefined;
  integer_range: RosParameterRange[] | undefined;

  providerId: string;

  constructor(node: string, name: string, value: RosParameterValue, type: string, providerId: string) {
    this.id = `${node}#${name}`;
    this.node = node;
    this.name = name;
    this.value = value;
    this.type = type;
    this.providerId = providerId;
  }
}
