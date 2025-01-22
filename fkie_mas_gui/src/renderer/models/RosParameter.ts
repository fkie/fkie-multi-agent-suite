import { generateUniqueId } from "../utils";

/**
 * RosParameter models parameters in a ROS system
 */
export default class RosParameter {
  id: string;

  node: string;

  name: string;

  value: string | number | boolean | string[];

  type: string;

  readonly: boolean = false;
  description: string | undefined;
  additional_constraints: string | undefined;
  min: number | undefined;
  max: number | undefined;
  step: number | undefined;

  providerId: string;

  constructor(
    node: string,
    name: string,
    value: string | number | boolean | string[],
    type: string,
    providerId: string
  ) {
    this.id = generateUniqueId();
    this.node = node;
    this.name = name;
    this.value = value;
    this.type = type;
    this.providerId = providerId;
  }
}
