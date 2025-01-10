import { TLaunchArg } from "@/types";

/**
 * LaunchArgument models arguments for launch files.
 */
export default class LaunchArgument implements TLaunchArg {
  /**
   * Name of the argument.
   */
  name: string;

  /**
   * Value
   */
  value: string;

  default_value?: string;

  description?: string;

  choices?: string[];

  constructor(name: string, value: string, default_value?: string, description?: string, choices?: string[]) {
    this.name = name;
    this.value = value;
    this.default_value = default_value;
    this.description = description;
    this.choices = choices;
  }
}
