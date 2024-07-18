/* eslint-disable camelcase */
/**
 * LaunchArgument models arguments for launch files.
 */
class LaunchArgument {
  /**
   * Name of the argument.
   */
  name: string;

  /**
   * Value
   */
  value: string;

  default_value: any;

  description: string;

  choices: string[];

  /**
   * Class Constructor
   *
   * @param {string} name - argument name.
   * @param {string} value - argument value.
   * @param {any} default_value - default argument value.
   * @param {string} description - argument description.
   * @param {string[]} choices - possible values.
   */
  constructor(
    name: string,
    value: string,
    default_value: any = undefined,
    description: string = "",
    choices: string[] = []
  ) {
    this.name = name;
    this.value = value;
    this.default_value = default_value;
    this.description = description;
    this.choices = choices;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.name} - ${this.value}`;
  };
}

export default LaunchArgument;
