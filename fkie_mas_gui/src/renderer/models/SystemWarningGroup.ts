import SystemWarning from "./SystemWarning";

/**
 * SystemWarningGroup models warning with same ID.
 */
class SystemWarningGroup {
  /**
   * ID of the warning group.
   */
  id: string;

  /**
   * List of warnings.
   */
  warnings: [SystemWarning];

  /**
   * Class Constructor
   *
   * @param {string} id - ID of the warning group.
   * @param {[SystemWarning]} warnings - List of warnings.
   */
  constructor(id: string, warnings: [SystemWarning]) {
    this.id = id;
    this.warnings = warnings;
  }

  /**
   * Generates a string representation of this class
   *
   * @return {string} description
   */
  toString: () => string = () => {
    return `${this.id} - count=${this.warnings.length}`;
  };
}

export default SystemWarningGroup;
