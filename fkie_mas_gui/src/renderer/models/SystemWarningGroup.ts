import SystemWarning from "./SystemWarning";

/**
 * SystemWarningGroup models warning with same ID.
 */
export default class SystemWarningGroup {
  /**
   * ID of the warning group.
   */
  id: string;

  /**
   * List of warnings.
   */
  warnings: [SystemWarning];

  constructor(id: string, warnings: [SystemWarning]) {
    this.id = id;
    this.warnings = warnings;
  }
}
