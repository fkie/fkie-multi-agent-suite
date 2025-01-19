import { JSONObject } from "@/types";
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
  warnings: SystemWarning[];

  constructor(id: string, warnings: SystemWarning[]) {
    this.id = id;
    this.warnings = warnings;
  }

  public static fromJson(obj: JSONObject): SystemWarningGroup {
    const id = obj.id ? (obj.id as string) : "";
    const warnings: SystemWarning[] = [];
    (obj.warnings as JSONObject[])?.forEach((item) => {
      const sysWarn: SystemWarning = SystemWarning.fromJson(item);
      warnings.push(sysWarn);
    });
    const result: SystemWarningGroup = new SystemWarningGroup(id, warnings);
    return result;
  }
}
