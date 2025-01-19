import { JSONObject } from "@/types";

/**
 * SystemWarning models a warning from system nodes on ROS side.
 */
export default class SystemWarning {
  /**
   * Short warning message.
   */
  msg: string;

  /**
   * Long description.
   */
  details: string;

  /**
   * Note on the possible solution.
   */
  hint: string;

  constructor(msg: string, details: string, hint: string) {
    this.msg = msg;
    this.details = details;
    this.hint = hint;
  }

  public static fromJson(obj: JSONObject): SystemWarning {
    const msg = obj.msg ? (obj.msg as string) : "";
    const details = obj.details ? (obj.details as string) : "";
    const hint = obj.hint ? (obj.hint as string) : "";
    const result: SystemWarning = new SystemWarning(msg, details, hint);
    return result;
  }
}
