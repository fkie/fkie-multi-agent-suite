import { JSONObject } from "@/types";

/**
 * ScreensMapping models multiple screen of a node.
 */
export default class ScreensMapping {
  /**
   * Name of the node.
   */
  name: string;

  /**
   * A list of screens detected for a node.
   */
  screens: string[];

  constructor(name: string, screens: string[]) {
    this.name = name;
    this.screens = screens;
  }

  public static fromJson(obj: JSONObject): ScreensMapping {
    const name = obj.name ? (obj.name as string) : "";
    const screens: string[] = [];
    (obj.screens as string[])?.forEach((item) => {
      screens.push(item);
    });
    const result: ScreensMapping = new ScreensMapping(name, screens);
    return result;
  }
}
