/* eslint-disable max-classes-per-file */
import { generateUniqueId } from "../utils";

/**
 * PathEvent models path events
 */

export class PATH_EVENT_TYPE extends String {
  static TYPES = {
    MOVED: "moved",
    DELETED: "deleted",
    CREATED: "created",
    MODIFIED: "modified",
  };
}

class PathEvent {
  id: string;

  eventType: PATH_EVENT_TYPE;

  srcPath: string;

  affected: string[];

  /**
   * Class Constructor
   *
   * @param {PATH_EVENT_TYPE} eventType - Event type.
   * @param {string} srcPath - Changes of event type are made on this path.
   * @param {string[]} affected - List of all launch files which include changed file.
   */
  constructor(eventType: PATH_EVENT_TYPE, srcPath: string, affected: string[]) {
    this.id = generateUniqueId();
    this.eventType = eventType;
    this.srcPath = srcPath;
    this.affected = affected;
  }
}

export default PathEvent;
