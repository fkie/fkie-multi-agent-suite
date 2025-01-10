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

export default class PathEvent {
  id: string;

  eventType: PATH_EVENT_TYPE;

  srcPath: string;

  affected: string[];

  constructor(eventType: PATH_EVENT_TYPE, srcPath: string, affected: string[]) {
    this.id = generateUniqueId();
    this.eventType = eventType;
    this.srcPath = srcPath;
    this.affected = affected;
  }
}
