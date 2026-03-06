import * as monaco from "monaco-editor";

import { FileItem } from "@/renderer/models";
import { SaveResult } from "../types";
import { fileFromUriPath } from "../utils";
import { MasFileProvider } from "./MasFileProvider";
import { MonacoDirtyManager } from "./MonacoDirtyManager";

export class FileSaveService {
  constructor(
    private fs: MasFileProvider,
    private dirty: MonacoDirtyManager
  ) {}

  async save(model: monaco.editor.ITextModel, providerId: string): Promise<SaveResult> {
    const path = fileFromUriPath(model.uri.path);

    const file = new FileItem("", path, "", "", false, model.getValue());

    const result = await this.fs.writeFile(providerId, file);

    const saveItem: SaveResult = { uriPath: model.uri.path, result: false, message: "" };
    if (result.bytesWritten > 0) {
      this.dirty.markSaved(model);
      saveItem.result = true;
    } else {
      saveItem.result = true;
      saveItem.message = result.error || "Unknown save error";
    }
    return saveItem;
  }
}
