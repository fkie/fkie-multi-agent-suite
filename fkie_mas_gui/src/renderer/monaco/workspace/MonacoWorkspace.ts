import * as MonacoReact from "@monaco-editor/react";

import { IRosContext } from "@/renderer/context/RosContext";
import { FileOpenService } from "./FileOpenService";
import { FileSaveService } from "./FileSaveService";
import { MasFileProvider } from "./MasFileProvider";
import { ModelRegistry } from "./ModelRegistry";
import { MonacoDirtyManager } from "./MonacoDirtyManager";

export class MonacoWorkspace {
  models: ModelRegistry;
  dirty: MonacoDirtyManager;
  opener: FileOpenService;
  saver: FileSaveService;

  constructor(
    monaco: MonacoReact.Monaco,
    private rosCtxRef: React.MutableRefObject<IRosContext>,
  ) {
    const fs = new MasFileProvider(this.rosCtxRef);

    this.dirty = new MonacoDirtyManager(monaco);

    this.models = new ModelRegistry(monaco);

    this.opener = new FileOpenService(fs);

    this.saver = new FileSaveService(fs, this.dirty);
  }
}
