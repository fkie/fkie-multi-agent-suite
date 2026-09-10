import { emitCloseComponent, emitEditorSelectRange } from "@/renderer/components/layout/events";

export function setupEditorWindowBridge() {
  window.editorManager?.onFileRange((editorId, filePath, fileRange, launchArgs, selectParameter) => {
    if (!fileRange && !selectParameter) return;

    emitEditorSelectRange({
      editorId: editorId,
      filePath: filePath,
      fileRange: fileRange,
      launchArgs: launchArgs,
      selectParameter: selectParameter,
    });
  });

  window.editorManager?.onClose((editorId) => {
    emitCloseComponent({ id: editorId });
  });
}
