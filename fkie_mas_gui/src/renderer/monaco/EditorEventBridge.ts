import {
    EVENT_CLOSE_COMPONENT,
    EVENT_EDITOR_SELECT_RANGE,
    eventCloseComponent,
    eventEditorSelectRange,
} from "@/renderer/pages/NodeManager/layout/events";
import { emitCustomEvent } from "react-custom-events";

export function setupEditorWindowBridge() {
  window.editorManager?.onFileRange((editorId, filePath, fileRange, launchArgs) => {
    if (!fileRange) return;

    emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(editorId, filePath, fileRange, launchArgs));
  });

  window.editorManager?.onClose((editorId) => {
    emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(editorId));
  });
}
