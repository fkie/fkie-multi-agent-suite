import {
  EVENT_CLOSE_COMPONENT,
  EVENT_EDITOR_SELECT_RANGE,
  eventCloseComponent,
  eventEditorSelectRange,
} from "@/renderer/pages/NodeManager/layout/events";
import { emitCustomEvent } from "react-custom-events";

export function setupEditorWindowBridge() {
  window.editorManager?.onFileRange((tabId, filePath, fileRange, launchArgs) => {
    if (!fileRange) return;

    emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, filePath, fileRange, launchArgs));
  });

  window.editorManager?.onClose((tabId) => {
    emitCustomEvent(EVENT_CLOSE_COMPONENT, eventCloseComponent(tabId));
  });
}
