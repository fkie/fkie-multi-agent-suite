import { Monaco } from "@monaco-editor/react";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";
import { IconButton, Paper, Stack, Tooltip } from "@mui/material";
import { IDisposable, editor as monacoEditor } from "monaco-editor";
import { JSX, MutableRefObject, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createPortal } from "react-dom";

import { TParameterInsert } from "@/renderer/monaco/ParameterEditing";
import { TParameterRequest } from "@/types";

const WIDGET_ID = "pending.parameter.edit.widget";
/** maximum undo steps used to restore the state before the insert */
const MAX_UNDO_STEPS = 100;

const SHOW_CONFIRM_BUTTONS = false;

type TPending = {
  request: TParameterRequest;
  /** text which was replaced by the insert - used as revert fallback */
  originalText: string;
  /** model decoration ids, index 0 tracks the inserted text */
  decorationIds: string[];
  /** model of the pending edit - kept to allow a revert after a model switch */
  model: monacoEditor.ITextModel;
  /** alternative version id before the insert - restores the dirty state on undo */
  versionIdBeforeInsert: number;
  /** true if the user changed something outside of the inserted range */
  editedOutside: boolean;
};

export type TUsePendingParameterEdit = {
  /** true while an unconfirmed insert exists */
  hasPendingEdit: boolean;
  /** insert the parameter text and mark it as pending */
  startPendingEdit: (request: TParameterRequest, insert: TParameterInsert) => void;
  /** confirm the pending insert (keeps the text) */
  acceptPendingEdit: () => void;
  /** revert the pending insert (restores the original text and the dirty state) */
  rejectPendingEdit: () => void;
  /** drop decorations and widget, but keep the inserted text (e.g. on save) */
  clearPendingState: () => void;
  /** portal with the confirm/discard buttons - must be rendered by the owner component */
  pendingEditWidget: JSX.Element | null;
};

export function usePendingParameterEdit(
  editorRef: MutableRefObject<monacoEditor.IStandaloneCodeEditor | undefined>,
  monaco: Monaco | null,
  /** called after the insert was confirmed */
  onAccepted?: (request: TParameterRequest) => void,
  /** called after a revert; restored=false means the dirty state must be re-evaluated */
  onReverted?: (model: monacoEditor.ITextModel, restored: boolean) => void
): TUsePendingParameterEdit {
  const [hasPendingEdit, setHasPendingEdit] = useState<boolean>(false);
  const pendingRef = useRef<TPending | null>(null);
  const widgetVisibleRef = useRef<boolean>(false);

  // stable refs to allow cross-references between the callbacks
  const acceptRef = useRef<() => void>(() => {});
  const rejectRef = useRef<() => void>(() => {});

  /** stable dom node used as portal target for the MUI buttons */
  const domNode = useMemo<HTMLDivElement>(() => {
    const node = document.createElement("div");
    node.className = "pending-edit-widget";
    // pull the widget slightly to the left edge and above the line
    node.style.transform = "translate(-20px, 0)";
    node.style.zIndex = "30";
    return node;
  }, []);

  const contentWidget = useMemo<monacoEditor.IContentWidget>(
    () => ({
      getId: () => WIDGET_ID,
      getDomNode: () => domNode,
      getPosition: () => {
        const pending = pendingRef.current;
        if (!pending || !monaco || pending.model.isDisposed()) return null;
        const range = pending.model.getDecorationRange(pending.decorationIds[0]);
        if (!range) return null;
        return {
          // column 1 -> widget is anchored at the beginning of the line, not at the insert column
          position: { lineNumber: range.startLineNumber, column: 1 },
          preference: [
            monaco.editor.ContentWidgetPositionPreference.ABOVE,
            monaco.editor.ContentWidgetPositionPreference.BELOW,
          ],
        };
      },
      // keep the buttons clickable without moving the editor cursor
      suppressMouseDown: true,
      allowEditorOverflow: true,
    }),
    [domNode, monaco]
  );

  const showWidget = useCallback((): void => {
    const editorInstance = editorRef.current;
    if (!editorInstance) return;
    if (widgetVisibleRef.current) {
      editorInstance.layoutContentWidget(contentWidget);
      return;
    }
    editorInstance.addContentWidget(contentWidget);
    widgetVisibleRef.current = true;
  }, [editorRef, contentWidget]);

  const hideWidget = useCallback((): void => {
    if (!widgetVisibleRef.current) return;
    const editorInstance = editorRef.current;
    if (editorInstance && !editorInstance.getModel()?.isDisposed()) {
      editorInstance.removeContentWidget(contentWidget);
    }
    widgetVisibleRef.current = false;
  }, [editorRef, contentWidget]);

  /** remove widget and decorations, keep the text */
  const clearPendingState = useCallback((): void => {
    hideWidget();
    const pending = pendingRef.current;
    if (pending && !pending.model.isDisposed()) {
      pending.model.deltaDecorations(pending.decorationIds, []);
    }
    pendingRef.current = null;
    setHasPendingEdit(false);
  }, [hideWidget]);

  const acceptPendingEdit = useCallback((): void => {
    const pending = pendingRef.current;
    if (!pending) return;
    const request = pending.request;
    clearPendingState();
    onAccepted?.(request);
  }, [clearPendingState, onAccepted]);

  /** revert the insert - works even if the editor shows another model */
  const rejectPendingEdit = useCallback((): void => {
    const pending = pendingRef.current;
    if (!pending) return;
    const model = pending.model;

    if (!model || model.isDisposed()) {
      clearPendingState();
      return;
    }

    let restored = false;
    if (!pending.editedOutside) {
      // undo restores content AND alternativeVersionId -> the dirty flag is reset correctly
      let guard = 0;
      while (model.getAlternativeVersionId() !== pending.versionIdBeforeInsert && guard < MAX_UNDO_STEPS) {
        model.undo();
        guard += 1;
      }
      restored = model.getAlternativeVersionId() === pending.versionIdBeforeInsert;
    }
    if (!restored) {
      // fallback: write back the original text (file stays marked as modified)
      const currentRange = model.getDecorationRange(pending.decorationIds[0]);
      if (currentRange) {
        model.pushEditOperations([], [{ range: currentRange, text: pending.originalText }], () => null);
      }
    }
    onReverted?.(model, restored);
    clearPendingState();
  }, [clearPendingState, onReverted]);

  acceptRef.current = acceptPendingEdit;
  rejectRef.current = rejectPendingEdit;

  const startPendingEdit = useCallback(
    (request: TParameterRequest, insert: TParameterInsert): void => {
      const editorInstance = editorRef.current;
      const model = editorInstance?.getModel();
      if (!editorInstance || !model || !monaco) return;

      // only one pending edit at a time
      if (pendingRef.current) rejectRef.current();

      const r = insert.range;
      const range = new monaco.Range(r.startLineNumber, r.startColumn, r.endLineNumber, r.endColumn);
      const originalText = model.getValueInRange(range);
      const startOffset = model.getOffsetAt(range.getStartPosition());
      const versionIdBeforeInsert = model.getAlternativeVersionId();

      // separate undo entry, so a single undo reverts exactly this insert
      editorInstance.pushUndoStop();
      editorInstance.executeEdits("pending-parameter-insert", [{ range, text: insert.text, forceMoveMarkers: true }]);
      editorInstance.pushUndoStop();

      const endPosition = model.getPositionAt(startOffset + insert.text.length);
      const insertedRange = new monaco.Range(
        range.startLineNumber,
        range.startColumn,
        endPosition.lineNumber,
        endPosition.column
      );

      editorInstance.setSelection(insertedRange);
      editorInstance.revealRangeInCenterIfOutsideViewport(insertedRange);
      editorInstance.focus();

      if (!SHOW_CONFIRM_BUTTONS) {
        // auto accept: keep the inserted text, no decorations / widget / pending state
        pendingRef.current = null;
        setHasPendingEdit(false);
        onAccepted?.(request);
        return;
      }

      // model decorations survive a model switch of the editor
      const decorationIds = model.deltaDecorations(
        [],
        [
          {
            range: insertedRange,
            options: {
              className: "pending-edit-text",
              stickiness: monaco.editor.TrackedRangeStickiness.GrowsOnlyWhenTypingAfter,
              hoverMessage: [
                { value: `**pending insert** for \`${request.paramName}\`` },
                { value: "Alt+Enter: apply — Alt+Backspace: discard" },
              ],
              minimap: { color: "#4caf50", position: monaco.editor.MinimapPosition.Inline },
              overviewRuler: { color: "#4caf50", position: monaco.editor.OverviewRulerLane.Left },
            },
          },
          {
            range: insertedRange,
            options: { isWholeLine: true, className: "pending-edit-line" },
          },
        ]
      );

      pendingRef.current = {
        request,
        originalText,
        decorationIds,
        model,
        versionIdBeforeInsert,
        editedOutside: false,
      };
      setHasPendingEdit(true);
      showWidget();
    },
    [editorRef, monaco, showWidget, onAccepted]
  );

  // the portal content is rendered after addContentWidget -> re-measure the widget
  useEffect(() => {
    if (!hasPendingEdit) return;
    const editorInstance = editorRef.current;
    if (!editorInstance || !widgetVisibleRef.current) return;
    // next frame: MUI buttons are in the dom now
    const raf = requestAnimationFrame(() => editorInstance.layoutContentWidget(contentWidget));
    return (): void => cancelAnimationFrame(raf);
  }, [hasPendingEdit, editorRef, contentWidget]);

  // attach editor listeners as soon as the editor instance is available
  useEffect(() => {
    if (!monaco) return;
    let cancelled = false;
    let retryTimer: ReturnType<typeof setTimeout> | undefined;
    const disposables: IDisposable[] = [];

    const attach = (): void => {
      if (cancelled) return;
      const editorInstance = editorRef.current;
      if (!editorInstance) {
        // editor is mounted asynchronously by the owner component
        retryTimer = setTimeout(attach, 50);
        return;
      }

      disposables.push(
        editorInstance.onDidChangeModelContent((event) => {
          const pending = pendingRef.current;
          if (!pending || pending.model.isDisposed()) return;
          const tracked = pending.model.getDecorationRange(pending.decorationIds[0]);
          if (!tracked) {
            // inserted text was removed manually -> nothing left to confirm
            clearPendingState();
            return;
          }
          if (!event.isUndoing && !event.isRedoing) {
            const outside = event.changes.some((c) => !monaco.Range.areIntersectingOrTouching(tracked, c.range));
            if (outside) pending.editedOutside = true;
          }
          // keep the widget aligned with the tracked range
          if (widgetVisibleRef.current) editorInstance.layoutContentWidget(contentWidget);
        })
      );

      // safety net: model switched without an explicit decision -> revert
      disposables.push(
        editorInstance.onDidChangeModel(() => {
          const pending = pendingRef.current;
          if (!pending) return;
          if (editorInstance.getModel()?.uri.toString() !== pending.model.uri.toString()) {
            rejectRef.current();
          }
        })
      );

      disposables.push(
        editorInstance.onKeyDown((event) => {
          if (!pendingRef.current || !event.altKey) return;
          if (event.keyCode === monaco.KeyCode.Enter) {
            event.preventDefault();
            event.stopPropagation();
            acceptRef.current();
          } else if (event.keyCode === monaco.KeyCode.Backspace) {
            event.preventDefault();
            event.stopPropagation();
            rejectRef.current();
          }
        })
      );
    };

    attach();

    return (): void => {
      cancelled = true;
      if (retryTimer) clearTimeout(retryTimer);
      for (const d of disposables) {
        d.dispose();
      }
      // discard an unconfirmed insert on unmount
      rejectRef.current();
    };
  }, [editorRef, monaco, contentWidget, clearPendingState]);

  const pendingEditWidget = hasPendingEdit
    ? createPortal(
        <Paper elevation={4} sx={{ display: "inline-flex", borderRadius: 1, px: 0.25 }}>
          <Stack direction="row" spacing={0}>
            <Tooltip title="Apply insert (Alt+Enter)" placement="top" disableInteractive>
              <IconButton
                size="small"
                color="success"
                onMouseDown={(e) => e.preventDefault()}
                onClick={acceptPendingEdit}
              >
                <CheckIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
            <Tooltip title="Discard insert (Alt+Backspace)" placement="top" disableInteractive>
              <IconButton
                size="small"
                color="error"
                onMouseDown={(e) => e.preventDefault()}
                onClick={rejectPendingEdit}
              >
                <CloseIcon fontSize="inherit" />
              </IconButton>
            </Tooltip>
          </Stack>
        </Paper>,
        domNode
      )
    : null;

  return {
    hasPendingEdit,
    startPendingEdit,
    acceptPendingEdit,
    rejectPendingEdit,
    clearPendingState,
    pendingEditWidget,
  };
}
