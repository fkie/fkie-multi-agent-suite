import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from "@mui/material";
import { Actions, Model } from "flexlayout-react";
import React, { createContext, useCallback, useContext, useMemo, useState } from "react";

import DraggablePaper from "@/renderer/components/UI/DraggablePaper";
import { useMonacoContext } from "@/renderer/hooks/useMonacoContext";
import { getBaseName, getFileName } from "@/renderer/models";
import { SaveResult } from "@/renderer/monaco/types";
import { isEditorEditorId } from "@/renderer/monaco/utils";
import { LAYOUT_TABS } from "../components/layout";
import { deleteTabAndSelectNodes } from "../components/layout/LayoutTargets";

type TCloseRequest = { editorId: string; model: Model; close?: (tabId: string) => void };

type TDirtyEditorGuardContext = {
  /**
   * Checks whether the tab can be closed immediately.
   * Returns false if the tab is a modified editor -> a dialog is shown instead.
   */
  guardTabClose: (model: Model, tabId: string, close?: (tabId: string) => void) => boolean;
  /** Ask the user for a list of editor ids (e.g. on app shutdown). */
  requestCloseEditors: (model: Model, editorIds: string[], close?: (tabId: string) => void) => void;
  /** Called when the user cancels the dialog (e.g. to abort app close). */
  onCancel?: () => void;
  hasPending: boolean;
};

const DirtyEditorGuardCtx = createContext<TDirtyEditorGuardContext>({
  guardTabClose: () => true,
  requestCloseEditors: () => {},
  hasPending: false,
});

export function useDirtyEditorGuard(): TDirtyEditorGuardContext {
  return useContext(DirtyEditorGuardCtx);
}

export function DirtyEditorGuardProvider({
  children,
  onCancel,
  onFocus,
}: {
  children: React.ReactNode;
  onCancel?: () => void;
  onFocus?: () => void;
}): JSX.Element {
  const monacoCtx = useMonacoContext();
  const [pending, setPending] = useState<TCloseRequest[]>([]);

  const guardTabClose = useCallback(
    (model: Model, tabId: string, close?: (tabId: string) => void): boolean => {
      if (!model.getNodeById(tabId)) return true;
      if (!isEditorEditorId(tabId)) return true;
      if (monacoCtx.getModifiedFilesByEditor(tabId).length === 0) return true;
      model.doAction(Actions.selectTab(tabId));
      setPending([{ editorId: tabId, model, close }]);
      return false;
    },
    [monacoCtx]
  );

  const requestCloseEditors = useCallback((model: Model, editorIds: string[], close?: (tabId: string) => void): void => {
    const unique = Array.from(new Set(editorIds));
    setPending(unique.map((editorId) => ({ editorId, model, close })));
  }, []);

  const closeDialog = useCallback(() => setPending([]), []);

  const closeRequest = useCallback(
    (req: TCloseRequest): void => {
      if (!req.model.getNodeById(req.editorId)) {
        monacoCtx.closeEditors([req.editorId]);
        return;
      }
      // let the owning layout run its own close logic if provided
      if (req.close) {
        req.close(req.editorId);
        return;
      }
      deleteTabAndSelectNodes(req.model, req.editorId, LAYOUT_TABS.NODES);
    },
    [monacoCtx]
  );

  const discardAll = useCallback(() => {
    for (const req of pending) {
      // discard changes so the guard does not trigger again on the delete action
      for (const m of monacoCtx.getModifiedFilesByEditor(req.editorId)) {
        monacoCtx.dirtyManager()?.markSaved(m);
      }
      closeRequest(req);
    }
    closeDialog();
  }, [pending, monacoCtx, closeRequest, closeDialog]);

  const saveAll = useCallback(async () => {
    const editorIds = pending.map((p) => p.editorId);
    const editorModels = monacoCtx.modelRegistry()?.getByEditorIds(editorIds) || [];
    const dirtyModels = monacoCtx.dirtyManager()?.reduceToDirty(Array.from(editorModels)) || [];
    const results: SaveResult[] = await Promise.all(dirtyModels.map((m) => monacoCtx.saveFile(m)));

    const failed = new Set<string>();
    for (const { editorIds: ids = [], result } of results) {
      for (const id of ids) if (!result) failed.add(id);
    }

    for (const req of pending) {
      if (!failed.has(req.editorId)) {
        closeRequest(req);
      }
    }
    if (failed.size > 0) onCancel?.();
    closeDialog();
  }, [pending, monacoCtx, onCancel, closeDialog, closeRequest]);

  const value = useMemo(
    () => ({ guardTabClose, requestCloseEditors, onCancel, hasPending: pending.length > 0 }),
    [guardTabClose, requestCloseEditors, onCancel, pending.length]
  );

  return (
    <DirtyEditorGuardCtx.Provider value={value}>
      {children}
      {pending.length > 0 && (
        <Dialog
          open
          onClose={() => {
            closeDialog();
            onCancel?.();
          }}
          onFocus={() => onFocus?.()}
          fullWidth
          scroll="paper"
          maxWidth="sm"
          PaperComponent={DraggablePaper}
          aria-labelledby="draggable-dialog-title"
        >
          <DialogTitle className="draggable-dialog-title" style={{ cursor: "move" }} id="draggable-dialog-title">
            Changed Files
          </DialogTitle>
          <DialogContent aria-label="list">
            {pending.map(({ editorId }) => {
              const editorModels = monacoCtx.modelRegistry()?.getByEditorIds([editorId]) || [];
              const dirtyModels = monacoCtx.dirtyManager()?.reduceToDirty(Array.from(editorModels)) || [];
              const files = dirtyModels.map((m) => getFileName(m.uri.path));
              return (
                <DialogContentText key={editorId}>
                  {`Modified files in "${getBaseName(editorId)}" tab: ${files}`}
                </DialogContentText>
              );
            })}
          </DialogContent>
          <DialogActions>
            <Button color="warning" onClick={discardAll}>
              Don&apos;t save
            </Button>
            <Button
              color="primary"
              onClick={() => {
                closeDialog();
                onCancel?.();
              }}
            >
              Cancel
            </Button>
            <Button autoFocus color="primary" onClick={saveAll}>
              Save all
            </Button>
          </DialogActions>
        </Dialog>
      )}
    </DirtyEditorGuardCtx.Provider>
  );
}
