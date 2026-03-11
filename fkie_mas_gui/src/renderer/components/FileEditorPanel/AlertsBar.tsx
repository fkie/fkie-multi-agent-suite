import { useMonacoContext } from "@/renderer/hooks/useMonacoContext";
import { fileFromUriPath } from "@/renderer/monaco/utils";
import { Alert, Stack } from "@mui/material";
import { editor } from "monaco-editor";
import { ForwardedRef } from "react";

interface Props {
  refEl: ForwardedRef<HTMLDivElement>;
  message?: string;
  activeModel?: editor.ITextModel | null;
  onClose: () => void;
}

/**
 * Editor alert bar
 */
export function AlertsBar({ refEl, message, activeModel, onClose }: Props) {
  const monacoCtx = useMonacoContext();

  return (
    <Stack ref={refEl} direction="column">
      {message && (
        <Alert
          severity="warning"
          style={{ minWidth: 0 }}
          onClose={() => {
            onClose();
          }}
        >
          {message}
        </Alert>
      )}
      {activeModel && monacoCtx.isReadOnly(activeModel) ? (
        <Alert severity="info" style={{ minWidth: 0 }}>
          {`no write permissions for ${fileFromUriPath(activeModel.uri.path)}`}
        </Alert>
      ) : (
        activeModel?.uri.path &&
        monacoCtx.isInstallPath(activeModel) && (
          <Alert severity="warning" style={{ minWidth: 0 }}>
            {`${fileFromUriPath(activeModel?.uri.path)} is located in 'install' path. The changes could be lost after rebuilding packages. You can build your packages with '--symlink-install' to edit your launch files in your sources.`}
          </Alert>
        )
      )}
    </Stack>
  );
}
