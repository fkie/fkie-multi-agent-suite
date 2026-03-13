import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import { IconButton } from "@mui/material";

import { useLoggingContext } from "@/renderer/hooks/useLoggingContext";

interface CopyButtonProps {
  value: string;
  logText?: string;
  fontSize?: string;
}

export default function CopyButton(props: CopyButtonProps): JSX.Element {
  const { value, fontSize = "inherit", logText = value } = props;
  const logCtx = useLoggingContext();

  return (
    <IconButton
      sx={{ color: (theme) => theme.palette.text.disabled, paddingTop: 0, paddingBottom: 0 }}
      size="small"
      component="span"
      onClick={() => {
        navigator.clipboard.writeText(value);
        if (logText) {
          logCtx.info(`${logText} copied!`, "", "copied to clipboard");
        }
      }}
    >
      <ContentCopyIcon sx={{ fontSize: { fontSize } }} />
    </IconButton>
  );
}
