import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import { IconButton } from "@mui/material";
import { forwardRef, useContext } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";

interface CopyButtonProps {
  value: string;
  logText?: string;
  fontSize?: string;
}

const CopyButton = forwardRef<HTMLDivElement, CopyButtonProps>(function CopyButton(props, ref) {
  const { value, fontSize = "inherit", logText = value } = props;
  const logCtx = useContext(LoggingContext);

  return (
    <IconButton
      ref={ref}
      sx={{ color: (theme) => theme.palette.text.disabled, paddingTop: 0, paddingBottom: 0 }}
      size="small"
      component="span"
      onClick={() => {
        navigator.clipboard.writeText(value);
        if (logText) {
          logCtx.success(`${logText} copied!`);
        }
      }}
    >
      <ContentCopyIcon sx={{ fontSize: { fontSize } }} />
    </IconButton>
  );
});

export default CopyButton;
