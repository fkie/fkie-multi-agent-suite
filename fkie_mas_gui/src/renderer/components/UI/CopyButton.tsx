import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import { IconButton } from "@mui/material";
import { forwardRef, useContext } from "react";

import { LoggingContext } from "@/renderer/context/LoggingContext";

interface CopyButtonProps {
  value: string;
  fontSize?: string;
}

const CopyButton = forwardRef<HTMLDivElement, CopyButtonProps>(function CopyButton(props, ref) {
  const { value, fontSize = "inherit" } = props;
  const logCtx = useContext(LoggingContext);

  return (
    <IconButton
      ref={ref}
      sx={{ color: (theme) => theme.palette.text.disabled, paddingTop: 0, paddingBottom: 0 }}
      size="small"
      component="span"
      onClick={() => {
        navigator.clipboard.writeText(value);
        logCtx.success(`${value} copied!`);
      }}
    >
      <ContentCopyIcon sx={{ fontSize: { fontSize } }} />
    </IconButton>
  );
});

export default CopyButton;
