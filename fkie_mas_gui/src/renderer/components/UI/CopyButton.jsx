import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import { IconButton } from "@mui/material";
import { grey } from "@mui/material/colors";
import PropTypes from "prop-types";
import { useContext } from "react";
import { LoggingContext } from "../../context/LoggingContext";

function CopyButton({ value, fontSize = "inherit" }) {
  const logCtx = useContext(LoggingContext);

  return (
    <IconButton
      sx={{ color: grey[300] }}
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
}

CopyButton.propTypes = {
  value: PropTypes.string.isRequired,
  fontSize: PropTypes.string,
};

export default CopyButton;
