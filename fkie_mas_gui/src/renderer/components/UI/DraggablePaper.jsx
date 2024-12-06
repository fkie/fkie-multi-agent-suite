import { Paper } from "@mui/material";
import PropTypes from "prop-types";
import React from "react";
import Draggable from "react-draggable";

function DraggablePaper({ dialogRef = null, ...props }) {
  return (
    <Draggable nodeRef={dialogRef} handle="#draggable-dialog-title" cancel={'[class*="MuiDialogContent-root"]'}>
      <Paper ref={dialogRef} {...props} />
    </Draggable>
  );
}

DraggablePaper.propTypes = {
  dialogRef: PropTypes.any,
};

export default DraggablePaper;
