import { Paper } from "@mui/material";
import PropTypes from "prop-types";
import React from "react";
import Draggable from "react-draggable";

function DraggablePaper({ dialogRef=null, ...props }) {
  return (
    <Draggable nodeRef={dialogRef}>
      <Paper {...props} />
    </Draggable>
  );
}

DraggablePaper.propTypes = {
  dialogRef: PropTypes.any,
};

export default DraggablePaper;
