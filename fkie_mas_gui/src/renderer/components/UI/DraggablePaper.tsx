import { Paper } from "@mui/material";
import { forwardRef, LegacyRef } from "react";
import Draggable from "react-draggable";

interface DraggablePaperProps {
  dialogRef: React.RefObject<HTMLElement> | undefined;
}

const DraggablePaper = forwardRef<HTMLDivElement, DraggablePaperProps>(function DraggablePaper(props, ref) {
  const { dialogRef = undefined, ...otherProps } = props;
  return (
    <Draggable
      ref={ref as LegacyRef<Draggable>}
      nodeRef={dialogRef}
      handle="#draggable-dialog-title"
      cancel={'[class*="MuiDialogContent-root"]'}
    >
      <Paper ref={dialogRef as React.RefObject<HTMLDivElement>} {...otherProps} />
    </Draggable>
  );
});

export default DraggablePaper;
