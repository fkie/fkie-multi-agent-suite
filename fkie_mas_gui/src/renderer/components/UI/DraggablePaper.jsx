import { Paper } from '@mui/material';
import Draggable from 'react-draggable';

function DraggablePaper(props) {
  return (
    <Draggable
      handle="#draggable-dialog-title"
      cancel={'[class*="MuiDialogContent-root"]'}
    >
      <Paper {...props} />
    </Draggable>
  );
}

DraggablePaper.defaultProps = {};

DraggablePaper.propTypes = {};

export default DraggablePaper;
