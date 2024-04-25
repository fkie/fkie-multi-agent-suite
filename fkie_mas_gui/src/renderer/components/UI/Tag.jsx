import { Chip } from '@mui/material';
import PropTypes from 'prop-types';
import CopyButton from './CopyButton';

const chipDefaultColors = [
  'default',
  'primary',
  'secondary',
  'error',
  'info',
  'success',
  'warning',
];

function Tag({ title, text, color, copyButton, wrap }) {
  const isDefaultColor = chipDefaultColors.includes(color);

  const chipSX = {
    // fontSize: SettingsCtx.fontSize,
    height: 'auto',
  };

  if (wrap) {
    chipSX['& .MuiChip-label'] = {
      display: 'block',
      whiteSpace: 'normal',
      wordWrap: 'break-word',
    };
  }

  let newText = text;
  if (title) {
    newText = ` ${newText}`;
  }

  return (
    <Chip
      size="small"
      color={isDefaultColor ? color : 'default'}
      style={isDefaultColor ? {} : { backgroundColor: color }}
      label={
        <>
          <strong>{title}</strong>
          {newText}
          {copyButton && <CopyButton value={copyButton} fontSize="0.6em" />}
        </>
      }
      sx={chipSX}
    />
  );
}

Tag.defaultProps = {
  title: '',
  text: '',
  color: 'info',
  copyButton: '',
  wrap: true,
};

Tag.propTypes = {
  title: PropTypes.string,
  text: PropTypes.string,
  color: PropTypes.string,
  copyButton: PropTypes.string,
  wrap: PropTypes.bool,
};

export default Tag;
