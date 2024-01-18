import PropTypes from 'prop-types';

import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuProvider({ onClick, providerId, providerName }) {
  const providerOptions = [
    {
      name: 'Info',
      key: `info-${providerId}`,
      onClick: () => {
        onClick('INFO', providerId, providerName);
      },
    },
    {
      name: 'Delete',
      key: `delete-${providerId}`,
      onClick: () => {
        onClick('DELETE', providerId, providerName);
      },
    },
  ];

  return (
    <OverflowMenu
      icon={<MoreVertSharpIcon sx={{ fontSize: 16 }} />}
      options={providerOptions}
      id="provider-options"
    />
  );
}

OverflowMenuProvider.defaultProps = {};

OverflowMenuProvider.propTypes = {
  onClick: PropTypes.func.isRequired,
  providerId: PropTypes.string.isRequired,
  providerName: PropTypes.string.isRequired,
};

export default OverflowMenuProvider;
