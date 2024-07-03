import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import PropTypes from 'prop-types';
import { useMemo } from 'react';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuProvider({ onClick, providerId, providerName }) {

  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: 'inherit' }} />}
        options={[
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
        ]}
        id="provider-options"
      />
    );
  }, [providerId, providerName]);

  return createMenu;
}

OverflowMenuProvider.defaultProps = {};

OverflowMenuProvider.propTypes = {
  onClick: PropTypes.func.isRequired,
  providerId: PropTypes.string.isRequired,
  providerName: PropTypes.string.isRequired,
};

export default OverflowMenuProvider;
