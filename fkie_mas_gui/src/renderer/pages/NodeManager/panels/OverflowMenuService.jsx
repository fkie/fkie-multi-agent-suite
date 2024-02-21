import PropTypes from 'prop-types';

import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuService({ onClick, serviceName, providerId }) {
  const serviceOptions = [
    {
      name: 'Info',
      key: `info-${serviceName}`,
      onClick: () => {
        onClick('INFO', serviceName, providerId);
      },
    },
    {
      name: 'Call',
      key: `service-call-${serviceName}`,
      onClick: () => {
        onClick('SERVICE_CALL', serviceName, providerId);
      },
    },
    {
      name: 'Copy to Clipboard',
      key: `clipboard-${serviceName}`,
      onClick: () => {
        onClick('clipboard', serviceName, providerId);
      },
    },
  ];

  return (
    <OverflowMenu
      icon={<MoreVertSharpIcon sx={{ fontSize: 'inherit' }} />}
      options={serviceOptions}
      id="Service Options"
    />
  );
}

OverflowMenuService.defaultProps = {};

OverflowMenuService.propTypes = {
  onClick: PropTypes.func.isRequired,
  serviceName: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
};

export default OverflowMenuService;
