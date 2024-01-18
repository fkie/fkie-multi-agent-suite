import PropTypes from 'prop-types';

import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuTopic({ onClick, topicName, providerId }) {
  const topicOptions = [
    {
      name: 'Info',
      key: `info-${topicName}`,
      onClick: () => {
        onClick('INFO', topicName, providerId);
      },
    },
    {
      name: 'Rate (Hz)',
      key: `hz-${topicName}`,
      onClick: () => {
        onClick('HZ', topicName, providerId);
      },
    },
    {
      name: 'Bandwidth',
      key: `bw-${topicName}`,
      onClick: () => {
        onClick('BW', topicName, providerId);
      },
    },
    {
      name: 'Delay',
      key: `delay-${topicName}`,
      onClick: () => {
        onClick('DELAY', topicName, providerId);
      },
    },
    {
      name: 'Echo',
      key: `echo-${topicName}`,
      onClick: () => {
        onClick('ECHO', topicName, providerId);
      },
    },
    {
      name: 'Publish',
      key: `publish-${topicName}`,
      onClick: () => {
        onClick('PUBLISH', topicName, providerId);
      },
    },
    {
      name: 'Copy to Clipboard',
      key: `clipboard-${topicName}`,
      onClick: () => {
        onClick('clipboard', topicName, providerId);
      },
    },
  ];

  return (
    <OverflowMenu
      icon={<MoreVertSharpIcon sx={{ fontSize: 16 }} />}
      options={topicOptions}
      id="Topic Options"
    />
  );
}

OverflowMenuTopic.defaultProps = {};

OverflowMenuTopic.propTypes = {
  onClick: PropTypes.func.isRequired,
  topicName: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
};

export default OverflowMenuTopic;
