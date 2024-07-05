import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import PropTypes from 'prop-types';
import { useMemo } from 'react';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuTopic({ onClick, topicName, providerId }) {

  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: 'inherit' }} />}
        options={[
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
        ]}
        id="Topic Options"
      />
    );
  }, [topicName, providerId]);

  return createMenu;
}

OverflowMenuTopic.propTypes = {
  onClick: PropTypes.func.isRequired,
  topicName: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
};

export default OverflowMenuTopic;
