import ContentCopyIcon from '@mui/icons-material/ContentCopy';
import InfoOutlinedIcon from '@mui/icons-material/InfoOutlined';
import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import SyncAltOutlinedIcon from '@mui/icons-material/SyncAltOutlined';
import { Stack, Typography } from '@mui/material';
import PropTypes from 'prop-types';
import { useMemo } from 'react';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuService({ onClick, serviceName, providerId }) {
  const createMenu = useMemo(() => {
    return (
      <OverflowMenu
        icon={<MoreVertSharpIcon sx={{ fontSize: 'inherit' }} />}
        options={[
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={'center'}>
                <InfoOutlinedIcon sx={{ fontSize: 'inherit' }} />
                <Typography>Info</Typography>
              </Stack>
            ),
            key: `info-${serviceName}`,
            onClick: () => {
              onClick('INFO', serviceName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={'center'}>
                <SyncAltOutlinedIcon sx={{ fontSize: 'inherit' }} />
                <Typography>Call</Typography>
              </Stack>
            ),
            key: `service-call-${serviceName}`,
            onClick: () => {
              onClick('SERVICE_CALL', serviceName, providerId);
            },
          },
          {
            name: (
              <Stack direction="row" spacing={1} alignItems={'center'}>
                <ContentCopyIcon sx={{ fontSize: 'inherit' }} />
                <Typography>Copy name to Clipboard</Typography>
              </Stack>
            ),
            key: `clipboard-${serviceName}`,
            onClick: () => {
              onClick('clipboard', serviceName, providerId);
            },
          },
        ]}
        id="Service Options"
      />
    );
  }, [serviceName, providerId]);

  return createMenu;
}

OverflowMenuService.propTypes = {
  onClick: PropTypes.func.isRequired,
  serviceName: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
};

export default OverflowMenuService;
