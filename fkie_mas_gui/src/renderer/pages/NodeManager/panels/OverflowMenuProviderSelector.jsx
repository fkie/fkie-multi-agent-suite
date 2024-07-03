import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import { IconButton } from '@mui/material';
import PropTypes from 'prop-types';
import { useMemo } from 'react';
import OverflowMenu from '../../../components/UI/OverflowMenu';

function OverflowMenuProviderSelector({
  onClick,
  providerMap,
  actionType,
  args,
  icon: MyIcon,
}) {
  const createMenu = useMemo(() => {
    const providerOptions = [];
    providerMap?.forEach((providerName, providerId) => {
      providerOptions.push({
        // name: `${providerName}#${providerId}`,
        name: providerName,
        key: `${actionType}-${providerId}`,
        providerId,
        onClick: () => {
          onClick(actionType, providerId, providerName, args);
        },
      });
    });
    if (providerMap?.size > 1) {
      return (
        <OverflowMenu
          icon={<MyIcon sx={{ fontSize: 'inherit' }} />}
          options={providerOptions}
          id="provider-options"
          showBadge
          colorizeItems
        />
      );
    }
    if (providerMap?.size === 1) {
      return (
        <IconButton
          size="small"
          onClick={() => {
            onClick(
              actionType,
              providerOptions[0].providerId,
              providerOptions[0].name,
              args,
            );
          }}
        >
          <MyIcon sx={{ fontSize: 'inherit' }} />
        </IconButton>
      );
    }
    return (
      <IconButton size="small" disabled>
        <MyIcon sx={{ fontSize: 'inherit' }} />
      </IconButton>
    );
  }, [providerMap, actionType, args]);

  return createMenu;
}

OverflowMenuProviderSelector.defaultProps = {
  actionType: '',
  args: {},
  icon: MoreVertSharpIcon,
};

OverflowMenuProviderSelector.propTypes = {
  onClick: PropTypes.func.isRequired, // onClick(actionType, providerId, providerName, args)
  providerMap: PropTypes.object.isRequired, // Map<providerId: string, name: string>
  actionType: PropTypes.string,
  args: PropTypes.object,
  icon: PropTypes.object,
};

export default OverflowMenuProviderSelector;
