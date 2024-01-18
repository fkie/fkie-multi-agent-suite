import { MenuItem, Select } from '@mui/material';
import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';

import { RosContext } from '../../context/RosContext';

function ProviderSelector({ defaultProvider, setSelectedProvider }) {
  const rosCtx = useContext(RosContext);
  const [currentProvider, setCurrentProvider] = useState(defaultProvider);
  const [providerNames, setProviderNames] = useState(
    defaultProvider
      ? [{ name: rosCtx.getProviderName(defaultProvider), id: defaultProvider }]
      : [],
  );

  useEffect(() => {
    let currProv = null;
    const provNames = [];
    let hasLocal = false;
    if (rosCtx.providersConnected.length === 1) {
      // select the first host if only one is available
      currProv = rosCtx.providersConnected[0];
      provNames.push({
        name: currProv.name(),
        id: currProv.id,
      });
    } else {
      // get all connected hosts and select the first local host
      rosCtx.providersConnected.forEach((prov) => {
        if (currentProvider === '') {
          if (prov && prov.isLocalHost && !hasLocal) {
            currProv = prov;
            hasLocal = true;
          }
        }
        provNames.push({ name: prov.name(), id: prov.id });
      });
    }
    provNames.sort((a, b) => -b.name.localeCompare(a.name));
    setProviderNames(provNames);
    setCurrentProvider(currProv !== null ? currProv.id : currentProvider);
  }, [currentProvider, rosCtx.providersConnected]);

  // inform about new provider
  useEffect(() => {
    if (setSelectedProvider) {
      setSelectedProvider(currentProvider);
    }
  }, [currentProvider, setSelectedProvider]);

  return (
    <Select
      autoWidth={false}
      value={currentProvider}
      onChange={(event, child) => {
        // wait before changing provider
        // it prevents React to render too early child components
        setTimeout(() => {
          setCurrentProvider(event.target.value);
        }, 200);
      }}
      size="small"
      displayEmpty
    >
      {providerNames.map((item) => {
        return (
          <MenuItem key={item.id} value={item.id}>
            {item.name}
          </MenuItem>
        );
      })}
    </Select>
  );
}

ProviderSelector.defaultProps = {
  defaultProvider: '',
};

ProviderSelector.propTypes = {
  defaultProvider: PropTypes.string,
  setSelectedProvider: PropTypes.func.isRequired,
};

export default ProviderSelector;
