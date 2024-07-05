import { MenuItem, Select } from '@mui/material';
import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';

import { RosContext } from '../../context/RosContext';

function ProviderSelector({ defaultProvider = '', setSelectedProvider }) {
  const rosCtx = useContext(RosContext);
  const [currentProvider, setCurrentProvider] = useState(defaultProvider);
  const [providerNames, setProviderNames] = useState(
    defaultProvider
      ? [{ name: rosCtx.getProviderName(defaultProvider), id: defaultProvider }]
      : [],
  );

  useEffect(() => {
    const provNames = [];
    let hasLocal = false;
    let newSelectedProvider = '';
    if (rosCtx.providersConnected.length === 1) {
      // select the first host if only one is available
      const currProv = rosCtx.providersConnected[0];
      provNames.push({
        name: currProv.name(),
        id: currProv.id,
      });
      newSelectedProvider = currProv.id;
    } else {
      // get all connected hosts and select the first local host
      rosCtx.providersConnected.forEach((prov) => {
        if (currentProvider === '') {
          if (prov && prov.isLocalHost && !hasLocal) {
            hasLocal = true;
            newSelectedProvider = prov.id;
          }
        } else if (currentProvider === prov.id) {
          newSelectedProvider = currentProvider;
        }
        provNames.push({ name: prov.name(), id: prov.id });
      });
    }
    provNames.sort((a, b) => -b.name.localeCompare(a.name));
    if (newSelectedProvider !== currentProvider) {
      // current provider is not in the name list, add to avoid warnings
      provNames.push({ name: currentProvider, id: currentProvider });
    }
    setProviderNames(provNames);
    setCurrentProvider(newSelectedProvider);
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

ProviderSelector.propTypes = {
  defaultProvider: PropTypes.string,
  setSelectedProvider: PropTypes.func.isRequired,
};

export default ProviderSelector;
