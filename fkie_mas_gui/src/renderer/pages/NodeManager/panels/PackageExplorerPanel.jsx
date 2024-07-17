import { Alert, AlertTitle, Box, FormControl, IconButton, Stack, Tooltip } from '@mui/material';
import { useCallback, useContext, useEffect, useMemo, useState } from 'react';
import { useCustomEventListener } from 'react-custom-events';
import RefreshIcon from '@mui/icons-material/Refresh';
import { EVENT_PROVIDER_STATE } from '../../../providers/eventTypes';
import { PackageExplorer, ProviderSelector, colorFromHostname } from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';

function PackageExplorerPanel() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [selectedProvider, setSelectedProvider] = useState();
  const [packageList, setPackageList] = useState([]);
  const [showReloadButton, setShowReloadButton] = useState(false);
  // const [hostColor, setHostColor] = useState(
  //   settingsCtx.get('backgroundColor'),
  // );
  const [hostStyle, setHostStyle] = useState({});
  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  const updatePackageList = useCallback(
    async (providerId, force = false) => {
      if (!rosCtx.initialized) return;
      if (!rosCtx.providers) return;
      if (!providerId) return;
      if (providerId.length === 0) return;

      const provider = rosCtx.getProviderById(providerId);
      if (!provider) return;
      if (!provider.packages || force) {
        const pl = await provider.getPackageList();
        setPackageList(() => pl);
      } else {
        setPackageList(() => provider.packages);
      }
    },

    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.initialized, rosCtx.providers]
  );

  // Update files when selected provider changes
  useEffect(() => {
    if (!selectedProvider) return;
    if (selectedProvider.length === 0) return;
    updatePackageList(selectedProvider);
    setShowReloadButton(true);
    // if (settingsCtx.get('colorizeHosts')) {
    //   const provider = rosCtx.getProviderById(selectedProvider);
    //   setHostColor(
    //     `linear-gradient(${settingsCtx.get(
    //       'backgroundColor',
    //     )}, ${colorFromHostname(provider.name())}, ${settingsCtx.get(
    //       'backgroundColor',
    //     )} 15px)`,
    //   );
    // } else {
    //   setHostColor(settingsCtx.get('backgroundColor'));
    // }
    if (settingsCtx.get('colorizeHosts')) {
      const provider = rosCtx.getProviderById(selectedProvider);
      const hColor = colorFromHostname(provider.name());
      setHostStyle({
        borderLeftStyle: 'solid',
        borderLeftColor: hColor,
        borderLeftWidth: '0.6em',
        borderBottomStyle: 'solid',
        borderBottomColor: hColor,
        borderBottomWidth: '0.6em',
      });
    } else {
      setHostStyle({});
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedProvider, settingsCtx.config]);

  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    if (!selectedProvider) return;
    if (selectedProvider === data.provider.id && data.newState === ConnectionState.STATES.CONNECTED) {
      updatePackageList(selectedProvider);
    }
  });

  const createPackageExplorer = useMemo(() => {
    return <PackageExplorer selectedProvider={selectedProvider} packageList={packageList} />;
  }, [selectedProvider, packageList]);

  return (
    <Box
      width="100%"
      height="100%"
      style={{
        background: settingsCtx.get('backgroundColor'),
      }}
      // paddingLeft="10px"
    >
      <Stack direction="column" height="100%" width="100% ">
        {rosCtx.providers && rosCtx.providers.length > 0 && (
          <Stack spacing={1} direction="row" width="100%" sx={hostStyle}>
            <FormControl sx={{ m: 1, width: '100%' }} variant="standard">
              <ProviderSelector
                defaultProvider={selectedProvider}
                setSelectedProvider={(provId) => setSelectedProvider(provId)}
              />
            </FormControl>
            {showReloadButton && (
              <Tooltip
                title="Reload package list"
                placement="bottom"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <IconButton
                  size="small"
                  onClick={() => {
                    updatePackageList(selectedProvider, true);
                  }}
                >
                  <RefreshIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )}
          </Stack>
        )}
        {(!rosCtx.providers || rosCtx.providers.length === 0) && (
          <Alert severity="info" style={{ minWidth: 0, marginTop: 10 }}>
            <AlertTitle>No providers available</AlertTitle>
            Please connect to a ROS provider
          </Alert>
        )}
        {selectedProvider && selectedProvider.length > 0 && <Stack overflow="auto">{createPackageExplorer}</Stack>}
      </Stack>
    </Box>
  );
}

PackageExplorerPanel.propTypes = {};

export default PackageExplorerPanel;
