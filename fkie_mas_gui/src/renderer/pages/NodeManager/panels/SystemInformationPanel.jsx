import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';
import ReactJson from 'react-json-view';

import { Box, Stack, Typography } from '@mui/material';

import { CopyButton, SearchBar } from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { generateUniqueId } from '../../../utils';

function SystemInformationPanel({ providerId }) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [systemInfoContent, setSystemInfoContent] = useState(null);
  const [provider, setProvider] = useState(null);
  const [providerDetails, setProviderDetails] = useState({});
  const [providerConfiguration, setProviderConfiguration] = useState({});
  const [providerWarnings, setProviderWarnings] = useState([]);
  const [filter, setFilter] = useState('');

  /** filter dictionaries with system information by given filter */
  function filterNestObject(item) {
    if (!item) return item;
    return Object.keys(item).reduce((object, key) => {
      let addKey = false;
      let nested = item[key];
      switch (typeof nested) {
        case 'object':
          nested = filterNestObject(item[key]);
          if (!['{}', 'null'].includes(JSON.stringify(nested))) {
            addKey = true;
          }
          break;
        case 'string':
          if (nested?.toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        case 'number':
          if (nested.toString().toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        case 'function':
          nested = nested();
          if (nested?.toLocaleLowerCase().includes(filter)) {
            addKey = true;
          }
          break;
        default:
          break;
      }
      if (key.toLocaleLowerCase().includes(filter)) {
        addKey = true;
      }
      if (addKey) {
        return {
          ...object,
          [key]: nested,
        };
      }
      return { ...object };
    }, {});
  }

  useEffect(() => {
    if (provider && provider.isLocalHost) {
      setSystemInfoContent(filterNestObject(rosCtx.systemInfo));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [provider, filter, setSystemInfoContent]);

  useEffect(() => {
    if (provider) {
      const infoContent = {};
      infoContent.URI = `${provider.host()}:${provider.crossbar.port}`;
      infoContent.ROS_VERSION = provider.rosState.ros_version;
      infoContent.ROS_DISTRO = provider.rosState.ros_distro;
      if (provider.rosState.ros_version === '1') {
        infoContent.ROS_MASTER_URI = provider.rosState.masteruri;
      }
      if (provider.rosState.ros_version === '2') {
        infoContent.ROS_DOMAIN_ID = provider.rosState.ros_domain_id;
      } else {
        infoContent.NETWORK_ID = provider.rosState.ros_domain_id;
      }
      infoContent['System Information'] = provider.systemInfo?.system_info;
      infoContent['System Environment'] = provider.systemEnv?.environment;
      // infoContent['Platform details'] = provider.platform_details;
      infoContent.Hostnames = provider.hostnames;
      if (provider.errorDetails) {
        infoContent['Last error'] = provider.errorDetails;
      }
      infoContent['Daemon version'] =
        `${provider.daemonVersion.version} (${provider.daemonVersion.date})`;
      setProviderDetails(filterNestObject(infoContent));
      setProviderConfiguration(
        filterNestObject(rosCtx.getProviderLaunchConfig(providerId)),
      );
      // join warnings to one list
      let warnings = [];
      provider.warnings.forEach((w) => {
        warnings = [...warnings, ...w.warnings];
      });
      setProviderWarnings(warnings);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [provider, rosCtx.getProviderLaunchConfig, filter]);

  useEffect(() => {
    setProvider(rosCtx.getProviderById(providerId));
  }, [providerId, rosCtx, rosCtx.getProviderById]);

  return (
    <Box
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      <Stack direction="column" margin="0.5em" spacing="0.5em">
        <SearchBar
          onSearch={(value) => setFilter(value.toLocaleLowerCase())}
          placeholder="filter"
          defaultValue=""
        />
        <Stack spacing={1} direction="row">
          <Typography variant="h5" sx={{ fontWeight: 'bold' }}>
            Name:
          </Typography>
          <Typography variant="h5">{provider?.name()}</Typography>
          <CopyButton value={`${provider?.name()}`} />
        </Stack>
        {providerWarnings.length > 0 && (
          <Stack sx={{ width: '100%' }}>
            <h5>Warnings</h5>
            {providerWarnings.map((item) => {
              return (
                <Stack key={generateUniqueId()} sx={{ width: '100%' }}>
                  <Typography variant="subtitle1" color="red">
                    {item.msg}
                  </Typography>
                  <Typography paddingLeft={1} variant="body1">
                    {item.details}
                  </Typography>
                  {item.hint && (
                    <Typography
                      paddingLeft={1}
                      variant="body1"
                      fontStyle="italic"
                    >
                      Hint: {item.hint}
                    </Typography>
                  )}
                </Stack>
              );
            })}
          </Stack>
        )}
        {providerDetails && (
          <Stack sx={{ width: '100%' }}>
            <h5>Provider Details</h5>
            <ReactJson
              name={false}
              collapsed={1}
              theme={
                settingsCtx.get('useDarkMode') ? 'grayscale' : 'rjv-default'
              }
              src={providerDetails}
              collapseStringsAfterLength={100}
              displayObjectSize={false}
              enableClipboard
              indentWidth={2}
              displayDataTypes={false}
              iconStyle="triangle"
              quotesOnKeys={false}
              sortKeys
            />
          </Stack>
        )}
        {providerConfiguration && (
          <Stack sx={{ width: '100%' }}>
            <h5>Provider Launch Configuration</h5>
            <ReactJson
              name={false}
              collapsed={1}
              theme={
                settingsCtx.get('useDarkMode') ? 'grayscale' : 'rjv-default'
              }
              src={providerConfiguration}
              collapseStringsAfterLength={100}
              displayObjectSize={false}
              enableClipboard
              indentWidth={2}
              displayDataTypes={false}
              iconStyle="triangle"
              quotesOnKeys={false}
              sortKeys
            />
          </Stack>
        )}
        {systemInfoContent && (
          <Stack sx={{ width: '100%' }}>
            <h5>This System Information</h5>
            <ReactJson
              name={false}
              collapsed={1}
              theme={
                settingsCtx.get('useDarkMode') ? 'grayscale' : 'rjv-default'
              }
              src={systemInfoContent}
              collapseStringsAfterLength={200}
              displayObjectSize={false}
              enableClipboard
              indentWidth={2}
              displayDataTypes={false}
              iconStyle="triangle"
              quotesOnKeys={false}
              sortKeys
            />
          </Stack>
        )}
      </Stack>
    </Box>
  );
}
SystemInformationPanel.defaultProps = {
  providerId: '',
};

SystemInformationPanel.propTypes = {
  providerId: PropTypes.string,
};

export default SystemInformationPanel;
