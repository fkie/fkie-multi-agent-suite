import RefreshIcon from '@mui/icons-material/Refresh';
import {
  IconButton,
  Stack,
  Table,
  TableBody,
  TableContainer,
  Tooltip,
} from '@mui/material';
import { useDebounceCallback } from '@react-hook/debounce';
import { useContext, useEffect, useMemo, useState } from 'react';
import { emitCustomEvent, useCustomEventListener } from 'react-custom-events';
import { ConnectToProviderModal, SearchBar } from '../../../components';
import { RosContext } from '../../../context/RosContext';
import { SettingsContext } from '../../../context/SettingsContext';
import { EVENT_PROVIDER_STATE } from '../../../providers/eventTypes';
import { EVENT_OPEN_CONNECT } from '../../../utils/events';
import ProviderPanelRow from './ProviderPanelRow';

function ProviderPanel() {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [providerRowsFiltered, setProviderRowsFiltered] = useState([]);
  const [filterText, setFilterText] = useState('');
  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  const debouncedCallbackFilterText = useDebounceCallback(
    (providers, searchTerm) => {
      if (searchTerm.length > 1) {
        const re = new RegExp(searchTerm, 'i');
        setProviderRowsFiltered(
          providers.filter((provider) => {
            const pos = provider.name().search(re);
            return pos !== -1;
          }),
        );
      } else {
        setProviderRowsFiltered(providers);
      }
    },
    300,
  );

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  useCustomEventListener(EVENT_PROVIDER_STATE, (data) => {
    debouncedCallbackFilterText([...rosCtx.providers], filterText);
  });

  useEffect(() => {
    debouncedCallbackFilterText(rosCtx.providers, filterText);
  }, [rosCtx.providers, filterText, debouncedCallbackFilterText]);

  useEffect(() => {
    if (rosCtx.providers.length === 0) {
      emitCustomEvent(EVENT_OPEN_CONNECT, {});
    }
  }, [rosCtx.rosInfo, rosCtx.providers]);

  const createProviderTable = useMemo(() => {
    const result = (
      <TableContainer>
        <Table aria-label="hosts table">
          <TableBody>
            {providerRowsFiltered.map((provider) => {
              return <ProviderPanelRow provider={provider} />;
            })}
          </TableBody>
        </Table>
      </TableContainer>
    );
    return result;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [providerRowsFiltered, rosCtx]);

  return (
    <Stack
      spacing={1}
      height="100%"
      // width="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      <Stack direction="row" spacing={0.5}>
        <SearchBar
          onSearch={(value) => {
            setFilterText(value);
          }}
          placeholder="Filter hosts"
          defaultValue={filterText}
          // fullWidth={true}
        />
        <ConnectToProviderModal />
        <Tooltip
          title="Refresh hosts list"
          placement="bottom"
          enterDelay={tooltipDelay}
          enterNextDelay={tooltipDelay}
        >
          <IconButton
            edge="start"
            aria-label="refresh hosts list"
            onClick={() => rosCtx.refreshProviderList()}
          >
            <RefreshIcon sx={{ fontSize: 'inherit' }} />
          </IconButton>
        </Tooltip>
      </Stack>
      {createProviderTable}
    </Stack>
  );
}

ProviderPanel.defaultProps = {};

ProviderPanel.propTypes = {};

export default ProviderPanel;
