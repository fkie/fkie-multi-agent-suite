import { Alert, AlertTitle } from '@mui/material';
import Box from '@mui/material/Box';
import Stack from '@mui/material/Stack';
import { blue, gray } from '@mui/material/colors';
import PropTypes from 'prop-types';
import { useContext } from 'react';
import TerminalClient from '../../../components/TerminalClient/TerminalClient';
import { SettingsContext } from '../../../context/SettingsContext';

function TerminalPanel({ selectedNodes }) {
  const settingsCtx = useContext(SettingsContext);

  return (
    <Box
      width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      {selectedNodes.length === 0 &&
        selectedNodes.map((node) => {
          return (
            <Stack key={`${node.name}_${node.providerName}`}>
              <Stack spacing={0}>
                <Box
                  display="flex"
                  justifyContent="center"
                  alignItems="center"
                  sx={{ color: blue[70] }}
                >
                  <h4>{node.name.replaceAll('_', ' ')}</h4>
                </Box>
                <Box display="flex" justifyContent="center" alignItems="center">
                  <h6
                    style={{ color: gray[70], fontSize: settingsCtx.fontSize }}
                  >
                    {node.providerName}
                  </h6>
                </Box>
              </Stack>
              {node.providerName && (
                <TerminalClient
                  tokenUrl={`${node.name.replaceAll('/', '')}`}
                  wsUrl={`ws://${node.providerName}:7681/ws`}
                  initialCommands={[
                    `screen -d -r $(ps -eF | awk '/._${node.name.replaceAll(
                      '_',
                      '__',
                    )} / {print $2}') \r`,
                  ]}
                />
              )}
            </Stack>
          );
        })}

      {selectedNodes && selectedNodes.length === 0 && (
        <Alert severity="info">
          <AlertTitle>Please select a node</AlertTitle>
        </Alert>
      )}
    </Box>
  );
}

TerminalPanel.propTypes = {
  selectedNodes: PropTypes.arrayOf(PropTypes.any).isRequired,
};

export default TerminalPanel;
