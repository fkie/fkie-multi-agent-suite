import PropTypes from 'prop-types';
import { useContext } from 'react';

import {
  Alert,
  AlertTitle,
  Box,
  Chip,
  ListItem,
  ListItemButton,
  ListItemText,
  Stack,
  Typography,
} from '@mui/material';

import { TreeItem, TreeView } from '@mui/x-tree-view';

import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';

import { VariableSizeList } from 'react-window';

import { SettingsContext } from '../../../context/SettingsContext';
import { LaunchContent } from '../../../models';

function LaunchFilePanel({ launchContent }) {
  const settingsCtx = useContext(SettingsContext);
  const nodes = launchContent.nodes.filter((e) => !e.composable_container);
  const nodelets = launchContent.nodes
    .filter((e) => e.composable_container)
    .reduce((tree, nodelet) => {
      const manager = tree.find(
        (n) => n.manager && n.manager === nodelet.composable_container,
      );
      if (manager) {
        manager.nodes.push(nodelet.node_name);
      } else {
        tree.push({
          manager: nodelet.composable_container,
          nodes: [nodelet.node_name],
        });
      }
      return tree;
    }, []);

  const renderNodeRow = ({ index, style }) => {
    return (
      <ListItem
        style={style}
        key={index}
        component="div"
        disablePadding
        dense
        disableGutters
      >
        <ListItemButton dense disableGutters>
          <ListItemText primary={`${nodes[index].node_name}`} />
        </ListItemButton>
      </ListItem>
    );
  };

  // const renderParameterRow = ({ columnIndex, rowIndex, style }) => {
  //   let text = '';

  //   if (columnIndex === 0) {
  //     text = launchContent.parameters[rowIndex].name;
  //   } else {
  //     text = launchContent.parameters[rowIndex].value;
  //   }

  //   return (
  //     <ListItem
  //       style={style}
  //       key={`${columnIndex}_${rowIndex}`}
  //       component="div"
  //       disablePadding
  //       dense
  //       disableGutters
  //     >
  //       <ListItemButton dense disableGutters>
  //         <ListItemText primary={`${text}`} />
  //       </ListItemButton>
  //     </ListItem>
  //   );
  // };
  const chipSX = {
    // fontSize: SettingsCtx.fontSize,
    height: 'auto',
    '& .MuiChip-label': {
      display: 'block',
      whiteSpace: 'normal',
    },
  };

  return (
    <Box
      // width="100%"
      height="100%"
      overflow="auto"
      backgroundColor={settingsCtx.get('backgroundColor')}
    >
      <Stack spacing={1}>
        <Stack direction="row" spacing={1}>
          {launchContent.host && launchContent.host.length > 0 && (
            <Chip
              size="small"
              color="default"
              label={
                <>
                  <strong>Host</strong>
                  {`  ${launchContent.host}  `}
                </>
              }
              sx={chipSX}
            />
          )}

          {launchContent.host && launchContent.host.length > 0 && (
            <Chip
              size="small"
              color="default"
              label={
                <>
                  <strong>MasterURI</strong>
                  {`  ${launchContent.masteruri}  `}
                </>
              }
              sx={chipSX}
            />
          )}
        </Stack>

        <Typography variant="body">
          <strong>Path</strong>
          {`  ${launchContent.path}  `}
        </Typography>

        {/* TODO: Fix and add arguments of the launch file */}

        <Typography variant="body">
          Arguments {`[${launchContent.args.length}]`}
        </Typography>

        {launchContent &&
          launchContent.args &&
          launchContent.args.length > 0 && (
            <TreeView
              aria-label="file system navigator"
              defaultCollapseIcon={<ExpandMoreIcon />}
              defaultExpandIcon={<ChevronRightIcon />}
            >
              {launchContent.args.map((arg) => {
                return (
                  <TreeItem
                    key={`${arg.name}_${arg.name}`}
                    nodeId={arg.name}
                    label={`${arg.name}: [${arg.value}]`}
                  />
                );
              })}
            </TreeView>
          )}

        {launchContent && nodelets.length > 0 && (
          <>
            <Typography variant="body">
              Nodelets {`[${nodelets.length}]`}
            </Typography>
            <TreeView
              aria-label="file system navigator"
              defaultCollapseIcon={<ExpandMoreIcon />}
              defaultExpandIcon={<ChevronRightIcon />}
            >
              {nodelets.map((nodelet) => {
                return (
                  <TreeItem
                    nodeId={nodelet.manager}
                    label={nodelet.manager}
                    key={nodelet.manager}
                  >
                    {nodelet.nodes.map((node) => {
                      return <TreeItem nodeId={node} label={node} key={node} />;
                    })}
                  </TreeItem>
                );
              })}
            </TreeView>
          </>
        )}

        {launchContent && nodes.length > 0 && (
          <>
            <Typography variant="body">Nodes {`[${nodes.length}]`}</Typography>
            <Box
              sx={{
                width: '100%',
                height: 150,
              }}
            >
              <VariableSizeList
                height={150}
                width="100%"
                itemSize={(index) => 30}
                itemCount={nodes.length}
              >
                {renderNodeRow}
              </VariableSizeList>
            </Box>
          </>
        )}

        <Typography variant="body">
          Parameters {`[${launchContent.parameters.length}]`}
        </Typography>

        {/* TODO: Do we want to show parameters? */}
        {/* {launchContent &&
          launchContent.parameters &&
          launchContent.parameters.length > 0 && (
            <>
              <Box
                sx={{
                  width: '100%',
                  height: 150,
                }}
              >
                <VariableSizeGrid
                  columnCount={2}
                  columnWidth={(index) => {
                    if (index === 0) {
                      if (width) return width * 0.8;

                      return 280;
                    }

                    if (index === 1) {
                      if (width) return width * 0.2;

                      return 20;
                    }

                    return 20;
                  }}
                  height={150}
                  width={width ? width - 40 : 300}
                  rowCount={launchContent.parameters.length}
                  rowHeight={(index) => 30}
                >
                  {renderParameterRow}
                </VariableSizeGrid>
              </Box>
            </>
          )} */}
      </Stack>

      {!launchContent && (
        <Alert severity="error" style={{ minWidth: 0 }}>
          <AlertTitle>Invalid Launch file</AlertTitle>
        </Alert>
      )}
    </Box>
  );
}

LaunchFilePanel.propTypes = {
  launchContent: PropTypes.instanceOf(LaunchContent).isRequired,
};

export default LaunchFilePanel;
