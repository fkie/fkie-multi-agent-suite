import PropTypes from 'prop-types';
import { useCallback, useContext } from 'react';

import IconButton from '@mui/material/IconButton';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemText from '@mui/material/ListItemText';
import Stack from '@mui/material/Stack';
import Tooltip from '@mui/material/Tooltip';

import CloseIcon from '@mui/icons-material/Close';
import EditIcon from '@mui/icons-material/Edit';
import InfoOutlinedIcon from '@mui/icons-material/InfoOutlined';
import RefreshIcon from '@mui/icons-material/Refresh';
import { blue } from '@mui/material/colors';

import { emitCustomEvent } from 'react-custom-events';
import { RosContext } from '../../context/RosContext';
import { SettingsContext } from '../../context/SettingsContext';
import { getFileName } from '../../models';
import FileEditorPanel from '../../pages/NodeManager/panels/FileEditorPanel';
import LaunchFilePanel from '../../pages/NodeManager/panels/LaunchFilePanel';
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from '../../utils/events';

const compareLaunchFiles = (a, b) => {
  if (getFileName(a.path) < getFileName(b.path)) {
    return -1;
  }
  if (getFileName(a.path) > getFileName(b.path)) {
    return 1;
  }
  return 0;
};

function LaunchFileList({
  providerId,
  launchContentList,
  selectNodesFromLaunch,
  onRemoveLaunch,
  onReloadLaunch,
  onMouseOver,
}) {
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  /**
   * Create and open a new panel with a [FileEditorPanel] for a given file path and host
   */
  const createFileEditorPanel = useCallback((provId, launchContent) => {
    const launchName = getFileName(launchContent.path);
    // const provider = rosCtx.getProviderById(provId);
    // const packages = provider?.packages?.filter((rosPackage) => {
    //   return launchContent.path.startsWith(
    //     rosPackage.path.endsWith('/')
    //       ? rosPackage.path
    //       : `${rosPackage.path}/`,
    //   );
    // });
    //  [${packages.length > 0 ? packages[0].name : ''}]@${providerName}
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(
        `editor-${provId}-${launchContent.path}`,
        `Editor - ${launchName}`,
        <FileEditorPanel
          providerId={provId}
          currentFilePath={launchContent.path}
          rootFilePath={launchContent.path}
        />,
        false,
        true,
        'editor',
      ),
    );
  }, []);

  /**
   * Create and open a new panel with a [LaunchFilePanel]
   */
  const createLaunchFilePanel = useCallback(
    (provId, launchContent) => {
      const provider = rosCtx.getProviderById(provId);
      const launchName = getFileName(launchContent.path);
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          'LaunchFile',
          `LaunchFile - ${launchName}@${provider.name()}`,
          <LaunchFilePanel launchContent={launchContent} />,
          true,
          true,
          '',
        ),
      );
    },
    [rosCtx],
  );

  return (
    <List dense disablePadding onMouseOver={onMouseOver}>
      {launchContentList.length > 0 &&
        launchContentList.sort(compareLaunchFiles).map((lc) => {
          if (!lc.path) return <div>Invalid launch file</div>;
          const launchName = getFileName(lc.path);

          return (
            <ListItem
              key={`${providerId}_${lc.path}`}
              disablePadding
              onClick={() => {
                selectNodesFromLaunch(providerId, lc);
              }}
              secondaryAction={
                <Stack direction="row" spacing={0.1}>
                  <Tooltip title="Information about launch file">
                    <IconButton
                      edge="end"
                      aria-label="Information about launch file"
                      onClick={() => {
                        createLaunchFilePanel(providerId, lc);
                      }}
                    >
                      <InfoOutlinedIcon style={{ fontSize: 'inherit' }} />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Reload launch file from provider">
                    <IconButton
                      edge="end"
                      aria-label="Reload launch file from provider"
                      onClick={() => {
                        onReloadLaunch(providerId, lc.path, lc.masteruri);
                      }}
                    >
                      <RefreshIcon sx={{ fontSize: 18 }} />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Edit launch file">
                    <IconButton
                      edge="end"
                      aria-label="Edit launch file"
                      onClick={() => {
                        createFileEditorPanel(providerId, lc);
                      }}
                    >
                      <EditIcon sx={{ fontSize: 16 }} />
                    </IconButton>
                  </Tooltip>

                  <Tooltip title="Remove launch file">
                    <IconButton
                      edge="end"
                      aria-label="Remove launch file"
                      onClick={() => {
                        onRemoveLaunch(providerId, lc.path, lc.masteruri);
                      }}
                    >
                      <CloseIcon sx={{ fontSize: 18 }} />
                    </IconButton>
                  </Tooltip>
                </Stack>
              }
            >
              <ListItemButton dense>
                <ListItemText
                  primary={launchName}
                  sx={{
                    color: settingsCtx.get('useDarkMode')
                      ? blue[300]
                      : blue[800],
                  }}
                />
              </ListItemButton>
            </ListItem>
          );
        })}
    </List>
  );
}

LaunchFileList.defaultProps = {
  onMouseOver: () => {},
};

LaunchFileList.propTypes = {
  providerId: PropTypes.string.isRequired,
  launchContentList: PropTypes.array.isRequired,
  selectNodesFromLaunch: PropTypes.func.isRequired,
  onRemoveLaunch: PropTypes.func.isRequired,
  onReloadLaunch: PropTypes.func.isRequired,
  onMouseOver: PropTypes.func,
};

export default LaunchFileList;
