import {
  Alert,
  AlertTitle,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Stack,
  TextField,
} from '@mui/material';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useState } from 'react';
import { Controller, useForm } from 'react-hook-form';

import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import { LaunchArgument, LaunchLoadRequest, getFileName } from '../../models';
import DraggablePaper from '../UI/DraggablePaper';
import Tag from '../UI/Tag';

function LaunchFileModal({
  selectedProvider,
  selectedLaunchFile,
  setSelectedLaunchFile,
  onLaunchCallback,
}) {
  const rosCtx = useContext(RosContext);
  const logCtx = useContext(LoggingContext);
  const [open, setOpen] = useState(false);
  const [selectedLaunch, setSelectedLaunch] = useState(null);
  const [messageLaunchLoaded, setMessageLaunchLoaded] = useState('');

  const { control, setValue, getValues } = useForm({
    defaultValues: {},
  });

  // Make a request to provider and get Launch attributes like required arguments, status and paths
  const getLaunchFile = useCallback(
    async (file) => {
      const provider = rosCtx.getProviderById(selectedProvider);
      if (!provider || !provider.isAvailable()) return;

      if (provider.launchLoadFile) {
        /*
         * ros_package -
         * launch - Launch file in the package path.
         * path - if set, this will be used instead of package/launch
         * @param {LaunchArgument[]} args - Arguments to load the launch file.
         * @param {boolean} force_first_file - If True, use first file if more than one was found in the package.
         * @param {boolean} request_args - If True, the launch file will not be loaded, only launch arguments are requested.
         * masteruri - Starts nodes of this file with specified ROS_MASTER_URI.
         * host - Start nodes of this file on specified host.
         * */
        const rosPackage = ''; // ROS package name.
        const launch = ''; // Launch file in the package path.
        const path = file;
        const args = [];
        const forceFirstFile = true;
        const requestArgs = true;
        const masteruri = '';
        const host = '';
        const request = new LaunchLoadRequest(
          rosPackage,
          launch,
          path,
          args,
          forceFirstFile,
          requestArgs,
          masteruri,
          host,
        );
        const result = await provider.launchLoadFile(request);

        if (result.status.code === 'ALREADY_OPEN') {
          logCtx.warn(
            `Launch file [${getFileName(path)}] was already loaded`,
            `File: ${path}`,
          );
          setMessageLaunchLoaded('Launch file was already loaded');

          // nothing else to do return
          onLaunchCallback();
          return;
        }

        if (result.status.code === 'PARAMS_REQUIRED') {
          setSelectedLaunch(() => result);

          // set default values to arguments in form
          result.args.forEach((arg) => {
            setValue(
              `${arg.name}`,
              !arg.value ? `${arg.default_value}` : `${arg.value}`,
            );
          });

          setMessageLaunchLoaded('');
        }

        if (result.status.code === 'OK') {
          // launch file does not have required arguments and will be loaded automatically
          // we need to trigger a launch update and close the modal

          setMessageLaunchLoaded('Launch file loaded successfully');
          setSelectedLaunch(null);

          // update node and launch list
          // rosCtx.updateNodeList(provider(.name()));
          // rosCtx.updateLaunchList(provider.name());
          logCtx.success(
            `Launch file [${getFileName(path)}] loaded`,
            `File: ${path}`,
          );

          // nothing else to do return
          onLaunchCallback();
          return;
        }

        if (result.status.code === 'ERROR') {
          setMessageLaunchLoaded(result.status.msg);
          logCtx.error(
            `Error on load "${getFileName(path)}"`,
            `Error message: ${result.status.msg}`,
          );
        }

        setOpen(true);
      } else {
        logCtx.error(
          `The provider [${selectedProvider}] does not support [launchLoadFile]`,
          'Please check your provider configuration',
        );
      }
    },
    // Do not include [logCtx] or [rosCtx] as dependency, because it will cause infinity loops
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [
      rosCtx.initialized,
      rosCtx.providers,
      selectedProvider,
      setValue,
      // rosCtx.updateLaunchList,
    ],
  );

  // The user clicked on launch, fill arguments a make a request to provider
  const launchSelectedFile = useCallback(async () => {
    if (!selectedLaunch) return;

    const provider = rosCtx.getProviderById(selectedProvider);
    if (!provider || !provider.isAvailable()) return;

    if (provider.launchLoadFile) {
      /*
       * ros_package -
       * launch - Launch file in the package path.
       * path - if set, this will be used instead of package/launch
       * @param {LaunchArgument[]} args - Arguments to load the launch file.
       * @param {boolean} force_first_file - If True, use first file if more than one was found in the package.
       * @param {boolean} request_args - If True, the launch file will not be loaded, only launch arguments are requested.
       * masteruri - Starts nodes of this file with specified ROS_MASTER_URI.
       * host - Start nodes of this file on specified host.
       * */
      const rosPackage = ''; // ROS package name.
      const launch = ''; // Launch file in the package path.
      const path = selectedLaunch.paths[0];
      const forceFirstFile = true;
      const requestArgs = false;

      // fill arguments
      const args = [];

      Object.entries(getValues()).forEach(([argName, argValue]) => {
        args.push(new LaunchArgument(argName, argValue));
      });

      const request = new LaunchLoadRequest(
        rosPackage,
        launch,
        path,
        args,
        forceFirstFile,
        requestArgs,
        provider.rosState.masteruri,
        provider.host(),
      );

      const resultLaunchLoadFile = await provider.launchLoadFile(request);

      if (!resultLaunchLoadFile) {
        logCtx.error(
          `Invalid response for [launchLoadFile], check DAEMON screen output`,
          'Please check your provider configuration',
        );
      } else if (resultLaunchLoadFile.status.code === 'OK') {
        setOpen(false);
        logCtx.success(
          `Launch file [${getFileName(path)}] loaded`,
          `File: ${path}`,
        );
      } else if (resultLaunchLoadFile.status.code === 'PARAMS_REQUIRED') {
        setMessageLaunchLoaded('Please fill all arguments');
      } else {
        setMessageLaunchLoaded(
          `Could not load file: ${resultLaunchLoadFile.status.msg}`,
        );
        logCtx.error(
          `Could not load file: "${path}"`,
          `Error message: ${resultLaunchLoadFile.status.msg}`,
        );
      }
    } else {
      logCtx.error(
        `The provider [${selectedProvider}] does not support [launchLoadFile]`,
        'Please check your provider configuration',
      );
    }

    // clear launch objects
    setSelectedLaunch(null);
    setSelectedLaunchFile(null);

    onLaunchCallback();

    // trigger node's update (will force a reload using useEffect hook)
    // rosCtx.updateNodeList(provider.name());
    // rosCtx.updateLaunchList(provider.name());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [getValues, selectedLaunch, selectedProvider, setSelectedLaunchFile]);

  // Request file and load arguments
  useEffect(() => {
    getLaunchFile(selectedLaunchFile.path);
    console.log(`set launch file to ${JSON.stringify(selectedLaunchFile)}`);
  }, [getLaunchFile, selectedLaunchFile]);

  const handleClose = (event, reason) => {
    if (reason && reason === 'backdropClick') return;
    setOpen(false);
  };

  return (
    <Dialog
      open={open}
      onClose={handleClose}
      scroll="paper"
      PaperComponent={DraggablePaper}
      aria-labelledby="draggable-dialog-title"
    >
      <DialogTitle style={{ cursor: 'move' }} id="draggable-dialog-title">
        Launch file
      </DialogTitle>
      <DialogContent>
        {selectedLaunch && (
          <Stack>
            <Tag color="info" text={selectedLaunch.paths[0]} wrap />
            <Stack>
              {selectedLaunch.args.map((arg) => {
                return (
                  <Controller
                    key={arg.name}
                    name={arg.name}
                    control={control}
                    render={({ field: { onChange, value } }) => (
                      <TextField
                        id={arg.name}
                        label={arg.name}
                        defaultValue={!value ? '' : value}
                        variant="filled"
                        size="small"
                        onChange={onChange}
                        focused
                      />
                    )}
                  />
                );
              })}
            </Stack>
          </Stack>
        )}
        {messageLaunchLoaded && messageLaunchLoaded.length > 0 && (
          // Prevent display issues when long path files are returned
          <Alert severity="warning" style={{ minWidth: 0 }}>
            <AlertTitle>
              {messageLaunchLoaded.replaceAll('/', ' / ')}
            </AlertTitle>
          </Alert>
        )}
      </DialogContent>
      <DialogActions>
        <Button
          autoFocus
          onClick={() => {
            setOpen(false);
            setSelectedLaunch(null);
          }}
        >
          Cancel
        </Button>
        <Button
          color="success"
          variant="contained"
          onClick={launchSelectedFile}
        >
          Load
        </Button>
      </DialogActions>
    </Dialog>
  );
}

LaunchFileModal.defaultProps = { onLaunchCallback: () => {} };

LaunchFileModal.propTypes = {
  selectedProvider: PropTypes.string.isRequired,
  selectedLaunchFile: PropTypes.object.isRequired,
  setSelectedLaunchFile: PropTypes.func.isRequired,
  onLaunchCallback: PropTypes.func,
};

export default LaunchFileModal;
