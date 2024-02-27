import DeleteIcon from '@mui/icons-material/Delete';
import MoreHorizIcon from '@mui/icons-material/MoreHoriz';
import {
  Alert,
  AlertTitle,
  Autocomplete,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  Stack,
  TextField,
  Typography,
} from '@mui/material';
import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useState } from 'react';

import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import useLocalStorage from '../../hooks/useLocalStorage';
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
  const [argHistory, setArgHistory] = useLocalStorage(
    'history:loadLaunchArgs',
    {},
  );
  const [lastOpenPath, setLastOpenPath] = useLocalStorage('lastOpenPath', '');
  const [currentArgs, setCurrentArgs] = useState([]);

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
          const argList = [];
          result.args.forEach((arg) => {
            const argValue = !arg.value ? arg.default_value : arg.value;
            let historyList = argHistory[arg.name];
            if (historyList === undefined) {
              historyList = [];
              // historyList = options.filter((value) => value !== argValue);
            }
            argList.push({
              name: arg.name,
              value: argValue,
              history: historyList,
              choices: arg.choices,
            });
          });
          setCurrentArgs(argList);

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
      argHistory,
      rosCtx.initialized,
      rosCtx.providers,
      selectedProvider,
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

      currentArgs.forEach((arg) => {
        console.log(`arg.name: ${arg.name}: ${arg.value}`);
        args.push(new LaunchArgument(arg.name, arg.value));
        // update history
        let hList = argHistory[arg.name];
        if (hList !== undefined) {
          hList = hList.filter((value) => value !== arg.value);
        } else {
          hList = [];
        }
        hList.unshift(arg.value);
        hList = hList.slice(0, 10);
        argHistory[arg.name] = hList;
      });
      setArgHistory(argHistory);

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
  }, [currentArgs, selectedLaunch, selectedProvider, setSelectedLaunchFile]);

  // Request file and load arguments
  useEffect(() => {
    getLaunchFile(selectedLaunchFile.path);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedLaunchFile]);

  const handleClose = (event, reason) => {
    if (reason && reason === 'backdropClick') return;
    setOpen(false);
  };

  const deleteHistoryOption = useCallback(
    (argName, option) => {
      setCurrentArgs(
        currentArgs.map((arg) => {
          if (arg.name === argName) {
            arg.history = arg.history.filter((value) => value !== option);
          }
          return arg;
        }),
      );
      const newHistory = {};
      // eslint-disable-next-line no-restricted-syntax
      for (const [key, value] of Object.entries(argHistory)) {
        if (key === argName) {
          newHistory[key] = value.filter((val) => val !== option);
        } else {
          newHistory[key] = value;
        }
      }
      setArgHistory(newHistory);
    },
    [argHistory, currentArgs, setArgHistory, setCurrentArgs],
  );

  const openFileDialog = useCallback(
    async (argName) => {
      let defaultPath = lastOpenPath;
      if (argName.startsWith('/')) {
        defaultPath = argName;
      }
      const filePath = await window.electronAPI.openFile(defaultPath);
      if (filePath) {
        setLastOpenPath(filePath);
        setCurrentArgs(
          currentArgs.map((arg) => {
            if (arg.name === argName) {
              arg.value = filePath;
            }
            return arg;
          }),
        );
      }
    },
    [currentArgs],
  );

  const isPathParam = (name, value) => {
    if (!value) {
      // offer path select dialog on empty string
      return true;
    }
    const lValue = value.toLocaleLowerCase();
    if (['true', 'false'].includes(lValue)) {
      return false;
    }
    const lName = name.toLocaleLowerCase();
    if (lName.includes('frame')) {
      return false;
    }
    if (lName.includes('[')) {
      return false;
    }
    return Number.isNaN(Number(value));
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
              {currentArgs.map((arg) => {
                const options = arg.choices ? arg.choices : arg.history;
                return (
                  <Stack key={`stack-launch-load-${arg.name}`} direction="row">
                    {options.length > 1 && (
                      // show autocomplete only if we have multiple option
                      <Autocomplete
                        key={`autocomplete-launch-load-${arg.name}`}
                        size="small"
                        fullWidth
                        autoHighlight
                        clearOnEscape
                        disableListWrap
                        // noOptionsText="Package not found"
                        options={arg.choices ? arg.choices : arg.history}
                        getOptionLabel={(option) => option}
                        // This prevents warnings on invalid autocomplete values
                        value={arg.value}
                        renderInput={(params) => (
                          <TextField
                            {...params}
                            label={arg.name}
                            color="info"
                            variant="outlined"
                            margin="dense"
                            size="small"
                            autoFocus
                          />
                        )}
                        renderOption={(props, option) => (
                          <Stack {...props} direction="row">
                            <Typography width="stretch">{option}</Typography>
                            <IconButton
                              component="label"
                              onClick={(event) => {
                                deleteHistoryOption(arg.name, option);
                                event.stopPropagation();
                              }}
                            >
                              <DeleteIcon fontSize="1em" />
                            </IconButton>
                          </Stack>
                        )}
                        onChange={(event, newArgValue) => {
                          setCurrentArgs(
                            currentArgs.map((item) => {
                              if (item.name === arg.name) {
                                item.value = newArgValue;
                              }
                              return item;
                            }),
                          );
                        }}
                        onInputChange={(event, newInputValue) => {
                          setCurrentArgs(
                            currentArgs.map((item) => {
                              if (item.name === arg.name) {
                                item.value = newInputValue;
                              }
                              return item;
                            }),
                          );
                        }}
                        isOptionEqualToValue={(option, value) => {
                          return (
                            value === undefined ||
                            value === '' ||
                            option.path === value.path
                          );
                        }}
                        onWheel={(event) => {
                          // scroll through the options using mouse wheel
                          const options = arg.choices
                            ? arg.choices
                            : arg.history;
                          let newIndex = -1;
                          options.forEach((value, index) => {
                            if (value === event.target.value) {
                              if (event.deltaY > 0) {
                                newIndex = index + 1;
                              } else {
                                newIndex = index - 1;
                              }
                            }
                          });
                          if (newIndex < 0) newIndex = 0;
                          else if (newIndex > options.length - 1)
                            newIndex = options.length - 1;
                          setCurrentArgs(
                            currentArgs.map((item) => {
                              if (item.name === arg.name) {
                                item.value = options[newIndex];
                              }
                              return item;
                            }),
                          );
                        }}
                      />
                    )}
                    {options.length <= 1 && (
                      // we have no history, show only the text field
                      <TextField
                        id={`textfield-launch-load-${arg.name}`}
                        fullWidth
                        label={arg.name}
                        value={arg.value}
                        variant="outlined"
                        size="small"
                        onChange={(event) => {
                          console.log(
                            `onChange ${newArgValue} ${event.target.value}`,
                          );
                          setCurrentArgs(
                            currentArgs.map((item) => {
                              if (item.name === arg.name) {
                                item.value = event.target.value;
                              }
                              return item;
                            }),
                          );
                        }}
                      />
                    )}
                    {isPathParam(arg.name, arg.value) && (
                      <IconButton
                        component="label"
                        // sx={{
                        //   visibility: isPathParam(arg.name)
                        //     ? 'visible'
                        //     : 'hidden',
                        // }}
                        onClick={() => {
                          openFileDialog(arg.name);
                        }}
                      >
                        <MoreHorizIcon />
                      </IconButton>
                    )}
                  </Stack>
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
