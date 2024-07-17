import {
  Autocomplete,
  Box,
  ButtonGroup,
  IconButton,
  Stack,
  TextField,
  Tooltip,
} from '@mui/material';
import PropTypes from 'prop-types';
import {
  useCallback,
  useContext,
  useEffect,
  useReducer,
  useState,
} from 'react';

import BorderColorIcon from '@mui/icons-material/BorderColor';
import InputIcon from '@mui/icons-material/Input';
import { emitCustomEvent } from 'react-custom-events';

import useLocalStorage from '../../hooks/useLocalStorage';
import Tag from '../UI/Tag';

import { RosContext } from '../../context/RosContext';
import {
  LAUNCH_FILE_EXTENSIONS,
  SettingsContext,
} from '../../context/SettingsContext';

import {
  LAYOUT_TABS,
  LAYOUT_TAB_SETS,
  LayoutTabConfig,
} from '../../pages/NodeManager/layout';

import { getBaseName, getFileExtension, getFileName } from '../../models';

import FileEditorPanel from '../../pages/NodeManager/panels/FileEditorPanel';
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from '../../utils/events';
import LaunchFileModal from '../LaunchFileModal/LaunchFileModal';
import TreeDirectory from './TreeDirectory';
import LoggingContext from '../../context/LoggingContext';

/**
 * Sorting function used for comparing two package objects
 */
const comparePackages = (a, b) => {
  if (a.name < b.name) {
    return -1;
  }
  if (a.name > b.name) {
    return 1;
  }
  return 0;
};

/**
 * Sorting function used for comparing two package items (files/directories)
 */
const comparePackageItems = (a, b) => {
  if (a.path < b.path) {
    return -1;
  }
  if (a.path > b.path) {
    return 1;
  }
  return 0;
};

function PackageExplorer({ packageList, selectedProvider }) {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [, forceUpdate] = useReducer((x) => x + 1, 0);
  const tooltipDelay = settingsCtx.get('tooltipEnterDelay');

  const [launchFileHistory, setLaunchFileHistory] = useLocalStorage(
    'PackageExplorer:launchFileHistory',
    [],
  );

  const [selectedFile, setSelectedFile] = useState(null);
  const [selectedLaunchFile, setSelectedLaunchFile] = useState(null);
  const [selectedPackage, setSelectedPackage] = useState(null);

  const [packageListFiltered, setPackageListFiltered] = useState([]);
  const [packageItemsTree, setPackageItemsTree] = useState({});

  const [packageItemList, setPackageItemList] = useState([]);
  const [ignoringNonRelevantPackageFiles, setIgnoringNonRelevantPackageFiles] =
    useState(false);

  /**
   * Keep history of latest launched files.
   */
  const onLaunchCallback = useCallback(() => {
    const provider = rosCtx.getProviderById(selectedProvider);
    if (!provider) {
      return;
    }
    setLaunchFileHistory((prevHistory) => {
      const launchFile = selectedLaunchFile;
      if (launchFile) {
        // Separate the history of the selected host from the rest.
        const hostHistory = prevHistory.filter(
          (file) => file.host === provider.host(),
        );
        const otherHistory = prevHistory.filter(
          (file) => file.host !== provider.host(),
        );
        // Find if the launch file was already in the host history.
        const foundIdx = hostHistory.findIndex(
          (file) => file.path === launchFile.path,
        );
        if (foundIdx > -1) {
          // Delete the old history entry.
          hostHistory.splice(foundIdx, 1);
        }
        // Add the currently launched file to the front of the array.
        hostHistory.unshift(launchFile);
        // Cap host history length and return the merged histories.
        // TODO: Make the history length a parameter.
        return [
          ...hostHistory.slice(0, settingsCtx.get('launchHistoryLength')),
          ...otherHistory,
        ];
      }
      return prevHistory;
    });
    // inform host panel tab about loaded launch file
    emitCustomEvent(
      EVENT_OPEN_COMPONENT,
      eventOpenComponent(LAYOUT_TABS.NODES, 'default', {}),
    );
  }, [
    rosCtx,
    selectedProvider,
    setLaunchFileHistory,
    selectedLaunchFile,
    settingsCtx,
  ]);

  /**
   * Reset all states considering launch file history.
   */
  const updateStates = useCallback(() => {
    const provider = rosCtx.getProviderById(selectedProvider);
    let hostLaunchFileHistory = [];
    if (provider) {
      hostLaunchFileHistory = launchFileHistory.filter(
        (file) => file.host === provider.host(),
      );
    }
    if (selectedPackage) return;
    const pit = hostLaunchFileHistory.reduce((prev, curr) => {
      prev[curr.name] = {
        fileName: curr.name,
        children: [],
        file: curr,
      };
      return prev;
    }, {});
    setPackageItemsTree(pit);
    setPackageItemList(hostLaunchFileHistory);
    setIgnoringNonRelevantPackageFiles(false);

    const sortedPackages = packageList.sort(comparePackages);
    setPackageListFiltered(sortedPackages);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    packageList,
    rosCtx,
    selectedProvider,
    selectedPackage,
    launchFileHistory,
  ]);

  // Reset states upon packageList or history modification.
  useEffect(() => {
    updateStates();
    setSelectedFile(null);
  }, [packageList, selectedPackage, launchFileHistory, updateStates]);

  /**
   * Callback function when a package is selected.
   */
  const handleOnSelectPackage = useCallback(
    async (newSelectedPackage) => {
      if (!newSelectedPackage) return;

      const packagePath = newSelectedPackage.path;
      const packageName = newSelectedPackage.name;

      if (!packagePath) return;
      if (!selectedProvider) return;
      if (selectedProvider.length === 0) return;

      const provider = rosCtx.getProviderById(selectedProvider);
      if (!provider || !provider.getPathList) return;

      let fl = await provider.getPathList(`${packagePath}/`);

      // if file list is too large, we will have render problems
      // filter files to consider only relevant ones
      // try to filter/ignore unnecessary files
      fl = fl.filter((f) => {
        // ignore temporal directories
        // TODO: Make a setting parameter for this
        if (f.path.includes('/node_modules/')) return false;
        if (f.path.includes('/build/')) return false;
        if (f.path.includes('/__pycache__/')) return false;
        return true;
      });

      // if list of files still too large, keep only "relevant" file extensions
      // TODO: Do we need a setting for this number?
      if (fl.length > 100) {
        setIgnoringNonRelevantPackageFiles(true);

        // try to filter/ignore unnecessary files
        fl = fl.filter((f) => {
          // check file extension for "interesting" files
          const fileExtension = getFileExtension(f.path);
          return [
            'launch',
            'yaml',
            'md',
            'h',
            'hpp',
            'c',
            'cpp',
            'py',
            'xml',
            'txt',
            'sdf',
            'config',
            'cfg',
          ].includes(fileExtension);
        });
      } else {
        setIgnoringNonRelevantPackageFiles(false);
      }

      const pathItemMap = new Map('', '');

      // Add extra properties to file object.
      const itemList = fl.map((f) => {
        f.name = getFileName(f.path);
        f.package = packageName;
        // remove the package path from the file path
        // replace the file name by file id, to prevent name collisions in subfolders
        f.relativePath = f.path
          .replace(`${packagePath}/`, '/')
          .replace(f.name, f.id);
        pathItemMap.set(f.id, f);
        return f;
      });

      itemList.sort(comparePackageItems);

      const treeFile = [];
      const level = { treeFile };

      // create a tree structure
      // reference: https://stackoverflow.com/questions/57344694/create-a-tree-from-a-list-of-strings-containing-paths-of-files-javascript
      itemList.forEach((item) => {
        item.relativePath.split('/').reduce((r, name, i, a) => {
          if (!r[name]) {
            r[name] = { treeFile: [] };

            if (pathItemMap.has(name)) {
              // file
              r.treeFile.push({
                fileName: pathItemMap.get(name).name,
                children: r[name].treeFile,
                file: pathItemMap.get(name),
              });
            } else {
              // directory
              r.treeFile.push({
                directoryName: name,
                children: r[name].treeFile,
                file: null,
              });
            }
          } else if (i === a.length - 1) {
            // TODO: Allow duplicate filenames ?
            // Filenames should always be at the end of the array.
            // r.treeFile.push({ name, children: [] });
          }

          return r[name];
        }, level);
      });

      const pit = {};
      // eslint-disable-next-line prefer-destructuring
      pit[`${packageName}`] = treeFile[0];

      setPackageItemsTree(pit);
      setPackageItemList(itemList);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rosCtx.getProviderById, selectedProvider, setSelectedPackage],
  );

  /**
   * Callback function when the search box is used.
   */
  const searchCallback = useCallback(
    (searchText) => {
      if (!searchText || (searchText && searchText.length === 0)) {
        const sortedPackages = packageList.sort(comparePackages);
        setPackageListFiltered(sortedPackages);
        return;
      }

      const searchResult = [];
      const includedPackages = [];
      packageList.forEach((p) => {
        if (
          p.name.indexOf(searchText) !== -1 &&
          !includedPackages.includes(p.name) // prevent duplicates
        ) {
          searchResult.push(p);
          includedPackages.push(p.name);
        }
      });

      const sortedPackages = searchResult.sort(comparePackages);
      setPackageListFiltered(sortedPackages);
    },
    [packageList],
  );

  /**
   * Callback when files on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (event, itemId) => {
      const callbackFile = packageItemList.find((item) => item.id === itemId);
      if (callbackFile) {
        setSelectedFile(callbackFile);
      } else {
        setSelectedFile(null);
      }
    },
    [packageItemList],
  );

  const onEditFile = useCallback(
    (fileObj) => {
      const id = `editor-${selectedProvider}-${fileObj.path}`;
      emitCustomEvent(
        EVENT_OPEN_COMPONENT,
        eventOpenComponent(
          id,
          getBaseName(fileObj.name),
          <FileEditorPanel
            tabId={id}
            providerId={selectedProvider}
            referenceFilePath={null}
            currentFilePath={fileObj.path}
            rootFilePath={fileObj.path}
          />,
          true,
          LAYOUT_TAB_SETS[settingsCtx.get('editorOpenLocation')],
          new LayoutTabConfig(false, 'editor'),
        ),
      );
    },
    [selectedProvider],
  );

  /**
   * Callback when files are double-clicked by the user
   */
  const onFileDoubleClick = useCallback(
    (label, itemId, ctrlKey, shiftKey, altKey) => {
      const callbackFile = packageItemList.find((item) => item.id === itemId);
      if (!callbackFile) return;

      // check if file is a launch
      const isFileLaunch = LAUNCH_FILE_EXTENSIONS.find(
        (fe) => callbackFile.path.indexOf(fe) !== -1,
      );
      if (isFileLaunch && shiftKey) {
        const packages = packageList.filter(
          (item) => item.name === callbackFile.package,
        );
        if (packages.length > 0) {
          // select package containing history file
          setPackageListFiltered(packageList);
          setSelectedPackage({
            name: callbackFile.package,
            path: packages[0].path,
          });
          handleOnSelectPackage({
            name: callbackFile.package,
            path: packages[0].path,
          });
          return;
        } else {
          logCtx.error(`package ${callbackFile.package} not found! Try to reload list.`)
          return;
        }
      }
      if (isFileLaunch) {
        setSelectedLaunchFile({ ...callbackFile });
        setSelectedFile({ ...callbackFile });
        return;
      }

      // edit file using monaco editor panel
      onEditFile(callbackFile);
    },
    [packageItemList, onEditFile, packageList, handleOnSelectPackage],
  );

  return (
    <>
      <Stack>
        <Stack direction="row" justifyItems="expand" alignItems="center">
          <Autocomplete
            id="autocomplete-package-search"
            size="small"
            fullWidth={true}
            autoHighlight
            clearOnEscape
            disableListWrap
            noOptionsText="Package not found"
            options={packageListFiltered}
            getOptionLabel={(option) => option.name}
            isOptionEqualToValue={(option, value) => option.name === value.name}
            // sx={{ flexGrow: 1 }}
            // This prevents warnings on invalid autocomplete values
            value={selectedPackage}
            renderInput={(params) => (
              <TextField
                {...params}
                label="Search Package"
                color="info"
                variant="outlined"
                margin="dense"
                size="small"
                // autoFocus
                sx={{ fontSize: 'inherit' }}
              />
            )}
            onChange={(event, newSelectedPackage) => {
              if (!newSelectedPackage) {
                setSelectedPackage(null);
              }
              setSelectedPackage(newSelectedPackage);
              handleOnSelectPackage(newSelectedPackage);
            }}
            onInputChange={(event, newInputValue) => {
              searchCallback(newInputValue);
            }}
            // isOptionEqualToValue={(option, value) => {
            //   return (
            //     value === undefined || value === '' || option.path === value.path
            //   );
            // }}
          />
          <ButtonGroup
            orientation="horizontal"
            aria-label="launch file control group"
          >
            <Tooltip
              title="Edit File"
              placement="left"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
              <span>
                <IconButton
                  disabled={!selectedFile}
                  size="small"
                  aria-label="Edit File"
                  onClick={() => {
                    onEditFile(selectedFile);
                  }}
                >
                  <BorderColorIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip
              title="Load"
              placement="left"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
            >
              <span>
                <IconButton
                  disabled={
                    !(
                      selectedFile &&
                      LAUNCH_FILE_EXTENSIONS.find(
                        (fe) => selectedFile.path.indexOf(fe) !== -1,
                      )
                    )
                  }
                  size="small"
                  aria-label="load"
                  onClick={() => {
                    setSelectedLaunchFile({ ...selectedFile });
                  }}
                >
                  <InputIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
          </ButtonGroup>
        </Stack>

        {ignoringNonRelevantPackageFiles && (
          <Tag
            key="ignore-non-relevant-packages"
            color="warning"
            title="Ignoring non-relevant package files"
            text="The folder contains too many files"
            wrap
          />
        )}
      </Stack>
      <Stack direction="row" height="100%" overflow="auto">
        <Box
          width="100%"
          // height="100%"
          overflow="auto"
          onKeyUp={(e) => {
            if (e.key === 'Delete') {
              // remove launch file from history
              setLaunchFileHistory((prevHistory) => {
                if (selectedFile) {
                  return prevHistory.filter(
                    (file) => file.id !== selectedFile.id,
                  );
                }
                return prevHistory;
              });
            }
          }}
        >
          {Object.keys(packageItemsTree).length > 0 && (
            <TreeDirectory
              selectedPackage={selectedPackage}
              packageItemsTree={packageItemsTree}
              onNodeSelect={handleSelect}
              onFileDoubleClick={onFileDoubleClick}
            />
          )}
        </Box>
        {/* <Box>
          <Paper elevation={2}>
            <ButtonGroup
              orientation="vertical"
              aria-label="launch file control group"
            >
              <Tooltip
                title="Edit File"
                placement="left"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <span>
                  <IconButton
                    disabled={!selectedFile}
                    size="medium"
                    aria-label="Edit File"
                    onClick={() => {
                      onEditFile(selectedFile);
                    }}
                  >
                    <BorderColorIcon fontSize="inherit" />
                  </IconButton>
                </span>
              </Tooltip>
              <Tooltip
                title="Load"
                placement="left"
                enterDelay={tooltipDelay}
                enterNextDelay={tooltipDelay}
              >
                <span>
                  <IconButton
                    disabled={
                      !(
                        selectedFile &&
                        LAUNCH_FILE_EXTENSIONS.find(
                          (fe) => selectedFile.path.indexOf(fe) !== -1,
                        )
                      )
                    }
                    size="medium"
                    aria-label="load"
                    onClick={() => {
                      setSelectedLaunchFile({ ...selectedFile });
                    }}
                  >
                    <InputIcon fontSize="inherit" />
                  </IconButton>
                </span>
              </Tooltip>
            </ButtonGroup>
          </Paper>
        </Box> */}
      </Stack>
      {selectedLaunchFile && (
        <LaunchFileModal
          selectedProvider={selectedProvider}
          selectedLaunchFile={selectedLaunchFile}
          setSelectedLaunchFile={setSelectedLaunchFile}
          onLaunchCallback={onLaunchCallback}
        />
      )}
    </>
  );
}

PackageExplorer.propTypes = {
  packageList: PropTypes.arrayOf(PropTypes.any).isRequired,
  selectedProvider: PropTypes.string.isRequired,
};

export default PackageExplorer;
