import BorderColorIcon from "@mui/icons-material/BorderColor";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InputIcon from "@mui/icons-material/Input";
import { Autocomplete, Box, ButtonGroup, IconButton, Stack, TextField, Tooltip } from "@mui/material";
import { forwardRef, useCallback, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { LAUNCH_FILE_EXTENSIONS, SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { getFileExtension, getFileName, PathItem, RosPackage } from "@/renderer/models";
import { LAYOUT_TABS } from "@/renderer/pages/NodeManager/layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "@/renderer/pages/NodeManager/layout/events";
import LaunchFileModal from "../LaunchFileModal/LaunchFileModal";
import Tag from "../UI/Tag";
import TreeDirectory from "./TreeDirectory";
import { TPackageItemsTree, TPackageTree, TPackageTreeItem } from "./types";

/**
 * Sorting function used for comparing two package items (files/directories)
 */
function comparePackageItems(a: RosPackage, b: RosPackage): number {
  if (a.path < b.path) {
    return -1;
  }
  if (a.path > b.path) {
    return 1;
  }
  return 0;
}

interface PackageExplorerProps {
  packageList: RosPackage[];
  selectedProvider?: string;
  reloadPackage: number;
}

const PackageExplorer = forwardRef<HTMLDivElement, PackageExplorerProps>(function PackageExplorer(props, ref) {
  const { packageList = [], selectedProvider = "", reloadPackage = 0 } = props;

  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [launchFileHistory, setLaunchFileHistory] = useLocalStorage<PathItem[]>(
    "PackageExplorer:launchFileHistory",
    []
  );

  const [selectedFile, setSelectedFile] = useState<PathItem | undefined>(undefined);
  const [selectedLaunchFile, setSelectedLaunchFile] = useState<PathItem | undefined>();
  const [selectedPackage, setSelectedPackage] = useState<RosPackage | null>(null);

  const [packageListFiltered, setPackageListFiltered] = useState<RosPackage[]>([]);
  const [packageItemsTree, setPackageItemsTree] = useState<TPackageItemsTree>({});

  const [packageItemList, setPackageItemList] = useState<PathItem[]>([]);
  const [ignoringNonRelevantPackageFiles, setIgnoringNonRelevantPackageFiles] = useState<boolean>(false);

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
  }, [settingsCtx.changed]);

  useEffect(() => {
    handleOnSelectPackage(selectedPackage);
  }, [reloadPackage]);

  /**
   * Keep history of latest launched files.
   */
  const onLaunchCallback = useCallback(() => {
    const provider = rosCtx.getProviderById(selectedProvider, true);
    if (!provider) {
      return;
    }
    setLaunchFileHistory((prevHistory: PathItem[]) => {
      const launchFile = selectedLaunchFile;
      if (launchFile) {
        // Separate the history of the selected host from the rest.
        const hostHistory = prevHistory.filter((file) => file.host === provider.host());
        const otherHistory = prevHistory.filter((file) => file.host !== provider.host());
        // Find if the launch file was already in the host history.
        const foundIdx = hostHistory.findIndex((file) => file.path === launchFile.path);
        if (foundIdx > -1) {
          // Delete the old history entry.
          hostHistory.splice(foundIdx, 1);
        }
        // Add the currently launched file to the front of the array.
        hostHistory.unshift(launchFile);
        // Cap host history length and return the merged histories.
        // TODO: Make the history length a parameter.
        return [...hostHistory.slice(0, settingsCtx.get("launchHistoryLength") as number), ...otherHistory];
      }
      return prevHistory;
    });
    // inform host panel tab about loaded launch file
    emitCustomEvent(EVENT_OPEN_COMPONENT, eventOpenComponent(LAYOUT_TABS.NODES, "default"));
  }, [rosCtx, selectedProvider, setLaunchFileHistory, selectedLaunchFile, settingsCtx]);

  /**
   * Reset all states considering launch file history.
   */
  const updateStates = useCallback(() => {
    const provider = rosCtx.getProviderById(selectedProvider, true);
    let hostLaunchFileHistory: PathItem[] = [];
    if (provider) {
      hostLaunchFileHistory = launchFileHistory.filter((file) => file.host === provider.host());
    }
    if (selectedPackage) return;
    const pit: TPackageItemsTree = hostLaunchFileHistory.reduce((prev, curr) => {
      prev[curr.name as string] = [
        {
          name: curr.name,
          children: [],
          file: curr,
          isDirectory: false,
        } as TPackageTreeItem,
      ];
      return prev;
    }, {});
    setPackageItemsTree(pit);
    setPackageItemList(hostLaunchFileHistory);
    setIgnoringNonRelevantPackageFiles(false);

    const sortedPackages = packageList.sort(comparePackageItems);
    setPackageListFiltered(sortedPackages);
  }, [packageList, rosCtx, selectedProvider, selectedPackage, launchFileHistory]);

  // Reset states upon packageList or history modification.
  useEffect(() => {
    updateStates();
    setSelectedFile(undefined);
  }, [packageList, selectedPackage, launchFileHistory, updateStates]);

  /**
   * Callback function when a package is selected.
   */
  async function handleOnSelectPackage(newSelectedPackage: RosPackage | null): Promise<void> {
    if (!newSelectedPackage) return;

    const packagePath = newSelectedPackage.path;
    const packageName = newSelectedPackage.name;

    if (!packagePath) return;
    if (!selectedProvider) return;
    if (selectedProvider.length === 0) return;

    const provider = rosCtx.getProviderById(selectedProvider, false);
    if (!provider || !provider.getPathList) return;

    let fl: PathItem[] = await provider.getPathList(`${packagePath}/`);

    // if file list is too large, we will have render problems
    // filter files to consider only relevant ones
    // try to filter/ignore unnecessary files
    fl = fl.filter((f) => {
      // ignore temporal directories
      // TODO: Make a setting parameter for this
      if (f.path.includes("/node_modules/")) return false;
      if (f.path.includes("/build/")) return false;
      if (f.path.includes("/__pycache__/")) return false;
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
        return ["launch", "yaml", "md", "h", "hpp", "c", "cpp", "py", "xml", "txt", "sdf", "config", "cfg"].includes(
          fileExtension
        );
      });
    } else {
      setIgnoringNonRelevantPackageFiles(false);
    }

    const pathItemMap = new Map<string, PathItem>();

    // Add extra properties to file object.
    const itemList: PathItem[] = fl.map((f) => {
      f.name = getFileName(f.path);
      f.package = packageName;
      // remove the package path from the file path
      // replace the file name by file id, to prevent name collisions in subfolders
      f.relativePath = f.path.replace(`${packagePath}/`, "/").replace(f.name, f.id);
      pathItemMap.set(f.id, f);
      return f;
    });

    const packageTree: TPackageTreeItem[] = [];
    const level: TPackageTree = { packageTree };

    // create a tree structure
    // reference: https://stackoverflow.com/questions/57344694/create-a-tree-from-a-list-of-strings-containing-paths-of-files-javascript
    for (const item of itemList) {
      item.relativePath?.split("/").reduce((prev: TPackageTree, name, i, a) => {
        if (!prev[name]) {
          prev[name] = { packageTree: [] };

          if (pathItemMap.has(name)) {
            // file
            prev.packageTree.push({
              name: pathItemMap.get(name)?.name as string,
              children: [],
              file: pathItemMap.get(name),
              isDirectory: false,
            });
          } else {
            // directory
            prev.packageTree.push({
              name: name,
              children: prev[name].packageTree,
              file: undefined,
              isDirectory: true,
            });
          }
        } else if (i === a.length - 1) {
          // TODO: Allow duplicate filenames ?
          // Filenames should always be at the end of the array.
          // r.treeFile.push({ name, children: [] });
        }

        return prev[name];
      }, level);
    }

    const pit: TPackageItemsTree = {};
    // eslint-disable-next-line prefer-destructuring
    pit[`${packageName}`] = packageTree;

    setPackageItemsTree(pit);
    setPackageItemList(itemList);
  }

  /**
   * Callback function when the search box is used.
   */
  function searchCallback(searchText: string): void {
    if (!searchText || (searchText && searchText.length === 0)) {
      const sortedPackages = packageList.sort(comparePackageItems);
      setPackageListFiltered(sortedPackages);
      return;
    }

    const searchResult: RosPackage[] = [];
    const includedPackages: string[] = [];
    for (const p of packageList) {
      if (
        p.name.indexOf(searchText) !== -1 &&
        !includedPackages.includes(p.name) // prevent duplicates
      ) {
        searchResult.push(p);
        includedPackages.push(p.name);
      }
    }

    const sortedPackages = searchResult.sort(comparePackageItems);
    setPackageListFiltered(sortedPackages);
  }

  /**
   * Callback when files on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (itemId: string): void => {
      const callbackFile: PathItem | undefined = packageItemList.find((item) => item.id === itemId);
      if (callbackFile) {
        setSelectedFile(callbackFile);
      } else {
        setSelectedFile(undefined);
      }
    },
    [packageItemList]
  );

  function onEditFile(fileObj: PathItem | undefined, external: boolean): void {
    if (fileObj) {
      navCtx.openEditor(selectedProvider, fileObj.path, fileObj.path, null, [], external);
    }
  }

  /**
   * Callback when files are double-clicked by the user
   */
  const onFileDoubleClick = useCallback(
    (_label: string, itemId: string, _ctrlKey: boolean, shiftKey: boolean): void => {
      const callbackFile = packageItemList.find((item) => item.id === itemId);
      if (!callbackFile) return;

      // check if file is a launch
      const isFileLaunch = LAUNCH_FILE_EXTENSIONS.find((fe) => callbackFile.path.indexOf(fe) !== -1);
      if (isFileLaunch && shiftKey) {
        const packages = packageList.filter((item) => item.name === callbackFile.package);
        if (packages.length > 0) {
          // select package containing history file
          setPackageListFiltered(packageList);
          setSelectedPackage({
            name: callbackFile.package,
            path: packages[0].path,
          } as RosPackage);
          handleOnSelectPackage({
            name: callbackFile.package,
            path: packages[0].path,
          } as RosPackage);
          return;
        }
        logCtx.error(`package ${callbackFile.package} not found! Try to reload list.`);
        return;
      }
      if (isFileLaunch) {
        setSelectedLaunchFile({ ...callbackFile });
        setSelectedFile({ ...callbackFile });
        return;
      }

      // edit file using monaco editor panel
      onEditFile(callbackFile, shiftKey);
    },
    [packageItemList, packageList]
  );

  return (
    <Stack ref={ref}>
      <Stack>
        <Stack direction="row" justifyItems="expand" alignItems="center">
          <Autocomplete
            id="autocomplete-package-search"
            size="small"
            fullWidth={true}
            autoHighlight
            clearOnEscape
            disableListWrap
            handleHomeEndKeys={false}
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
                sx={{ fontSize: "inherit" }}
              />
            )}
            onChange={(_event, newSelectedPackage: RosPackage | null) => {
              if (!newSelectedPackage) {
                setSelectedPackage(null);
              }
              setSelectedPackage(newSelectedPackage);
              handleOnSelectPackage(newSelectedPackage);
            }}
            onInputChange={(_event, newInputValue) => {
              searchCallback(newInputValue);
            }}
            // isOptionEqualToValue={(option, value) => {
            //   return (
            //     value === undefined || value === '' || option.path === value.path
            //   );
            // }}
          />
          <ButtonGroup orientation="horizontal" aria-label="launch file control group">
            <Tooltip
              title="Edit File"
              placement="bottom"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <span>
                <IconButton
                  disabled={!selectedFile}
                  size="small"
                  aria-label="Edit File"
                  onClick={(event) => {
                    onEditFile(selectedFile, event.nativeEvent.shiftKey);
                  }}
                >
                  <BorderColorIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip
              title="Load"
              placement="bottom"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <span>
                <IconButton
                  disabled={
                    !(selectedFile && LAUNCH_FILE_EXTENSIONS.find((fe) => selectedFile.path.indexOf(fe) !== -1))
                  }
                  color="primary"
                  size="small"
                  aria-label="load"
                  onClick={() => {
                    setSelectedLaunchFile(selectedFile ? { ...selectedFile } : undefined);
                  }}
                >
                  <InputIcon fontSize="inherit" />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip
              title="Copy absolute path"
              placement="bottom"
              enterDelay={tooltipDelay}
              enterNextDelay={tooltipDelay}
              disableInteractive
            >
              <span>
                <IconButton
                  disabled={!selectedFile?.path}
                  size="small"
                  aria-label="copy"
                  onClick={() => {
                    if (selectedFile?.path) {
                      navigator.clipboard.writeText(selectedFile.path);
                      logCtx.success(`${selectedFile.path} copied!`);
                    }
                  }}
                >
                  <ContentCopyIcon fontSize="inherit" />
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
            if (e.key === "Delete") {
              // remove launch file from history
              setLaunchFileHistory((prevHistory) => {
                if (selectedFile) {
                  return prevHistory.filter((file) => file.id !== selectedFile.id);
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
              onNodeSelect={(itemId: string) => handleSelect(itemId)}
              onFileDoubleClick={(label: string, itemId: string, ctrlKey: boolean, shiftKey: boolean) =>
                onFileDoubleClick(label, itemId, ctrlKey, shiftKey)
              }
            />
          )}
        </Box>
      </Stack>
      {selectedLaunchFile && (
        <LaunchFileModal
          selectedProvider={selectedProvider}
          selectedLaunchFile={selectedLaunchFile}
          setSelectedLaunchFile={setSelectedLaunchFile}
          onLaunchCallback={onLaunchCallback}
        />
      )}
    </Stack>
  );
});
export default PackageExplorer;
