import BorderColorIcon from "@mui/icons-material/BorderColor";
import ContentCopyIcon from "@mui/icons-material/ContentCopy";
import InputIcon from "@mui/icons-material/Input";
import RefreshIcon from "@mui/icons-material/Refresh";
import {
  Alert,
  AlertTitle,
  Autocomplete,
  Box,
  ButtonGroup,
  IconButton,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from "@mui/material";
import CircularProgress from "@mui/material/CircularProgress";
import React, { HTMLAttributes, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent, useCustomEventListener } from "react-custom-events";

import LaunchFileModal from "@/renderer/components/LaunchFileModal/LaunchFileModal";
import TreeDirectory from "@/renderer/components/PackageExplorer/TreeDirectory";
import { TPackageItemsTree, TPackageTree, TPackageTreeItem } from "@/renderer/components/PackageExplorer/types";
import { Tag } from "@/renderer/components/UI";
import { colorFromHostname } from "@/renderer/components/UI/Colors";
import LoggingContext from "@/renderer/context/LoggingContext";
import NavigationContext from "@/renderer/context/NavigationContext";
import { RosContext } from "@/renderer/context/RosContext";
import { LAUNCH_FILE_EXTENSIONS, SettingsContext } from "@/renderer/context/SettingsContext";
import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { getFileExtension, getFileName, PathItem, RosPackage } from "@/renderer/models";
import { ConnectionState } from "@/renderer/providers";
import { EventProviderState } from "@/renderer/providers/events";
import { EVENT_PROVIDER_STATE } from "@/renderer/providers/eventTypes";
import { LAYOUT_TABS } from "../layout";
import { EVENT_OPEN_COMPONENT, eventOpenComponent } from "../layout/events";

/**
 * Sorting function used for comparing two package items (files/directories)
 */
function comparePackageItems(a: ProviderPackage, b: ProviderPackage): number {
  if (a.providerId < b.providerId) {
    return -1;
  }
  if (a.providerId > b.providerId) {
    return 1;
  }
  if (a.rosPackage.path < b.rosPackage.path) {
    return -1;
  }
  if (a.rosPackage.path > b.rosPackage.path) {
    return 1;
  }
  return 0;
}

export class ProviderPackage {
  id: string;

  providerId: string;
  providerName: string;

  rosPackage: RosPackage;

  constructor(providerId: string, providerName: string, rosPackage: RosPackage) {
    this.id = `${providerId}-${rosPackage.name}`;
    this.providerId = providerId;
    this.providerName = providerName;
    this.rosPackage = rosPackage;
  }
}

export default function PackageExplorerPanel(): JSX.Element {
  const logCtx = useContext(LoggingContext);
  const navCtx = useContext(NavigationContext);
  const rosCtx = useContext(RosContext);
  const settingsCtx = useContext(SettingsContext);

  const [loading, setLoading] = useState(false);
  const [selectedFile, setSelectedFile] = useState<PathItem | undefined>(undefined);
  const [selectedLaunchFile, setSelectedLaunchFile] = useState<PathItem | undefined>();
  const [selectedPackage, setSelectedPackage] = useState<ProviderPackage | null>(null);

  const [packageList, setPackageList] = useState<ProviderPackage[]>([]);
  const [packageListFiltered, setPackageListFiltered] = useState<ProviderPackage[]>([]);
  const [showReloadButton, setShowReloadButton] = useState(false);
  const [ignoringNonRelevantPackageFiles, setIgnoringNonRelevantPackageFiles] = useState<boolean>(false);
  const [packageItemsTree, setPackageItemsTree] = useState<TPackageItemsTree>({});
  const [packageItemList, setPackageItemList] = useState<PathItem[]>([]);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [backgroundColor, setBackgroundColor] = useState<string>(settingsCtx.get("backgroundColor") as string);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);

  const [launchFileHistory, setLaunchFileHistory] = useLocalStorage<PathItem[]>(
    "PackageExplorer:launchFileHistory",
    []
  );

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setBackgroundColor(settingsCtx.get("backgroundColor") as string);
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  async function updatePackageList(force: boolean = false): Promise<void> {
    if (!rosCtx.initialized) return;
    if (!rosCtx.providers) return;
    setLoading(true);
    // if providerId is valid we update only packages of this provider
    let newPackageList: ProviderPackage[] = [];

    for (const provider of rosCtx.providers) {
      if (provider.connectionState === ConnectionState.STATES.CONNECTED) {
        if (!provider.packages || force) {
          const pl = await provider.getPackageList(force);
          newPackageList = [
            ...newPackageList,
            ...pl.map((rosPkg) => new ProviderPackage(provider.id, provider.name(), rosPkg)),
          ];
        } else {
          newPackageList = [
            ...newPackageList,
            ...(provider.packages?.map((rosPkg) => new ProviderPackage(provider.id, provider.name(), rosPkg)) || []),
          ];
        }
      }
    }
    setPackageList(newPackageList.sort(comparePackageItems));
    if (!selectedPackage) {
      setPackageListFiltered(newPackageList);
    }
    setLoading(false);
  }

  // Update files when selected provider changes
  useEffect(() => {
    updatePackageList();
    setShowReloadButton(true);
  }, [rosCtx.initialized]);

  useCustomEventListener(EVENT_PROVIDER_STATE, (data: EventProviderState) => {
    if (data.newState === ConnectionState.STATES.CONNECTED) {
      updatePackageList();
    }
  });

  /**
   * Reset all states considering launch file history.
   */
  const updateStates = useCallback(() => {
    if (selectedPackage) return;
    const provPackageHistory: TPackageItemsTree = {};
    for (const prov of rosCtx.providers) {
      const provHistory = launchFileHistory.filter(
        (file) => file.host === prov.host() || file.providerName === prov.name()
      );
      if (provHistory.length > 0) {
        const pit: TPackageTreeItem[] = provHistory.map((curr) => {
          // update id of provider files
          curr.providerId = prov.id;
          curr.providerName = prov.name();
          return {
            name: curr.name,
            children: [],
            file: curr,
            isDirectory: false,
            appendPackageName: true,
          } as TPackageTreeItem;
        });
        provPackageHistory[`${prov.name()}`] = [
          {
            name: `${prov.name()}`,
            children: pit,
            file: undefined,
            isDirectory: false,
            appendPackageName: true,
          } as TPackageTreeItem,
        ];
      }
    }
    setPackageItemsTree(provPackageHistory);
    setPackageItemList(launchFileHistory);
    setIgnoringNonRelevantPackageFiles(false);

    const sortedPackages = packageList.sort(comparePackageItems);
    setPackageListFiltered(sortedPackages);
  }, [packageList, rosCtx, selectedPackage, launchFileHistory]);

  /**
   * Callback function when the search box is used.
   */
  function searchCallback(searchText: string): void {
    if (!searchText || (searchText && searchText.length === 0)) {
      setPackageListFiltered(packageList);
      return;
    }

    const searchResult: ProviderPackage[] = [];
    const includedPackages: string[] = [];
    for (const p of packageList) {
      if (
        p.rosPackage.name.indexOf(searchText) !== -1 &&
        !includedPackages.includes(p.id) // prevent duplicates
      ) {
        searchResult.push(p);
        includedPackages.push(p.id);
      }
    }

    setPackageListFiltered(searchResult);
  }

  // Reset states upon packageList or history modification.
  useEffect(() => {
    updateStates();
    setSelectedFile(undefined);
  }, [packageList, selectedPackage, launchFileHistory]);

  /**
   * Callback function when a package is selected.
   */
  async function handleOnSelectPackage(newSelectedPackage: ProviderPackage | null): Promise<void> {
    if (!newSelectedPackage) return;

    const providerId = newSelectedPackage.providerId;
    const packagePath = newSelectedPackage.rosPackage.path;
    const packageName = newSelectedPackage.rosPackage.name;

    if (!packagePath) return;

    const provider = rosCtx.getProviderById(providerId, false);
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
              appendPackageName: false,
            });
          } else {
            // directory
            prev.packageTree.push({
              name: name,
              children: prev[name].packageTree,
              file: undefined,
              isDirectory: false,
              appendPackageName: false,
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
      navCtx.openEditor(fileObj.providerId || "", fileObj.path, fileObj.path, null, [], external);
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
        const packages = packageList.filter(
          (item) => item.providerId === callbackFile.providerId && item.rosPackage.name === callbackFile.package
        );
        if (packages.length > 0) {
          // select package containing history file
          setPackageListFiltered(packageList);
          setSelectedPackage({
            rosPackage: { name: callbackFile.package, path: packages[0].rosPackage.path },
            providerId: callbackFile.providerId,
            providerName: callbackFile.providerName,
          } as ProviderPackage);
          handleOnSelectPackage({
            rosPackage: { name: callbackFile.package, path: packages[0].rosPackage.path },
            providerId: callbackFile.providerId,
            providerName: callbackFile.providerName,
          } as ProviderPackage);
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

  /**
   * Keep history of latest launched files.
   */
  const updateHistory = useCallback(() => {
    const provider = rosCtx.getProviderById(selectedLaunchFile?.providerId || "", true);
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
  }, [rosCtx, selectedLaunchFile, settingsCtx]);

  useEffect(() => {
    if (selectedLaunchFile) {
      updateHistory();
    }
  }, [selectedLaunchFile]);

  const createPackageSelector = useMemo(() => {
    return (
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
        // loading={loading}
        getOptionLabel={(option) => `${option.rosPackage.name} [${option.providerName}]`}
        isOptionEqualToValue={(option, value) => option.rosPackage.name === value.rosPackage.name}
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
            slotProps={{
              input: {
                ...params.InputProps,
                endAdornment: (
                  <React.Fragment>
                    {loading ? <CircularProgress color="inherit" size={20} /> : null}
                    {params.InputProps.endAdornment}
                  </React.Fragment>
                ),
              },
            }}
          />
        )}
        renderOption={(props, option) => {
          return (
            <Stack
              {...(props as HTMLAttributes<HTMLDivElement>)}
              key={option.id}
              direction="row"
              style={getHostStyle(option.providerName)}
            >
              <Tooltip title={`${option.rosPackage.name}`} placement="top" disableInteractive>
                <Typography
                  sx={{ ml: 0, fontWeight: "bold" }}
                  noWrap
                  // width="stretch"
                >
                  {option.rosPackage.name}
                </Typography>
              </Tooltip>
              <Typography color="grey" sx={{ ml: 1 }} noWrap>
                | {option.providerName}
              </Typography>
            </Stack>
          );
        }}
        onChange={(_event, newSelectedPackage: ProviderPackage | null) => {
          if (!newSelectedPackage) {
            setSelectedPackage(null);
          }
          setSelectedPackage(newSelectedPackage);
          handleOnSelectPackage(newSelectedPackage);
        }}
        onInputChange={(_event, newInputValue) => {
          searchCallback(newInputValue);
        }}
      />
    );
  }, [packageListFiltered, selectedPackage, loading]);

  const getHostStyle = useCallback(
    function getHostStyle(providerName: string): object {
      if (colorizeHosts) {
        const hColor = colorFromHostname(providerName);
        return {
          borderLeftStyle: "solid",
          borderLeftColor: hColor,
          borderLeftWidth: "0.6em",
          overflowWrap: "normal",
          // borderBottomStyle: "solid",
          // borderBottomColor: hColor,
          // borderBottomWidth: "0.6em",
        };
      }
      return { overflowWrap: "normal" };
    },
    [colorizeHosts]
  );

  return (
    <Box
      width="100%"
      height="100%"
      sx={{ backgroundColor: backgroundColor }}
      // paddingLeft="10px"
    >
      <Stack>
        <Stack direction="column" height="100%" width="100% ">
          <Stack direction="row" justifyItems="expand" alignItems="center">
            {createPackageSelector}
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
                    updatePackageList(true);
                  }}
                >
                  <RefreshIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            )}
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
                selectedPackage={selectedPackage?.rosPackage}
                providerName={selectedPackage?.providerName}
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
            selectedProvider={undefined}
            selectedLaunchFile={selectedLaunchFile}
            setSelectedLaunchFile={setSelectedLaunchFile}
            onLaunchCallback={() => {}}
          />
        )}
        {(!rosCtx.providers || rosCtx.providers.length === 0) && (
          <Alert severity="info" style={{ minWidth: 0, marginTop: 10 }}>
            <AlertTitle>No providers available</AlertTitle>
            Please connect to a ROS provider
          </Alert>
        )}
      </Stack>
    </Box>
  );
}
