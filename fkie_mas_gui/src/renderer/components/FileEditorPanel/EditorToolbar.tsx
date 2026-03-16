import ArrowBackIosNewIcon from "@mui/icons-material/ArrowBackIosNew";
import ArrowForwardIosIcon from "@mui/icons-material/ArrowForwardIos";
import CloudSyncOutlinedIcon from "@mui/icons-material/CloudSyncOutlined";
import SaveAltOutlinedIcon from "@mui/icons-material/SaveAltOutlined";
import UpgradeIcon from "@mui/icons-material/Upgrade";
import { CircularProgress, IconButton, Link, NativeSelect, Stack, Tooltip, Typography } from "@mui/material";
import { editor } from "monaco-editor";
import React, { ForwardedRef, useCallback, useEffect, useMemo, useState } from "react";

import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { getFileAbb, getFileName, LaunchIncludedFile } from "@/renderer/models";
import { fileFromUriPath } from "@/renderer/monaco/utils";
import { TFileRange, TLaunchArg } from "@/types";
import { colorFromHostname, OverflowMenu } from "../UI";

export type THistoryModel = {
  uriPath: string;
  range: TFileRange | null;
  launchArgs: TLaunchArg[];
};

interface EditorToolbarProps {
  refEl: ForwardedRef<HTMLDivElement>;
  providerName: string;
  packageName: string;
  rootFilePath: string;
  currentFileState: { name: string; requesting: boolean; path: string };
  activeModel: editor.ITextModel | null | undefined;
  activeModelDirty: boolean;
  historyModel: THistoryModel | undefined;
  includedFiles: LaunchIncludedFile[];
  modifiedFiles: string[];
  eventButton?: React.MouseEvent<HTMLDivElement, MouseEvent>;
  setEditorModel: (
    uriPath: string,
    range?: TFileRange | null,
    launchArgs?: TLaunchArg[],
    forceReload?: boolean,
    appendToHistory?: boolean
  ) => Promise<boolean>;
  saveModel: (editorModel: editor.ITextModel) => Promise<void>;
  reloadCurrentFile: () => void;
}

/**
 * Top toolbar for editor actions
 */
export function EditorToolbar(props: EditorToolbarProps): JSX.Element {
  const {
    refEl,
    providerName,
    packageName,
    rootFilePath,
    currentFileState,
    activeModel,
    activeModelDirty,
    historyModel = undefined,
    includedFiles,
    modifiedFiles,
    eventButton,
    setEditorModel,
    saveModel,
    reloadCurrentFile = () => {},
  } = props;

  const settingsCtx = useSettingsContext();
  const [historyModels, setHistoryModels] = useState<THistoryModel[]>([]);
  const [historyIndex, setHistoryIndex] = useState<number>(-1);
  const [tooltipDelay, setTooltipDelay] = useState<number>(settingsCtx.get("tooltipEnterDelay") as number);
  const [colorizeHosts, setColorizeHosts] = useState<boolean>(settingsCtx.get("colorizeHosts") as boolean);
  const [selectParentFiles, setSelectParentFiles] = useState<LaunchIncludedFile[]>([]);
  const modifiedFileSet = useMemo(() => new Set(modifiedFiles.map(fileFromUriPath)), [modifiedFiles]);
  const selectableFiles = useMemo(
    () => Array.from(new Set([rootFilePath, ...includedFiles.map((f) => f.inc_path)])),
    [rootFilePath, includedFiles]
  );

  useEffect(() => {
    setTooltipDelay(settingsCtx.get("tooltipEnterDelay") as number);
    setColorizeHosts(settingsCtx.get("colorizeHosts") as boolean);
  }, [settingsCtx.changed]);

  const addToHistory = useCallback(
    (model: THistoryModel) => {
      setHistoryModels((prev) => {
        if (historyIndex === -1) {
          if (prev[prev.length - 1]?.uriPath === model.uriPath) {
            return prev;
          }
          return [...prev, model];
        }
        if (prev[historyIndex + 1]?.uriPath === model.uriPath) {
          return prev;
        }
        return [...prev.slice(0, historyIndex + 1), model, ...prev.slice(historyIndex + 1)];
      });
      setHistoryIndex((prev) => prev + 1);
    },
    [historyIndex]
  );

  useEffect(() => {
    if (!historyModel) return;
    addToHistory(historyModel);
  }, [historyModel]);

  useEffect(() => {
    if (eventButton?.button === 3) {
      openPrevModel();
    } else if (eventButton?.button === 4) {
      openNextModel();
    }
  }, [eventButton]);

  useEffect(() => {
    // update parent files
    const path = fileFromUriPath(activeModel?.uri.path || "");
    if (path) {
      const parentPaths = includedFiles.filter((item) => {
        if (path === item.inc_path) {
          return true;
          // check args to select the correct file, if the same file included twice
          // skipped: args are not always fully resolved, especially if they contain find-pkg-share
          // return equalLaunchArgs(currentLaunchArgs, item.args || []);
        }
        return false;
      });
      setSelectParentFiles(parentPaths);
    }
  }, [activeModel, includedFiles]);

  const setFromHistory = useCallback(
    (index: number) => {
      const model = historyModels.at(index);
      if (model && model?.uriPath !== activeModel?.uri.path)
        setEditorModel(model.uriPath, model.range, model.launchArgs, false, false);
    },
    [activeModel, historyModels, setEditorModel]
  );

  useEffect(() => {
    // open one from history
    setFromHistory(historyIndex);
  }, [historyIndex]);

  const openPrevModel = useCallback(() => {
    setHistoryIndex((prev) => {
      if (prev > 0) return prev - 1;
      return prev;
    });
  }, []);

  const openNextModel = useCallback(() => {
    setHistoryIndex((prev) => {
      if (prev < historyModels.length - 1) return prev + 1;
      return prev;
    });
  }, [historyModels]);

  const hostStyle =
    providerName && colorizeHosts
      ? {
          flexGrow: 1,
          borderBottomStyle: "solid",
          borderBottomColor: colorFromHostname(providerName),
          borderBottomWidth: "0.3em",
        }
      : { flexGrow: 1 };

  return (
    <Stack direction="row" spacing={0.5} alignItems="center" ref={refEl} sx={hostStyle}>
      <Tooltip title="Save File" disableInteractive>
        <span>
          <IconButton
            edge="end"
            disabled={!activeModelDirty}
            aria-label="Save File"
            onClick={() => {
              if (activeModel) {
                saveModel(activeModel);
              }
            }}
          >
            <SaveAltOutlinedIcon style={{ fontSize: "0.8em" }} />
          </IconButton>
        </span>
      </Tooltip>

      <OverflowMenu
        icon={<UpgradeIcon style={{ fontSize: "0.8em" }} />}
        tooltip="Open parent file"
        autoClick={true}
        disabled={selectParentFiles.length === 0}
        options={selectParentFiles.map((item) => {
          return {
            name: `${getFileName(item.path)} (${getFileName(item.inc_path)}:${item.line_number})`,
            tooltip: item.path,
            key: `${item.path}-${item.line_number}`,
            onClick: (): void => {
              setEditorModel(item.path, {
                startLineNumber: item.line_number,
                endLineNumber: item.line_number,
                startColumn: 0,
                endColumn: 0,
              });
            },
          };
        })}
        id="path-options"
      />

      <Tooltip title="go back" disableInteractive>
        <span>
          <IconButton
            edge="end"
            aria-label="preview"
            disabled={historyModels.length === 1 || historyIndex === 0}
            onClick={() => {
              openPrevModel();
            }}
          >
            <ArrowBackIosNewIcon style={{ fontSize: "0.8em" }} />
          </IconButton>
        </span>
      </Tooltip>
      <Tooltip title="go forward" disableInteractive>
        <span>
          <IconButton
            edge="end"
            aria-label="preview"
            disabled={!(historyIndex !== -1 && historyIndex + 1 < historyModels.length)}
            onClick={() => {
              openNextModel();
            }}
          >
            <ArrowForwardIosIcon style={{ fontSize: "0.8em" }} />
          </IconButton>
        </span>
      </Tooltip>

      <Tooltip title="Drop unsaved changes and reload current file from host" disableInteractive>
        <IconButton
          edge="end"
          aria-label="Reload file"
          onClick={async () => {
            reloadCurrentFile();
          }}
        >
          <CloudSyncOutlinedIcon style={{ fontSize: "0.8em" }} />
        </IconButton>
      </Tooltip>
      <Stack direction="row" width="100%" spacing={0.4} alignItems="center">
        {currentFileState.requesting && <CircularProgress size="0.8em" />}
        <Tooltip title={fileFromUriPath(activeModel?.uri.path || "")} disableInteractive>
          <div>
            <NativeSelect
              value={fileFromUriPath(currentFileState.path)}
              inputProps={{
                name: "current file",
                id: "uncontrolled-native",
              }}
              sx={{ fontWeight: "normal", fontSize: "0.8em", padding: "0.1em" }}
              onChange={(event: React.ChangeEvent<HTMLSelectElement>) => {
                setEditorModel(event.target.value);
              }}
            >
              {selectableFiles.map((fileName) => {
                return (
                  <option key={fileName} value={fileName}>
                    {modifiedFileSet.has(fileName) ? "*" : ""}
                    {getFileName(fileName)}
                  </option>
                );
              })}
            </NativeSelect>
          </div>
        </Tooltip>

        <Stack direction="row" spacing={0.2}>
          {modifiedFiles
            .filter((path) => path !== activeModel?.uri.path)
            .map((path) => {
              return (
                <Tooltip
                  key={path}
                  title={`changed ${getFileName(path)}`}
                  enterDelay={tooltipDelay}
                  enterNextDelay={tooltipDelay}
                  disableInteractive
                >
                  <Link
                    noWrap
                    aria-label={`modified ${path}`}
                    href="#"
                    // underline="none"
                    color="inherit"
                    onClick={() => {
                      setEditorModel(path);
                    }}
                  >
                    <Typography variant="body2" padding="0.4em" fontSize="0.6em">
                      {`[*${getFileAbb(path)}]`}
                    </Typography>
                  </Link>
                </Tooltip>
              );
            })}
        </Stack>
        <Typography flexGrow={1} />
        <Typography
          noWrap
          style={{
            padding: "0.4em",
            fontWeight: 100,
            fontSize: "0.6em",
          }}
        >
          {packageName} &#8226; {providerName}
        </Typography>
      </Stack>
    </Stack>
  );
}
