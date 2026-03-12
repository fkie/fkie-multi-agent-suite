import FolderCopyOutlinedIcon from "@mui/icons-material/FolderCopyOutlined";
import SearchOutlinedIcon from "@mui/icons-material/SearchOutlined";
import { Stack, ToggleButton, Tooltip, Typography } from "@mui/material";
import React, { useCallback, useEffect, useState } from "react";
import SplitPane, { Pane, SashContent } from "split-pane-react";

import useLocalStorage from "@/renderer/hooks/useLocalStorage";
import { useSettingsContext } from "@/renderer/hooks/useSettingsContext";
import { LaunchIncludedFile } from "@/renderer/models";
import { TLaunchArg } from "@/types";
import { SearchBar } from "../UI";
import ExplorerTree from "./ExplorerTree";
import SearchTree from "./SearchTree";

interface EditorSidebarProps {
  editorId: string;
  providerId: string;
  rootFilePath: string;
  includedFiles: LaunchIncludedFile[];
  selectedUriPath: string;
  launchArgs: TLaunchArg[];
  modifiedUriPaths: string[];
  sideBarWidth: number;
  keyboardEvent?: React.KeyboardEvent;
  panelSize?: DOMRect;
  onStateChange: (collapsed: boolean) => void;
}

/**
 * Sidebar container
 */
export function EditorSidebar(props: EditorSidebarProps) {
  const {
    editorId,
    providerId,
    rootFilePath,
    includedFiles,
    selectedUriPath = "",
    launchArgs = [],
    modifiedUriPaths = [],
    sideBarWidth,
    keyboardEvent,
    panelSize,
    onStateChange,
  } = props;
  const settingsCtx = useSettingsContext();
  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [enableExplorer, setEnableExplorer] = useState(false);
  const [globalSearchTerm, setGlobalSearchTerm] = useState("");
  const [fontSize, setFontSize] = useState<number>(settingsCtx.get("fontSize") as number);
  const [savedExplorerBarHight, setSavedExplorerBarHight] = useLocalStorage<number>(
    "Editor:explorerBarHight",
    fontSize * 20
  );
  const [panelHeight, setPanelHeight] = useState<number>(panelSize?.height || fontSize * 2);
  const [explorerBarMinSize, setExplorerBarMinSize] = useState<number>(fontSize * 2);
  const [explorerBarHeight, setExplorerBarHeight] = useState<number>(fontSize * 2);

  useEffect(() => {
    // update height and width of the split panel on the left side
    if (enableExplorer) {
      if (enableGlobalSearch) {
        setExplorerBarHeight(savedExplorerBarHight);
      } else {
        setExplorerBarHeight(panelHeight - explorerBarMinSize);
      }
    } else {
      setExplorerBarHeight(explorerBarMinSize);
    }
  }, [enableExplorer, enableGlobalSearch, panelHeight]);

  useEffect(() => {
    if (!panelSize) return;
    setPanelHeight(panelSize.height);
  }, [panelSize]);

  useEffect(() => {
    onStateChange(!(enableExplorer || enableGlobalSearch));
  }, [enableExplorer, enableGlobalSearch]);

  useEffect(() => {
    const newFontSize = settingsCtx.get("fontSize") as number;
    setFontSize(newFontSize);
    setExplorerBarMinSize(newFontSize * 2 + 2);
  }, [settingsCtx.changed]);

  useEffect(() => {
    if (!keyboardEvent) return;
    if (keyboardEvent.ctrlKey && keyboardEvent.shiftKey && keyboardEvent.key === "E") {
      setEnableExplorer((prev) => !prev);
    }
    if (keyboardEvent.ctrlKey && keyboardEvent.shiftKey && keyboardEvent.key === "F") {
      setEnableGlobalSearch((prev) => !prev);
    }
  }, [keyboardEvent]);

  const handleChangeExplorer = useCallback((isExpanded: boolean): void => {
    setEnableExplorer(isExpanded);
  }, []);

  const handleChangeSearch = useCallback((isExpanded: boolean): void => {
    setEnableGlobalSearch(isExpanded);
  }, []);

  return (
    <SplitPane
      // defaultSize={sideBarWidth}
      sizes={[explorerBarHeight]}
      onChange={([size]) => {
        if (size !== explorerBarHeight && size >= explorerBarMinSize) {
          setSavedExplorerBarHight(size);
        }
        setExplorerBarHeight(size);
      }}
      split="horizontal"
      resizerSize={6}
      sashRender={(_index, active) => (
        <SashContent className={`sash-wrap-line ${active ? "active" : "inactive"}`}>
          {/* <span className="line"/> */}
        </SashContent>
      )}
    >
      <Pane minSize={explorerBarMinSize}>
        <Stack>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Tooltip title="Explorer (Ctrl+Shift+E)" placement="right" disableInteractive>
              <ToggleButton
                size="small"
                value="showExplorer"
                selected={enableExplorer}
                onChange={() => {
                  handleChangeExplorer(!enableExplorer);
                }}
              >
                <FolderCopyOutlinedIcon sx={{ fontSize: "inherit" }} />
              </ToggleButton>
            </Tooltip>
            {enableExplorer && sideBarWidth > fontSize * 7 && <Typography fontSize="0.8em">Explorer</Typography>}
          </Stack>
          {enableExplorer && (
            <Stack
              overflow="auto"
              direction="column"
              height={explorerBarHeight - fontSize * 2 - 2}
              width={sideBarWidth}
            >
              <ExplorerTree
                editorId={editorId}
                providerId={providerId}
                rootFilePath={rootFilePath}
                includedFiles={includedFiles}
                selectedUriPath={selectedUriPath}
                launchArgs={launchArgs}
                modifiedUriPaths={modifiedUriPaths}
              />
            </Stack>
          )}
        </Stack>
      </Pane>
      <Stack paddingTop="2px">
        <Stack direction="row" alignItems="center" spacing={1}>
          <Tooltip title="Search (Ctrl+Shift+F)" placement="right" disableInteractive>
            <ToggleButton
              size="small"
              value="showSearch"
              selected={enableGlobalSearch}
              onChange={() => handleChangeSearch(!enableGlobalSearch)}
            >
              <SearchOutlinedIcon sx={{ fontSize: "inherit" }} />
            </ToggleButton>
          </Tooltip>
          {enableGlobalSearch && (
            <SearchBar
              onSearch={(value) => {
                setGlobalSearchTerm(value);
              }}
              placeholder="Search in all included files..."
              defaultValue={globalSearchTerm}
              searchIcon={undefined}
            />
          )}
        </Stack>
        {enableGlobalSearch && (
          <Stack
            direction="column"
            height={(panelSize?.height ? panelSize?.height : 100) - explorerBarHeight - fontSize * 2 - 8}
            overflow="auto"
          >
            <SearchTree
              editorId={editorId}
              providerId={providerId}
              rootFilePath={rootFilePath}
              includedFiles={includedFiles}
              searchTerm={globalSearchTerm}
            />
          </Stack>
        )}
      </Stack>
    </SplitPane>
  );
}
