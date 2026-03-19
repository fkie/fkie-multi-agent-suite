import FolderCopyOutlinedIcon from "@mui/icons-material/FolderCopyOutlined";
import SearchOutlinedIcon from "@mui/icons-material/SearchOutlined";
import { Stack, ToggleButton, Tooltip, Typography } from "@mui/material";
import React, { RefObject, useCallback, useEffect, useState } from "react";
import SplitPane, { Pane, SashContent } from "split-pane-react";

import { useEditorSidebarLayout } from "@/renderer/hooks/editor/useEditorSidebarLayout";
import { LaunchIncludedFile } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { TLaunchArg } from "@/types";
import { SearchBar } from "../UI";
import ExplorerTree from "./ExplorerTree";
import SearchTree from "./SearchTree";

interface EditorSidebarProps {
  editorId: string;
  provider: Provider;
  rootFilePath: string;
  includedFiles: LaunchIncludedFile[];
  selectedUriPath: string;
  launchArgs: TLaunchArg[];
  modifiedUriPaths: string[];
  sideBarWidth: number;
  keyboardEvent?: React.KeyboardEvent;
  panelRef: RefObject<HTMLDivElement>;
  onStateChange: (collapsed: boolean) => void;
}

/**
 * Sidebar container
 */
export function EditorSidebar(props: EditorSidebarProps) {
  const {
    editorId,
    provider,
    rootFilePath,
    includedFiles,
    selectedUriPath = "",
    launchArgs = [],
    modifiedUriPaths = [],
    sideBarWidth,
    keyboardEvent,
    panelRef,
    onStateChange,
  } = props;
  const [enableGlobalSearch, setEnableGlobalSearch] = useState(false);
  const [enableExplorer, setEnableExplorer] = useState(false);
  const [globalSearchTerm, setGlobalSearchTerm] = useState("");

  const {
    fontSize,
    explorerBarHeight,
    explorerBarMinSize,
    setExplorerBarHeight,
    globalSearchHeight,
    showExplorerName,
  } = useEditorSidebarLayout({ panelRef, sideBarWidth, enableGlobalSearch, enableExplorer });

  useEffect(() => {
    onStateChange(!(enableExplorer || enableGlobalSearch));
  }, [enableExplorer, enableGlobalSearch]);

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
            {showExplorerName && <Typography fontSize="0.8em">Explorer</Typography>}
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
                provider={provider}
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
          <Stack direction="column" height={globalSearchHeight} overflow="auto">
            <SearchTree
              editorId={editorId}
              provider={provider}
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
