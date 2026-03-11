import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import StopIcon from "@mui/icons-material/Stop";
import { Box, CircularProgress, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { editor } from "monaco-editor/esm/vs/editor/editor.api";
import { useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { useMonacoContext } from "@/renderer/hooks/useMonacoContext";
import { getFileName, LaunchIncludedFile } from "@/renderer/models";
import { createUriPath } from "@/renderer/monaco/utils";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "@/renderer/pages/NodeManager/layout/events";
import { SearchFileTreeItem, SearchResultTreeItem } from "./SearchTreeItem";
import { TSearchResult } from "./types";

interface SearchTreeProps {
  tabId: string;
  providerId: string;
  rootFilePath: string;
  includedFiles?: LaunchIncludedFile[];
  searchTerm: string;
}

export default function SearchTree(props: SearchTreeProps): JSX.Element {
  const { tabId, providerId, rootFilePath, includedFiles, searchTerm = "" } = props;

  const monacoCtx = useMonacoContext();
  const [expandedSearchResults, setExpandedSearchResults] = useState<string[]>([]);
  const [searchResults, setSearchResults] = useState<TSearchResult[]>([]);
  const [currentIndex, setCurrentIndex] = useState((includedFiles?.length || 0) + 1);
  const [currentSearchText, setCurrentSearchText] = useState("");
  const [progress, setProgress] = useState(0);
  // files to load and search
  const ownUriPaths = useMemo(() => {
    const resultSet = new Set<string>();
    resultSet.add(rootFilePath);
    for (const f of includedFiles || []) {
      resultSet.add(createUriPath(providerId, f.inc_path));
    }
    return Array.from(resultSet);
  }, [providerId, rootFilePath, includedFiles]);

  // create global search tree if search results are changed
  const globalSearchTree = useMemo(() => {
    const tree: Record<string, TSearchResult[]> = {};

    for (const item of searchResults) {
      if (!tree[item.file]) {
        tree[item.file] = [];
      }
      tree[item.file].push(item);
    }
    return tree;
  }, [searchResults]);

  useEffect(() => {
    setExpandedSearchResults(Object.keys(globalSearchTree));
  }, [globalSearchTree]);

  /**
   * Search through all available models a given text, and return all coincidences
   */
  async function findAllTextMatches(searchText: string, isRegex: boolean): Promise<void> {
    if (currentIndex < ownUriPaths.length) {
      // search only in own models
      const uriPath = ownUriPaths[currentIndex];
      const result = await monacoCtx.getModel(tabId, uriPath, false);
      if (!result.model) return;
      const matches: editor.FindMatch[] = result.model.findMatches(searchText, true, isRegex, false, null, false);
      const newResults: TSearchResult[] = [];
      for (const match of matches || []) {
        const lineNumber = match.range.startLineNumber;
        const text = result.model?.getLineContent(match.range.startLineNumber);
        if (text) {
          newResults.push({
            file: result.model.uri.path,
            text,
            lineNumber,
            range: match.range,
          });
        }
      }
      setSearchResults((prev) => [...prev, ...newResults]);
      setCurrentIndex((prev) => prev + 1);
    }
  }

  const debouncedFindAllMatches = useDebounceCallback(async (searchText: string): Promise<void> => {
    setSearchResults([]);
    setCurrentSearchText(searchText);
    if (!searchText) {
      setCurrentIndex(ownUriPaths.length);
      return;
    }
    if (searchText.length < 3) return;
    setProgress(0);
    setCurrentIndex(0);
  }, 500);

  useEffect(() => {
    if (currentIndex < ownUriPaths.length) {
      findAllTextMatches(currentSearchText, false);
    }
    setProgress((currentIndex / ownUriPaths.length) * 100);
  }, [currentIndex]);

  useEffect(() => {
    debouncedFindAllMatches(searchTerm);
  }, [searchTerm]);

  return (
    <Stack spacing="0.5em">
      {currentIndex < ownUriPaths.length && (
        <Stack padding="0.5em" spacing="0.5em" direction="row" justifyItems="center" alignItems="center">
          <Box sx={{ position: "relative", display: "inline-flex" }}>
            <CircularProgress size="1.2em" variant="determinate" value={progress} />
            <Box
              sx={{
                top: 0,
                left: 0,
                bottom: 0,
                right: 0,
                position: "absolute",
                display: "flex",
                alignItems: "center",
                justifyContent: "center",
              }}
            >
              <Tooltip title="stop searching" placement="bottom">
                <IconButton
                  color="primary"
                  onClick={() => {
                    setCurrentIndex(ownUriPaths.length);
                  }}
                  size="small"
                >
                  <StopIcon fontSize="inherit" />
                </IconButton>
              </Tooltip>
            </Box>
          </Box>
          <Typography variant="body2">{getFileName(ownUriPaths[currentIndex])}</Typography>
        </Stack>
      )}
      <SimpleTreeView
        aria-label="Search results"
        expansionTrigger={"iconContainer"}
        expandedItems={expandedSearchResults}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        onExpandedItemsChange={(_event: React.SyntheticEvent | null, itemIds: string[]) =>
          setExpandedSearchResults(itemIds)
        }
        onSelectedItemsChange={(_event, itemId) => {
          if (itemId) {
            const index = expandedSearchResults.indexOf(itemId);
            const copyExpanded = [...expandedSearchResults];
            if (index === -1) {
              copyExpanded.push(itemId);
            } else {
              copyExpanded.splice(index, 1);
            }
            setExpandedSearchResults(copyExpanded);
          } else {
            setExpandedSearchResults([]);
          }
        }}
        sx={{ flexGrow: 1, overflow: "auto" }}
      >
        {Object.keys(globalSearchTree).map((fileName) => {
          const entries = globalSearchTree[fileName];
          return (
            <SearchFileTreeItem
              key={fileName}
              itemId={fileName}
              fileName={`${getFileName(fileName)}`}
              countChildren={entries.length}
            >
              {entries.map((entry) => {
                return (
                  <SearchResultTreeItem
                    key={`${fileName}-${entry.lineNumber}-${entry.range}`}
                    itemId={`${fileName}-${entry.lineNumber}-${entry.range}`}
                    lineNumber={entry.lineNumber}
                    lineText={entry.text}
                    onClick={() => {
                      emitCustomEvent(
                        EVENT_EDITOR_SELECT_RANGE,
                        eventEditorSelectRange(tabId, entry.file, entry.range)
                      );
                    }}
                  />
                );
              })}
            </SearchFileTreeItem>
          );
        })}
      </SimpleTreeView>
    </Stack>
  );
}
