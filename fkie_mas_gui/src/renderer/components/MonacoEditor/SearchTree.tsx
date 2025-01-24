import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import StopIcon from "@mui/icons-material/Stop";
import { Box, CircularProgress, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import { editor } from "monaco-editor/esm/vs/editor/editor.api";
import { forwardRef, LegacyRef, useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";

import { MonacoContext } from "@/renderer/context/MonacoContext";
import { getFileName } from "@/renderer/models";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "@/renderer/pages/NodeManager/layout/events";
import { SearchFileTreeItem, SearchResultTreeItem } from "./SearchTreeItem";
import { TSearchResult } from "./types";

interface SearchTreeProps {
  tabId: string;
  providerId: string;
  ownUriPaths: string[];
  searchTerm: string;
}

const SearchTree = forwardRef<HTMLDivElement, SearchTreeProps>(function SearchTree(props, ref) {
  const { tabId, providerId, searchTerm = "", ownUriPaths = [] } = props;

  const monacoCtx = useContext(MonacoContext);
  const [expandedSearchResults, setExpandedSearchResults] = useState<string[]>([]);
  const [globalSearchTree, setGlobalSearchTree] = useState({});
  const [searchResults, setSearchResults] = useState<TSearchResult[]>([]);
  const [currentIndex, setCurrentIndex] = useState(ownUriPaths.length);
  const [currentSearchText, setCurrentSearchText] = useState("");
  const [currentIncludedText, setCurrentIncludedText] = useState(new Set(""));
  const [progress, setProgress] = useState(0);

  /**
   * Search through all available models a given text, and return all coincidences
   */
  async function findAllTextMatches(searchText: string, isRegex: boolean): Promise<void> {
    if (currentIndex < ownUriPaths.length) {
      // search only in own models
      const uriPath = ownUriPaths[currentIndex];
      const result = await monacoCtx.getModel(tabId, providerId, uriPath, false);
      if (result.model) {
        const matches: editor.FindMatch[] = result.model.findMatches(searchText, true, isRegex, false, null, false);
        matches?.forEach((match) => {
          const lineNumber = match.range.startLineNumber;
          const text = result.model?.getLineContent(match.range.startLineNumber);
          if (text && !currentIncludedText.has(text)) {
            setSearchResults(
              (prev) =>
                [
                  ...prev,
                  {
                    file: result.model ? result.model.uri.path : "",
                    text,
                    lineNumber,
                    range: match.range,
                  } as TSearchResult,
                ] as TSearchResult[]
            );
            currentIncludedText.add(text);
          }
        });
        setCurrentIndex((prev) => prev + 1);
      }
    }
  }

  const debouncedFindAllMatches = useDebounceCallback(async function (searchText: string): Promise<void> {
    setSearchResults([]);
    setCurrentSearchText(searchText);
    if (!searchText) {
      setCurrentIndex(ownUriPaths.length);
      return;
    }
    if (searchText.length < 3) return;
    setCurrentIncludedText(new Set(""));
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
    const newSearchTree = {};
    searchResults.forEach((item) => {
      const entry = newSearchTree[item.file];
      if (!entry) {
        newSearchTree[item.file] = [item];
      } else {
        newSearchTree[item.file].push(item);
      }
    });
    setGlobalSearchTree(newSearchTree);
    setExpandedSearchResults(Object.keys(newSearchTree));
  }, [searchResults]);

  useEffect(() => {
    debouncedFindAllMatches(searchTerm);
  }, [searchTerm]);

  function selectSearchResult(entry: TSearchResult): void {
    emitCustomEvent(EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange(tabId, entry.file, entry.range));
  }

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
        ref={ref as LegacyRef<HTMLUListElement>}
        expansionTrigger={"iconContainer"}
        expandedItems={expandedSearchResults}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        onExpandedItemsChange={(_event: React.SyntheticEvent, itemIds: string[]) => setExpandedSearchResults(itemIds)}
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
                    key={`${fileName}-${entry.lineNumber}`}
                    itemId={`${fileName}-${entry.lineNumber}`}
                    lineNumber={entry.lineNumber}
                    lineText={entry.text}
                    onClick={() => {
                      selectSearchResult(entry);
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
});

export default SearchTree;
