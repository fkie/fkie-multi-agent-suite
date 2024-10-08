import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import StopIcon from "@mui/icons-material/Stop";
import { Box, CircularProgress, IconButton, Stack, Tooltip, Typography } from "@mui/material";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useDebounceCallback } from "@react-hook/debounce";
import PropTypes from "prop-types";
import { useContext, useEffect, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { MonacoContext } from "../../context/MonacoContext";
import { getFileName } from "../../models";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "../../pages/NodeManager/layout/events";
import { SearchFileTreeItem, SearchResultTreeItem } from "./SearchTreeItem";

function SearchTree({ tabId, providerId, searchTerm = "", ownUriPaths = [] }) {
  const monacoCtx = useContext(MonacoContext);
  const [expandedSearchResults, setExpandedSearchResults] = useState([]);
  const [globalSearchTree, setGlobalSearchTree] = useState({});
  const [searchResults, setSearchResults] = useState([]);
  const [currentIndex, setCurrentIndex] = useState(ownUriPaths.length);
  const [currentSearchText, setCurrentSearchText] = useState("");
  const [currentIncludedText, setCurrentIncludedText] = useState(new Set(""));
  const [progress, setProgress] = useState(0);

  /**
   * Search through all available models a given text, and return all coincidences
   *
   * @param {string} searchText - Text to search
   */
  const findAllTextMatches = async (searchText, isRegex) => {
    if (currentIndex < ownUriPaths.length) {
      // search only in own models
      const uriPath = ownUriPaths[currentIndex];
      const result = await monacoCtx.getModel(tabId, providerId, uriPath, false);
      if (result.model) {
        const matches = result.model.findMatches(searchText, true, isRegex, false, null, false);
        matches?.forEach((match) => {
          const lineNumber = match.range.startLineNumber;
          const text = result.model.getLineContent(match.range.startLineNumber);
          if (!currentIncludedText.has(text)) {
            setSearchResults((prev) => [
              ...prev,
              {
                file: result.model.uri.path,
                text,
                lineNumber,
                range: match.range,
              },
            ]);
            currentIncludedText.add(text);
          }
        });
        setCurrentIndex((prev) => prev + 1);
      }
    }
  };

  const debouncedFindAllMatches = useDebounceCallback(async (searchText) => {
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

  function selectSearchResult(entry) {
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
        expandedItems={expandedSearchResults}
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // defaultEndIcon={<div style={{ width: 24 }} />}
        onSelectedItemsChange={(event, itemId) => {
          const index = expandedSearchResults.indexOf(itemId);
          const copyExpanded = [...expandedSearchResults];
          if (index === -1) {
            copyExpanded.push(itemId);
          } else {
            copyExpanded.splice(index, 1);
          }
          setExpandedSearchResults(copyExpanded);
        }}
        sx={{ flexGrow: 1, overflow: "auto" }}
      >
        {Object.keys(globalSearchTree).map((fileName) => {
          const entries = globalSearchTree[fileName];
          return (
            <SearchFileTreeItem
              fontSize="0.8em"
              key={fileName}
              itemId={fileName}
              labelText={`${getFileName(fileName)}`}
              labelCount={entries.length}
            >
              {entries.map((entry) => {
                return (
                  <SearchResultTreeItem
                    fontSize="0.8em"
                    key={`${fileName}-${entry.lineNumber}`}
                    itemId={`${fileName}-${entry.lineNumber}`}
                    labelText={`${entry.lineNumber}`}
                    labelInfo={entry.text}
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
}

SearchTree.propTypes = {
  tabId: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
  ownUriPaths: PropTypes.arrayOf(PropTypes.string),
  searchTerm: PropTypes.string,
};

export default SearchTree;
