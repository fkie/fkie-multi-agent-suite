import { useMonaco } from '@monaco-editor/react';
import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import { TreeView } from '@mui/x-tree-view';
import { useDebounceCallback } from '@react-hook/debounce';
import PropTypes from 'prop-types';
import { useEffect, useState } from 'react';
import { emitCustomEvent } from 'react-custom-events';
import { getFileName } from '../../models';
import {
  EVENT_EDITOR_SELECT_RANGE,
  eventEditorSelectRange,
} from '../../utils/events';
import { SearchFileTreeItem, SearchResultTreeItem } from './SearchTreeItem';

function SearchTree({ tabId, searchTerm = '', ownUriPaths = [] }) {
  const monaco = useMonaco();
  const [expandedSearchResults, setExpandedSearchResults] = useState([]);
  const [globalSearchTree, setGlobalSearchTree] = useState({});

  /**
   * Search through all available models a given text, and return all coincidences
   *
   * @param {string} searchText - Text to search
   */
  const findAllTextMatches = (searchText, isRegex) => {
    if (!searchText) return [];
    if (searchText.length < 3) return [];

    const searchResult = [];
    const includedText = new Set('');

    if (searchText) {
      // search only in own models
      ownUriPaths.forEach((uriPath) => {
        const model = monaco.editor.getModel(monaco.Uri.file(uriPath));
        const matches = model.findMatches(
          searchText,
          true,
          isRegex,
          false,
          null,
          false,
        );
        matches.forEach((match) => {
          const lineNumber = match.range.startLineNumber;
          const text = model.getLineContent(match.range.startLineNumber);

          if (!includedText.has(text)) {
            searchResult.push({
              file: model.uri.path,
              text,
              lineNumber,
              range: match.range,
            });

            includedText.add(text);
          }
        });
      });
    }
    return searchResult;
  };

  const debouncedFindAllMatches = useDebounceCallback((searchText) => {
    const searchResult = findAllTextMatches(searchText, false);

    const newSearchTree = {};
    searchResult.forEach((item) => {
      const entry = newSearchTree[item.file];
      if (!entry) {
        newSearchTree[item.file] = [item];
      } else {
        newSearchTree[item.file].push(item);
      }
    });
    setGlobalSearchTree(newSearchTree);
    setExpandedSearchResults(Object.keys(newSearchTree));
  }, 50);

  useEffect(() => {
    debouncedFindAllMatches(searchTerm);
  }, [debouncedFindAllMatches, searchTerm]);

  function selectSearchResult(entry) {
    emitCustomEvent(
      EVENT_EDITOR_SELECT_RANGE,
      eventEditorSelectRange(tabId, entry.file, entry.range),
    );
  }

  return (
    <TreeView
      aria-label="Search results"
      expanded={expandedSearchResults}
      defaultCollapseIcon={<ArrowDropDownIcon />}
      defaultExpandIcon={<ArrowRightIcon />}
      // defaultEndIcon={<div style={{ width: 24 }} />}
      onNodeSelect={(event, nodeId) => {
        const index = expandedSearchResults.indexOf(nodeId);
        const copyExpanded = [...expandedSearchResults];
        if (index === -1) {
          copyExpanded.push(nodeId);
        } else {
          copyExpanded.splice(index, 1);
        }
        setExpandedSearchResults(copyExpanded);
      }}
      sx={{ flexGrow: 1, overflow: 'auto' }}
    >
      {Object.keys(globalSearchTree).map((fileName) => {
        const entries = globalSearchTree[fileName];
        return (
          <SearchFileTreeItem
            fontSize="0.8em"
            key={fileName}
            nodeId={fileName}
            labelText={`${getFileName(fileName)}`}
            labelCount={entries.length}
          >
            {entries.map((entry) => {
              return (
                <SearchResultTreeItem
                  fontSize="0.8em"
                  key={`${fileName}-${entry.lineNumber}`}
                  nodeId={`${fileName}-${entry.lineNumber}`}
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
    </TreeView>
  );
}

SearchTree.propTypes = {
  tabId: PropTypes.string.isRequired,
  ownUriPaths: PropTypes.arrayOf(PropTypes.string),
  searchTerm: PropTypes.string,
};

export default SearchTree;
