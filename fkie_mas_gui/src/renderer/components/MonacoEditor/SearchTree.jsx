import PropTypes from 'prop-types';
import { useContext, useEffect, useState } from 'react';

import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import { TreeView } from '@mui/x-tree-view';
import { useDebounceCallback } from '@react-hook/debounce';
import { emitCustomEvent } from 'react-custom-events';
import { MonacoContext } from '../../context/MonacoContext';
import { getFileName } from '../../models';
import {
  EVENT_EDITOR_SELECT_RANGE,
  eventEditorSelectRange,
} from '../../utils/events';
import { SearchFileTreeItem, SearchResultTreeItem } from './SearchTreeItem';

function SearchTree({ tabId, ownUriPaths, searchTerm }) {
  const monacoCtx = useContext(MonacoContext);
  const [expandedSearchResults, setExpandedSearchResults] = useState([]);
  const [globalSearchTree, setGlobalSearchTree] = useState({});

  const debouncedFindAllMatches = useDebounceCallback((searchText) => {
    const searchResult = monacoCtx.findAllTextMatches(
      searchText,
      false,
      ownUriPaths,
    );
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
                  labelText={entry.lineNumber}
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

SearchTree.defaultProps = {
  searchTerm: '',
  ownUriPaths: [],
};

SearchTree.propTypes = {
  tabId: PropTypes.string.isRequired,
  ownUriPaths: PropTypes.arrayOf(PropTypes.string),
  searchTerm: PropTypes.string,
};

export default SearchTree;
