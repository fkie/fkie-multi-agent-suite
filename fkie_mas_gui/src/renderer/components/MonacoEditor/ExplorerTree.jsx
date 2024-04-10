import PropTypes from 'prop-types';
import { useCallback, useContext, useEffect, useMemo, useState } from 'react';

import ArrowDropDownIcon from '@mui/icons-material/ArrowDropDown';
import ArrowRightIcon from '@mui/icons-material/ArrowRight';
import { TreeView } from '@mui/x-tree-view';
import { emitCustomEvent } from 'react-custom-events';
import { LoggingContext } from '../../context/LoggingContext';
import { RosContext } from '../../context/RosContext';
import { getFileName } from '../../models';
import {
  EVENT_EDITOR_SELECT_RANGE,
  eventEditorSelectRange,
} from '../../utils/events';
import { FileTreeItem } from './FileTreeItem';

function ExplorerTree({
  tabId,
  providerId,
  rootFilePath,
  includedFiles,
  selectedUriPath,
  modifiedUriPaths,
}) {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const [includeRoot, setIncludeRoot] = useState(null);
  const [expandedExplorerResults, setExpandedExplorerResults] = useState([]);

  const createUriPath = useCallback(
    (path) => {
      return `/${tabId}:${path}`;
    },
    [tabId],
  );

  useEffect(() => {
    const provider = rosCtx.getProviderById(providerId);
    const providerHost = provider.host();
    if (!providerHost) return;
    const rootItem = {
      uriPath: createUriPath(rootFilePath),
      inc_path: rootFilePath,
      exists: true,
      rec_depth: -1,
      line_number: -1,
      children: [],
    };
    let currentFile = rootItem;
    includedFiles.forEach((file) => {
      file.children = [];
      file.uriPath = createUriPath(file.inc_path);
      if (file.rec_depth - 1 === currentFile.rec_depth) {
        currentFile.children.push(file);
      } else if (file.rec_depth - 1 > currentFile.rec_depth) {
        currentFile = currentFile.children.slice(-1)[0];
        currentFile.children.push(file);
      } else {
        currentFile = rootItem;
        while (file.rec_depth - 1 > currentFile.rec_depth) {
          currentFile = currentFile.children.slice(-1)[0];
        }
        currentFile.children.push(file);
      }
    });
    setExpandedExplorerResults([
      `${rootItem.inc_path}-${rootItem.line_number}`,
      ...includedFiles.map((item) => {
        return `${item.inc_path}-${item.line_number}`;
      }),
    ]);
    setIncludeRoot(rootItem);
  }, [createUriPath, includedFiles, providerId, rootFilePath, rosCtx]);

  /** Create from TreeView from given root item */
  const includeFilesToTree = useCallback(
    (file) => {
      // eslint-disable-next-line react/jsx-no-useless-fragment
      if (!file) return <></>;
      return (
        <FileTreeItem
          key={`${file.inc_path}-${file.line_number}`}
          nodeId={`${file.inc_path}-${file.line_number}`}
          labelText={`${getFileName(file.inc_path)}`}
          labelLine={file.line_number}
          textColor={!file.exists ? 'red' : ''}
          modified={modifiedUriPaths.includes(file.uriPath)}
          selected={selectedUriPath === file.uriPath}
          onLabelClick={(event) => {
            emitCustomEvent(
              EVENT_EDITOR_SELECT_RANGE,
              eventEditorSelectRange(tabId, file.inc_path, null),
            );
            event.stopPropagation();
          }}
          onLabelDoubleClick={(event) => {
            navigator.clipboard.writeText(file.inc_path);
            logCtx.success(`${file.inc_path} copied!`);
            event.stopPropagation();
          }}
          onLinenumberClick={(event) => {
            emitCustomEvent(
              EVENT_EDITOR_SELECT_RANGE,
              eventEditorSelectRange(tabId, file.path, {
                startLineNumber: file.line_number,
                endLineNumber: file.line_number,
                startColumn: 0,
                endColumn: 0,
              }),
            );
            event.stopPropagation();
          }}
        >
          {file.children.map((child) => {
            return includeFilesToTree(child);
          })}
        </FileTreeItem>
      );
    },
    [logCtx, modifiedUriPaths, selectedUriPath, tabId],
  );

  return (
    <TreeView
      aria-label="Explorer"
      expanded={expandedExplorerResults}
      defaultCollapseIcon={<ArrowDropDownIcon />}
      defaultExpandIcon={<ArrowRightIcon />}
      onNodeSelect={(event, nodeId) => {
        const index = expandedExplorerResults.indexOf(nodeId);
        const copyExpanded = [...expandedExplorerResults];
        if (index === -1) {
          copyExpanded.push(nodeId);
        } else {
          copyExpanded.splice(index, 1);
        }
        setExpandedExplorerResults(copyExpanded);
      }}
      sx={{ flexGrow: 1, overflow: 'auto' }}
    >
      {useMemo(() => {
        return includeFilesToTree(includeRoot);
      }, [includeFilesToTree, includeRoot])}
    </TreeView>
  );
}

ExplorerTree.defaultProps = {
  selectedUriPath: '',
  modifiedUriPaths: [],
};

ExplorerTree.propTypes = {
  tabId: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
  rootFilePath: PropTypes.string.isRequired,
  includedFiles: PropTypes.array.isRequired,
  selectedUriPath: PropTypes.string,
  modifiedUriPaths: PropTypes.array,
};

export default ExplorerTree;
