import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import { useCallback, useContext, useEffect, useMemo, useState } from "react";
import { emitCustomEvent } from "react-custom-events";
import { LoggingContext } from "../../context/LoggingContext";
import { RosContext } from "../../context/RosContext";
import { getFileName } from "../../models";
import { EVENT_EDITOR_SELECT_RANGE, eventEditorSelectRange } from "../../pages/NodeManager/layout/events";
import { FileTreeItem } from "./FileTreeItem";

function ExplorerTree({
  tabId,
  providerId,
  rootFilePath,
  includedFiles,
  selectedUriPath = "",
  launchArgs = {},
  modifiedUriPaths = [],
}) {
  const logCtx = useContext(LoggingContext);
  const rosCtx = useContext(RosContext);
  const [includeRoot, setIncludeRoot] = useState(null);
  const [expandedExplorerResults, setExpandedExplorerResults] = useState([]);

  const createUriPath = useCallback(
    (path) => {
      return `/${tabId}:${path}`;
    },
    [tabId]
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

  /** Create from SimpleTreeView from given root item */
  const includeFilesToTree = useCallback(
    (file, lineNumberCount) => {
      // eslint-disable-next-line react/jsx-no-useless-fragment
      if (!file) return <></>;
      const fileLaunchArgs = {};
      file.args?.forEach((item) => (fileLaunchArgs[item.name] = item.value));
      return (
        <FileTreeItem
          key={`${file.inc_path}-${file.line_number + lineNumberCount}`}
          itemId={`${file.inc_path}-${file.line_number + lineNumberCount}`}
          labelText={`${getFileName(file.inc_path)}`}
          labelLine={file.line_number}
          textColor={!file.exists ? "red" : ""}
          modified={modifiedUriPaths.includes(file.uriPath)}
          selected={
            selectedUriPath === file.uriPath &&
            (launchArgs.size === 0 ||
              file.args?.filter((item) => {
                return !(launchArgs[item.name] === item.value);
              }).length === 0)
          }
          onLabelClick={(event) => {
            emitCustomEvent(
              EVENT_EDITOR_SELECT_RANGE,
              eventEditorSelectRange(tabId, file.inc_path, null, fileLaunchArgs)
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
              eventEditorSelectRange(
                tabId,
                file.path,
                {
                  startLineNumber: file.line_number,
                  endLineNumber: file.line_number,
                  startColumn: 0,
                  endColumn: 0,
                },
                fileLaunchArgs
              )
            );
            event.stopPropagation();
          }}
        >
          {file.children.map((child) => {
            return includeFilesToTree(child, lineNumberCount + file.line_number);
          })}
        </FileTreeItem>
      );
    },
    [logCtx, modifiedUriPaths, selectedUriPath, tabId]
  );

  return (
    <SimpleTreeView
      aria-label="Explorer"
      expandedItems={expandedExplorerResults}
      slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
      onSelectedItemsChange={(event, itemId) => {
        const index = expandedExplorerResults.indexOf(itemId);
        const copyExpanded = [...expandedExplorerResults];
        if (index === -1) {
          copyExpanded.push(itemId);
        } else {
          copyExpanded.splice(index, 1);
        }
        setExpandedExplorerResults(copyExpanded);
      }}
      sx={{ flexGrow: 1, overflow: "auto" }}
    >
      {useMemo(() => {
        return includeFilesToTree(includeRoot, 0);
      }, [includeFilesToTree, includeRoot])}
    </SimpleTreeView>
  );
}

ExplorerTree.propTypes = {
  tabId: PropTypes.string.isRequired,
  providerId: PropTypes.string.isRequired,
  rootFilePath: PropTypes.string.isRequired,
  includedFiles: PropTypes.array.isRequired,
  selectedUriPath: PropTypes.string,
  launchArgs: PropTypes.object,
  modifiedUriPaths: PropTypes.array,
};

export default ExplorerTree;
