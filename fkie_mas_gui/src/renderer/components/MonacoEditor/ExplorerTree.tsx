import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import { forwardRef, LegacyRef, useCallback, useContext, useEffect, useMemo, useState } from "react";

import { RosContext } from "@/renderer/context/RosContext";
import { LaunchArgument, LaunchIncludedFile } from "@/renderer/models";
import { Provider } from "@/renderer/providers";
import { TLaunchArg } from "@/types";
import FileTreeItem from "./FileTreeItem";
import { TLaunchIncludeItem } from "./types";

export function equalLaunchArgs(launchArgs: TLaunchArg[], argList: LaunchArgument[]): boolean {
  if (launchArgs && launchArgs.length > 0) {
    const notEqual = argList?.filter((item) => {
      // ignore args with not resolved statements
      const found = launchArgs.filter(
        (li) => li.name === item.name && (li.value === item.value || (!li.value && !item.value))
      );
      return !(found.length > 0 || item.value?.search("/\\$\\(") !== -1);
    });
    return notEqual.length === 0;
  }
  return true;
}

interface ExplorerTreeProps {
  tabId: string;
  providerId: string;
  rootFilePath: string;
  includedFiles: LaunchIncludedFile[];
  selectedUriPath: string;
  launchArgs: TLaunchArg[];
  modifiedUriPaths: string[];
}

const ExplorerTree = forwardRef<HTMLDivElement, ExplorerTreeProps>(function ExplorerTree(props, ref) {
  const {
    tabId,
    providerId,
    rootFilePath,
    includedFiles,
    selectedUriPath = "",
    launchArgs = [],
    modifiedUriPaths = [],
  } = props;
  const rosCtx = useContext(RosContext);
  const [includeRoot, setIncludeRoot] = useState<TLaunchIncludeItem>();
  const [expandedExplorerResults, setExpandedExplorerResults] = useState<string[]>([]);

  const createUriPath = useCallback(
    function (path: string): string {
      return `/${tabId}:${path}`;
    },
    [tabId]
  );

  useEffect(() => {
    const provider: Provider | undefined = rosCtx.getProviderById(providerId, true);
    if (!provider) return;
    const providerHost = provider.host();
    if (!providerHost) return;
    const rootItem: TLaunchIncludeItem = {
      uriPath: createUriPath(rootFilePath),
      children: [],
      file: {
        inc_path: rootFilePath,
        exists: true,
        rec_depth: -1,
        line_number: -1,
        conditional_excluded: false,
      } as LaunchIncludedFile,
    } as TLaunchIncludeItem;
    let currentFile: TLaunchIncludeItem = rootItem;
    includedFiles.forEach((file) => {
      const incItem: TLaunchIncludeItem = {
        children: [],
        uriPath: createUriPath(file.inc_path),
        file: file,
      };
      let curDepth = currentFile.file.rec_depth || 0;
      const fileDepth = file.rec_depth || 0;
      if (fileDepth - 1 === curDepth) {
        currentFile.children.push(incItem);
      } else if (fileDepth - 1 > curDepth) {
        currentFile = currentFile.children.slice(-1)[0];
        currentFile.children.push(incItem);
      } else {
        currentFile = rootItem;
        curDepth = currentFile.file.rec_depth || 0;
        while (fileDepth - 1 > curDepth) {
          currentFile = currentFile.children.slice(-1)[0];
          curDepth = currentFile.file.rec_depth || 0;
        }
        currentFile.children.push(incItem);
      }
    });
    setIncludeRoot(rootItem);
  }, [includedFiles, providerId, rootFilePath, rosCtx]);

  /** Create from SimpleTreeView from given root item */
  const includeFilesToTree = useCallback(
    (item: TLaunchIncludeItem, lineNumber: number, parentItems: string[] = []): JSX.Element => {
      // eslint-disable-next-line react/jsx-no-useless-fragment
      if (!item) return <></>;
      const id = `${lineNumber}-${item.file.inc_path}-${item.file.line_number}`;
      const pathList: string[] = [...parentItems, id];
      const selected =
        selectedUriPath === item.uriPath &&
        // compare launchArgs if there are several files with the same name
        (item.uriPath.endsWith(`:${rootFilePath}`) || equalLaunchArgs(launchArgs, item.file.args || []));
      if (selected) {
        // expand all items from root to the selected item
        setExpandedExplorerResults(pathList);
      }
      return (
        <FileTreeItem
          key={id}
          tabId={tabId}
          itemId={id}
          item={item}
          modified={modifiedUriPaths.includes(item.uriPath)}
          selected={selected}
        >
          {item.children.map((child) => {
            return includeFilesToTree(child, item.file.line_number, pathList);
          })}
        </FileTreeItem>
      );
    },
    [modifiedUriPaths, tabId, rootFilePath, launchArgs, selectedUriPath]
  );

  return (
    <SimpleTreeView
      aria-label="Explorer"
      ref={ref as LegacyRef<HTMLUListElement>}
      expansionTrigger={"iconContainer"}
      expandedItems={expandedExplorerResults}
      slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
      onExpandedItemsChange={(_event: React.SyntheticEvent, itemIds: string[]) => setExpandedExplorerResults(itemIds)}
      onSelectedItemsChange={(_event, itemId) => {
        if (itemId) {
          const index = expandedExplorerResults.indexOf(itemId);
          const copyExpanded = [...expandedExplorerResults];
          if (index === -1) {
            copyExpanded.push(itemId);
          } else {
            copyExpanded.splice(index, 1);
          }
          setExpandedExplorerResults(copyExpanded);
        } else {
          setExpandedExplorerResults([]);
        }
      }}
      sx={{ flexGrow: 1, overflow: "auto" }}
    >
      {useMemo(() => {
        if (includeRoot) {
          return includeFilesToTree(includeRoot, 0);
        }
        return <></>;
      }, [includeFilesToTree, includeRoot, selectedUriPath])}
    </SimpleTreeView>
  );
});

export default ExplorerTree;
