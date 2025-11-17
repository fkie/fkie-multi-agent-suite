import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import { useCallback, useEffect, useMemo, useState } from "react";

import { RosPackage } from "@/renderer/models";
import { generateUniqueId } from "@/renderer/utils";
import FileTreeItem from "./FileTreeItem";
import FolderTreeItem from "./FolderTreeItem";
import HistoryGroupItem from "./HistoryGroupItem";
import PackageTreeItem from "./PackageTreeItem";
import { TPackageItemsTree, TPackageTreeItem } from "./types";

interface TreeDirectoryProps {
  packageItemsTree: TPackageItemsTree;
  selectedPackage: RosPackage | undefined;
  providerName: string | undefined;
  onNodeSelect: (itemId: string) => void;
  onFileDoubleClick: (label: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

export default function TreeDirectory(props: TreeDirectoryProps): JSX.Element {
  const {
    packageItemsTree,
    selectedPackage = undefined,
    providerName = undefined,
    onNodeSelect = (): void => {},
    onFileDoubleClick = (): void => {},
  } = props;

  const [expanded, setExpanded] = useState<string[]>([]);
  const [selectedItems, setSelectedItems] = useState<string | null>(null);

  /**
   * Remove expanded items if tree items changed
   */
  useEffect(() => {
    if (packageItemsTree) {
      setExpanded(Object.keys(packageItemsTree));
    } else {
      setExpanded([]);
    }
  }, [packageItemsTree]);

  /**
   * Callback when items on the tree are expanded/retracted
   */
  const handleToggle = useCallback((_event, nodeIds: string[]) => {
    setExpanded(nodeIds);
  }, []);

  /**
   * Callback when items on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (_event: React.SyntheticEvent | null, itemId: string | null) => {
      setSelectedItems(itemId);
      onNodeSelect(itemId ? itemId : "");
    },
    [onNodeSelect]
  );

  /**
   * Callback when folders on the tree are double-clicked by the user
   */
  function handleFolderDoubleClick(_label: string, id: string): void {
    if (!expanded) return;

    const alreadyExpanded = expanded.find((item) => item === id);

    if (alreadyExpanded) {
      setExpanded((prev) => [...prev.filter((item) => item !== id)]);
      return;
    }

    if (id) {
      setExpanded((prev) => [...prev, id]);
    }
  }

  /**
   * Callback when folders on the tree are single-clicked by the user
   */
  const handleFolderClick = useCallback(
    (_label: string, id: string) => {
      if (!expanded) return;

      const alreadyExpanded = expanded.find((item) => item === id);

      if (alreadyExpanded) {
        setExpanded((prev) => [...prev.filter((item) => item !== id)]);
        return;
      }

      if (id) {
        setExpanded((prev) => [...prev, id]);
      }
    },
    [expanded]
  );

  /**
   * Callback when files on the tree are double-clicked by the user
   */
  const handleFileDoubleClick = useCallback(
    (label: string, id: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => {
      if (!label || !id) return;
      if (onFileDoubleClick) onFileDoubleClick(label, id, ctrlKey, shiftKey, altKey);
    },
    [onFileDoubleClick]
  );

  /**
   * Build the tree structure for files and directories
   */
  function buildPackageTree(treeItems: TPackageTreeItem[], pathId: string): JSX.Element[] {
    const folders = treeItems.filter((item) => item.children && item.children.length > 0);
    folders.sort((a, b) => {
      return a.name.localeCompare(b.name);
    });
    const jsxFolders = folders.map((group) => {
      const newPathId = `${pathId}#${group.name}`;
      return (
        // valid children means that item is a directory
        <FolderTreeItem
          key={newPathId}
          itemId={newPathId}
          directoryName={group.name.replace("/", "")}
          path={""}
          onDoubleClick={(directoryName: string, itemId: string) => handleFolderDoubleClick(directoryName, itemId)}
          onClick={(directoryName: string, id: string) => handleFolderClick(directoryName, id)}
        >
          {buildPackageTree(group.children, newPathId)}
        </FolderTreeItem>
      );
    });

    const files = treeItems.filter((item) => item.file && item.children.length === 0);
    files.sort((a, b) => {
      return a.name.localeCompare(b.name);
    });
    // we do not need to sort files, they are sorted while get files
    const jsxFiles = files.map((item) => {
      if (item.file) {
        // no children means that [item.value] is a file
        return (
          <FileTreeItem
            key={`${pathId}#${item.file.id}`}
            itemId={`${item.file.id}`}
            file={item.file}
            showPackage={item.appendPackageName}
            onDoubleClick={(labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) =>
              handleFileDoubleClick(labelText, itemId, ctrlKey, shiftKey, altKey)
            }
          />
        );
      }
      console.error("Invalid item in package tree", item);
      return <div key={`${pathId}#${generateUniqueId()}`} />;
    });
    return [...jsxFolders, ...jsxFiles];
  }

  /**
   * Memoize the generation of the tree to improve render performance
   * The idea is to prevent rerendering when scrolling/focusing the component
   */
  const generateTree = useMemo(() => {
    return (
      <SimpleTreeView
        aria-label="package list"
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // use either the expanded state or the key of the file directory (expand the first layer)
        expandedItems={expanded.length > 0 ? expanded : Object.keys(packageItemsTree)}
        selectedItems={selectedItems}
        onExpandedItemsChange={(event: React.SyntheticEvent | null, itemIds: string[]) => handleToggle(event, itemIds)}
        onSelectedItemsChange={(event: React.SyntheticEvent | null, itemIds: string | null) =>
          handleSelect(event, itemIds)
        }
        multiSelect={false}
      >
        {Object.entries(packageItemsTree).map(([packageName, packageTree]) => {
          return packageTree.map((itemTree) => {
            const { name, children, file, appendPackageName } = itemTree;
            if (file && children && children.length === 0) {
              // The root item is a launch file from the saved history.

              return (
                <FileTreeItem
                  key={`${packageName}#${file.id}`}
                  itemId={`${file.id}`}
                  file={file}
                  showPackage={true}
                  onDoubleClick={(
                    labelText: string,
                    itemId: string,
                    ctrlKey: boolean,
                    shiftKey: boolean,
                    altKey: boolean
                  ) => handleFileDoubleClick(labelText, itemId, ctrlKey, shiftKey, altKey)}
                />
              );
            }
            if (appendPackageName && name) {
              return (
                <HistoryGroupItem
                  key={name}
                  itemId={name}
                  providerName={name}
                >
                  {itemTree?.children && buildPackageTree(itemTree.children, packageName)}
                </HistoryGroupItem>
              );
            }
            return (
              <PackageTreeItem
                key={packageName}
                itemId={packageName}
                packageName={packageName}
                providerName={providerName}
                path={selectedPackage?.path as string}
                exists={!!itemTree}
              >
                {itemTree?.children && buildPackageTree(itemTree.children, packageName)}
              </PackageTreeItem>
            );
          });
        })}
      </SimpleTreeView>
    );
  }, [
    expanded,
    packageItemsTree,
    selectedItems,
    selectedPackage,
    providerName,
    // handleToggle,
    // handleSelect,
    // handleFileDoubleClick,
    // buildTreePackageItems,
  ]);

  return generateTree;
}
