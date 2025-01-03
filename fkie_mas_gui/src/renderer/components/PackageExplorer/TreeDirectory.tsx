import { RosPackage } from "@/renderer/models";
import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import { SimpleTreeView } from "@mui/x-tree-view";
import { forwardRef, LegacyRef, useCallback, useEffect, useMemo, useState } from "react";
import { generateUniqueId } from "../../utils";
import FileTreeItem from "./FileTreeItem";
import FolderTreeItem from "./FolderTreeItem";
import PackageTreeItem from "./PackageTreeItem";
import { TPackageItemsTree, TPackageTreeItem } from "./types";

interface TreeDirectoryProps {
  packageItemsTree: TPackageItemsTree;
  selectedPackage: RosPackage | null;
  onNodeSelect: (itemId: string) => void;
  onFileDoubleClick: (label: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) => void;
}

const TreeDirectory = forwardRef<HTMLDivElement, TreeDirectoryProps>(function TreeDirectory(props, ref) {
  const { packageItemsTree, selectedPackage = null, onNodeSelect = () => {}, onFileDoubleClick = () => {} } = props;

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
    (_event: React.SyntheticEvent, itemId: string | null) => {
      setSelectedItems(itemId);
      onNodeSelect(itemId ? itemId : "");
    },
    [onNodeSelect]
  );

  /**
   * Callback when folders on the tree are double-clicked by the user
   */
  const handleFolderDoubleClick = useCallback(
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
  const buildTreePackageItems = useCallback(
    (packageName, pathId: string, treeItem: TPackageTreeItem) => {
      if (!treeItem) {
        console.error("Invalid item ", packageName, treeItem);
        return <div key={`${pathId}#${generateUniqueId()}`} />;
      }
      const { name, children, file } = treeItem;

      if (file && children && children.length === 0) {
        // no children means that [item.value] is a file
        return (
          <FileTreeItem
            key={`${pathId}#${file.id}`}
            itemId={`${file.id}`}
            file={file}
            onDoubleClick={(labelText: string, itemId: string, ctrlKey: boolean, shiftKey: boolean, altKey: boolean) =>
              handleFileDoubleClick(labelText, itemId, ctrlKey, shiftKey, altKey)
            }
          />
        );
      }

      const newPathId = `${pathId}#${name}`;
      return (
        // valid children means that item is a directory
        <FolderTreeItem
          key={newPathId}
          itemId={newPathId}
          directoryName={name.replace("/", "")}
          path={""}
          onDoubleClick={(directoryName: string, itemId: string) => handleFolderDoubleClick(directoryName, itemId)}
          onClick={(directoryName: string, id: string) => handleFolderClick(directoryName, id)}
        >
          {children.map((tItem) => {
            return buildTreePackageItems(packageName, newPathId, tItem);
          })}
        </FolderTreeItem>
      );
    },
    [handleFolderDoubleClick, handleFileDoubleClick, handleFolderClick]
  );

  /**
   * Memoize the generation of the tree to improve render performance
   * The idea is to prevent rerendering when scrolling/focusing the component
   */
  const generateTree = useMemo(() => {
    return (
      <SimpleTreeView
        ref={ref as LegacyRef<HTMLUListElement>}
        aria-label="package list"
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // use either the expanded state or the key of the file directory (expand the first layer)
        expandedItems={expanded.length > 0 ? expanded : Object.keys(packageItemsTree)}
        selectedItems={selectedItems}
        onExpandedItemsChange={(event: React.SyntheticEvent, itemIds: string[]) => handleToggle(event, itemIds)}
        onSelectedItemsChange={(event: React.SyntheticEvent, itemIds: string | null) => handleSelect(event, itemIds)}
        multiSelect={false}
      >
        {Object.entries(packageItemsTree).map(([packageName, packageTree]) => {
          return packageTree.map((itemTree) => {
            const { children, file } = itemTree;
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
            return (
              <PackageTreeItem
                key={packageName}
                itemId={packageName}
                packageName={packageName}
                path={selectedPackage?.path as string}
                exists={itemTree ? true : false}
              >
                {itemTree &&
                  itemTree.children &&
                  itemTree.children.map((item) => {
                    return buildTreePackageItems(packageName, packageName, item);
                  })}
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
    // handleToggle,
    // handleSelect,
    // handleFileDoubleClick,
    // buildTreePackageItems,
  ]);

  return generateTree;
});

export default TreeDirectory;
