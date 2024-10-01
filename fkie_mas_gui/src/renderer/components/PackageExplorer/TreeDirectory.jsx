import ArrowDropDownIcon from "@mui/icons-material/ArrowDropDown";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import FolderOutlinedIcon from "@mui/icons-material/FolderOutlined";
import { blue, red } from "@mui/material/colors";
import { SimpleTreeView } from "@mui/x-tree-view";
import PropTypes from "prop-types";
import { useCallback, useEffect, useMemo, useState } from "react";
import { FileIcon } from "react-file-icon";
import { LAUNCH_FILE_EXTENSIONS } from "../../context/SettingsContext";
import { getFileExtension } from "../../models";
import { generateUniqueId } from "../../utils";
import defaultFileIconStyles from "./FileIconDefaultStyles";
import PackageItemTree from "./PackageItemTree";

function TreeDirectory({
  packageItemsTree,
  selectedPackage = {},
  onNodeSelect = () => {},
  onFileDoubleClick = () => {},
}) {
  const [expanded, setExpanded] = useState([]);
  const [selectedItems, setSelectedItems] = useState([]);

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
  const handleToggle = useCallback((event, nodeIds) => {
    setExpanded(nodeIds);
  }, []);

  /**
   * Callback when items on the tree are selected by the user
   */
  const handleSelect = useCallback(
    (event, itemId) => {
      setSelectedItems([itemId]);

      if (onNodeSelect) {
        onNodeSelect(event, itemId);
      }
    },
    [onNodeSelect]
  );

  /**
   * Callback when folders on the tree are double-clicked by the user
   */
  const handleFolderDoubleClick = useCallback(
    (label, id) => {
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
    (label, id) => {
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
    (label, id, ctrlKey, shiftKey, altKey) => {
      if (!label || !id) return;
      if (onFileDoubleClick) onFileDoubleClick(label, id, ctrlKey, shiftKey, altKey);
    },
    [onFileDoubleClick]
  );

  /**
   * Build the tree structure for files and directories
   */
  const buildTreePackageItems = useCallback(
    (packageName, pathId, treeItem) => {
      if (!treeItem) {
        console.error("Invalid item ", packageName, treeItem);
        return <div key={`${pathId}#${generateUniqueId()}`} />;
      }
      const { directoryName, children, file } = treeItem;

      if (file && children && children.length === 0) {
        // no children means that [item.value] is a file
        const fileExtension = getFileExtension(file.name);
        const isLaunchFile = LAUNCH_FILE_EXTENSIONS.find((fe) => file.path.endsWith(fe));
        return (
          <PackageItemTree
            key={`${pathId}#${file.id}`}
            itemId={`${file.id}`}
            labelText={file.name.replace("/", "")}
            tooltip={isLaunchFile ? file.path : ""}
            // enableCopy={!!isLaunchFile}
            enableCopy={false}
            labelIconComponent={
              <FileIcon extension={fileExtension} radius={10} {...defaultFileIconStyles[fileExtension]} />
            }
            iconColor={blue[700]}
            color={blue[700]}
            bgColor={blue[200]}
            paddingLeft={0.0}
            onDoubleClick={handleFileDoubleClick}
          />
        );
      }

      const newPathId = `${pathId}#${directoryName}`;
      return (
        // valid children means that item is a directory
        <PackageItemTree
          key={newPathId}
          itemId={newPathId}
          labelText={directoryName.replace("/", "")}
          labelIcon={FolderOutlinedIcon}
          enableCopy={false}
          iconColor={blue[700]}
          color={blue[700]}
          bgColor={blue[200]}
          paddingLeft={0.5}
          onDoubleClick={handleFolderDoubleClick}
          onClick={handleFolderClick}
        >
          {children.map((tItem) => {
            return buildTreePackageItems(packageName, newPathId, tItem);
          })}
        </PackageItemTree>
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
        aria-label="package list"
        slots={{ collapseIcon: ArrowDropDownIcon, expandIcon: ArrowRightIcon }}
        // use either the expanded state or the key of the file directory (expand the first layer)
        expandedItems={expanded.length > 0 ? expanded : Object.keys(packageItemsTree)}
        selectedItems={selectedItems}
        onExpandedItemsChange={handleToggle}
        onSelectedItemsChange={handleSelect}
        multiSelect={false}
      >
        {Object.entries(packageItemsTree).map(([packageName, itemTree]) => {
          const { children, file } = itemTree;
          if (file && children && children.length === 0) {
            // The root item is a launch file from the saved history.
            const fileExtension = getFileExtension(file.name);
            return (
              <PackageItemTree
                key={`${packageName}#${file.id}`}
                itemId={`${file.id}`}
                labelText={`${file.name.replace("/", "")} [${file.package}]`}
                labelIconComponent={
                  <FileIcon extension={fileExtension} radius={10} {...defaultFileIconStyles[fileExtension]} />
                }
                tooltip={file.path}
                enableCopy={false}
                iconColor={blue[700]}
                color={blue[700]}
                bgColor={blue[200]}
                paddingLeft={0.0}
                onDoubleClick={handleFileDoubleClick}
              />
            );
          }
          return (
            <PackageItemTree
              key={packageName}
              itemId={packageName}
              labelText={packageName}
              tooltip={selectedPackage?.path}
              // enableCopy={!!selectedPackage}
              enableCopy={false}
              iconColor={itemTree ? blue[700] : red[700]}
              color={itemTree ? blue[700] : red[700]}
              bgColor={itemTree ? blue[200] : red[200]}
              paddingLeft={0.5}
            >
              {itemTree &&
                itemTree.children &&
                itemTree.children.map((item) => {
                  return buildTreePackageItems(packageName, packageName, item);
                })}
            </PackageItemTree>
          );
        })}

        {/* this box creates an empty space at the end, to prevent items to be covered by app bar */}
        {/* <Box sx={{ height: '6em', width: '100%' }} /> */}
      </SimpleTreeView>
    );
  }, [
    expanded,
    packageItemsTree,
    selectedItems,
    handleToggle,
    handleSelect,
    selectedPackage,
    handleFileDoubleClick,
    buildTreePackageItems,
  ]);

  return generateTree;
}

TreeDirectory.propTypes = {
  packageItemsTree: PropTypes.any.isRequired,
  selectedPackage: PropTypes.any,
  onNodeSelect: PropTypes.func,
  onFileDoubleClick: PropTypes.func,
};

export default TreeDirectory;
