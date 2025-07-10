import { PathItem } from "@/renderer/models";

export type TPackageItemsTree = { [name: string]: TPackageTreeItem[] };

export type TPackageTree = {
  packageTree: TPackageTreeItem[];
};

export type TPackageTreeItem = {
  // treePath: string;
  children: TPackageTreeItem[];
  file: PathItem | undefined;
  name: string;
  isDirectory: boolean;
  appendPackageName: boolean;
};
