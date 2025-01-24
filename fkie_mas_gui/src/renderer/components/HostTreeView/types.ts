import { RosNode } from "@/renderer/models";

export type NodeTree = {
  nodeTree: NodeTreeItem[];
};

export type KeyTreeItem = {
  key: string;
  idGlobal: string | undefined;
};

export type NodeTreeItem = {
  treePath: string;
  children: NodeTreeItem[];
  node: RosNode | null;
  providerId: string | undefined;
  providerName: string | undefined;
  name: string;
};
