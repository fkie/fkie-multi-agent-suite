import { Range } from "monaco-editor";

import { LaunchIncludedFile } from "@/renderer/models";

type TLaunchIncludeItem = {
  children: TLaunchIncludeItem[];
  uriPath: string;
  file: LaunchIncludedFile;
};

type TSearchResult = {
  file: string;
  text: string;
  lineNumber: number;
  range: Range;
};

export type { TLaunchIncludeItem, TSearchResult };
