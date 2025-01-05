import { LaunchIncludedFile } from "@/renderer/models";
import { Range } from "monaco-editor/esm/vs/editor/editor.api";

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
