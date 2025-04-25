import { languages } from "monaco-editor/esm/vs/editor/editor.api";

export type TTagAttributeProposals = {
  tag: string;
  proposals: languages.CompletionItem[];
};
