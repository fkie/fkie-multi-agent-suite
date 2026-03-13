import { languages } from "monaco-editor";

export type TTagAttributeProposals = {
  tag: string;
  proposals: languages.CompletionItem[];
};
