import { Monaco } from "@monaco-editor/react";
import { editor, languages } from "monaco-editor";

import { RosPackage } from "@/renderer/models";
import { TFileRange } from "@/types";
import { getTagAttributeProposals, getTagProposals } from "./Proposals/ProposalsR2";

export async function createXMLDependencyProposalsR2(
  monaco: Monaco | null,
  range: TFileRange,
  lineContent: string,
  packages: RosPackage[]
): Promise<languages.CompletionItem[]> {
  if (!monaco) return [];
  // List of suggestions
  return [
    ...(await createAttributeSuggestionsFromTag(monaco, range, lineContent)),
    ...(await getTagProposals(monaco, range, lineContent)),
    ...createPackageList(packages, monaco, range),
  ];
}

function createPackageList(packages: RosPackage[], monaco: Monaco, range: TFileRange): languages.CompletionItem[] {
  const result = packages?.map((item: RosPackage) => {
    return {
      label: `${item.name}`,
      kind: monaco.languages.CompletionItemKind.Field,
      insertText: `${item.name}`,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range,
    };
  });
  return result ? result : [];
}

async function createAttributeSuggestionsFromTag(
  monaco: Monaco,
  range: TFileRange,
  lineContent: string
): Promise<languages.CompletionItem[]> {
  const tags = [
    "launch",
    "include",
    "group",
    "let",
    "arg",
    "executable",
    "node",
    "param",
    "remap",
    "env",
    "set_env",
    "unset_env",
    "push_ros_namespace",
    "timer",
    "node_container",
    "load_composable_node",
    "composable_node",
  ];

  for (const tag in tags) {
    // if a tag is found, add its attributes as suggestions
    if (lineContent.includes(`<${tags[tag]}`))
      return (await getTagAttributeProposals(monaco, range)).find((item) => item.tag === tags[tag])?.proposals || [];
  }
  // otherwise return nothing
  return [];
}

export function createDocumentSymbolsR2(model: editor.ITextModel /*, token*/): languages.DocumentSymbol[] {
  const parser = new DOMParser();

  const symbolList: languages.DocumentSymbol[] = [];

  for (let lineNumber = 1; lineNumber <= model.getLineCount(); lineNumber += 1) {
    const lineContent = model.getLineContent(lineNumber);

    if (lineContent.length > 0) {
      const xml = parser.parseFromString(lineContent, "text/xml");

      // Add nodes as symbols
      const nodeElement = xml.querySelector("node");
      const nodeName = nodeElement?.getAttribute("name");
      if (nodeName) {
        const packageName = nodeElement?.getAttribute("pkg");
        const typeName = nodeElement?.getAttribute("exec");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<node> ${nodeName} exec="${typeName}" pkg="${packageName}"`,
          kind: 5,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add parameters as symbols
      const parameterElement = xml.querySelector("param");
      const parameterName = parameterElement?.getAttribute("name");
      if (parameterName) {
        const parameterValue = parameterElement?.getAttribute("value");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<param> ${parameterName}: ${parameterValue}`,
          kind: 13,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add arguments as symbols
      const argumentElement = xml.querySelector("arg");
      const argumentName = argumentElement?.getAttribute("name");
      if (argumentName) {
        const argumentValue = argumentElement?.getAttribute("value");
        const defaultValue = argumentElement?.getAttribute("default");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<arg> ${argumentName}: ${argumentValue ? argumentValue : defaultValue}`,
          kind: 13,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add lets as symbols
      const letElement = xml.querySelector("let");
      const letName = letElement?.getAttribute("name");
      if (letName) {
        const letValue = letElement?.getAttribute("value");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<let> ${letName}: ${letValue}`,
          kind: 14,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add set_env as symbols
      const setEnvElement = xml.querySelector("set_env");
      const setEnvName = setEnvElement?.getAttribute("name");
      if (setEnvName) {
        const value = setEnvElement?.getAttribute("value");
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<set_env> ${setEnvName}: ${value}`,
          kind: 12,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }

      // Add unset_env as symbols
      const unsetEnvElement = xml.querySelector("unset_env");
      const unsetEnvName = unsetEnvElement?.getAttribute("name");
      if (unsetEnvName) {
        symbolList.push({
          range: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
          name: `<unset_env> ${unsetEnvName}`,
          kind: 12,
          detail: "",
          containerName: "",
          tags: [],
          selectionRange: {
            startLineNumber: lineNumber,
            startColumn: 1,
            endLineNumber: lineNumber,
            endColumn: 1,
          },
        });
      }
    }
  }

  return symbolList;
}
